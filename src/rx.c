#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include <pthread.h>

#include <signal.h>
#include <fcntl.h>
#include <sched.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <sys/time.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>


#include "si4463.h"

#include "suit.h"

#include "fec.h"

#include "aes_crypto.h"

static int receiver(int write_fd, si4463_t si, int using_fec);
static void fec_recovery_data(int fd, int oobfd, size_t packet_size);

static inline void clean_data_stripe();
static inline int data_write(int write_fd, int write_oob, U8* buffer);


static int use_fec = 0;
static int use_aes = 0;

static volatile int quit = 0; // Flags should be set in signal handler

#if USE_STAT_FILE
FILE* statf = NULL; // DEBUG
#endif

static size_t packet_size = 0;
static size_t data_size = 0;

static int SPI_Channel = 0;
static int SPI_Speed = MAX_SPEED;
static int GPIO_PowerDown = 0;
static int GPIO_gpio1 = 3;
static int GPIO_interrupt = 2;
static int RADIO_channel = 0;
static int Modulation = 1;

#define POOL_SIZE 2*(MAX_DATA_PACKETS_PER_BLOCK+MAX_FEC_PACKETS_PER_BLOCK)

static U8 data_pool[POOL_SIZE][MAX_USER_PACKET_LENGTH];
static int pool_item = 0;

static U8* data_blocks[MAX_DATA_PACKETS_PER_BLOCK];
static U8* fec_blocks[MAX_FEC_PACKETS_PER_BLOCK];

static int data_stripe   = 0;
static int fec_stripe    = 0;
static int last_data_written_block = 0;
static int missed = 0;

static long long int packet_counter = 0;
static long long int lost_packet_counter = 0;
static long long int bad_crc_packet_counter = 0;

static int pipes[2];
static int fd = -1;
static int oob_fd = -1;
static int oob_fd_f = -1;

static struct ctx_data ctx;
static unsigned long long int session_id = 0;

static char* key = NULL;
static size_t key_len = 0;

static char* salt = NULL;
static size_t salt_len = 0;

static void quit_handler(int sig)
{
    quit = 1;
}

static inline U8* get_pool_item()
{
    if (pool_item >= POOL_SIZE)
        pool_item = 0;

    return data_pool[pool_item++];
}

static void set_real_time()
{
    cpu_set_t zset;
    struct sched_param schedule;

    schedule.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (0 != sched_setscheduler(0, SCHED_FIFO, &schedule))
    {
        perror("Set prio failed:");
        return;
    }

    CPU_ZERO(&zset); 
    CPU_SET(0, &zset);     /* set the bit that represents core 0. */
    if (0 != sched_setaffinity(0, sizeof(cpu_set_t), &zset))
    {
        perror("Set affinity failed:");
        return;
    }

}

static inline 
char c2b(char f)
{
    switch(toupper(f))
    {
        case '0':
            return 0;
        case '1':
            return 1;
        case '2':
            return 2;
        case '3':
            return 3;
        case '4':
            return 4;
        case '5':
            return 5;
        case '6':
            return 6;
        case '7':
            return 7;
        case '8':
            return 8;
        case '9':
            return 9;
        case 'A':
            return 0xa;
        case 'B':
            return 0xb;
        case 'C':
            return 0xc;
        case 'D':
            return 0xd;
        case 'E':
            return 0xe;
        case 'F':
            return 0xf;
    }
    fprintf(stderr, "Error: bad hex char\n");
    return -1;
}

static
int hex_to_bin(const char* hex, char**bin, size_t* res_len)
{
    const char* b = hex;
    char l,h;
    char *buf = NULL;

    int i = 0;

    *res_len = strlen(hex)/2;

    *bin = buf = malloc(*res_len);
    
    while (b[0] != 0)
    {
        if (b[1] == 0)
        {
            fprintf(stderr, "Error: bad hex length");
            return -1;
        }
        h =  c2b(b[0]);
        l =  c2b(b[1]);

        if (h == -1 || l == -1)
        {
            free(bin);
            *bin = NULL;
            *res_len = 0;
            return -1;
        }
        buf[i] = (h << 4) + l;
        ++i;
        b += 2;
    }

    return 0;
}

static inline
uint16_t get_cseq(const U8* packet)
{
    uint16_t cseq = packet[0] & 0x3F;
    cseq <<= 8;
    cseq += packet[1];

    return cseq;
}


static inline
void clean_data_stripe()
{
    for(int i = 0; i < MAX_DATA_PACKETS_PER_BLOCK; ++i)
    {
        data_blocks[i] = 0x0;
    }
    data_stripe = 0;
    last_data_written_block = 0;
    missed = 0;
}

static inline
void clean_fec_stripe()
{
    for (int i = 0; i < MAX_FEC_PACKETS_PER_BLOCK; ++i)
    {
        fec_blocks[i] = 0x0;
    }
    fec_stripe = 0;
}

U8* store_fec_packet(int fd, int oobfd, U8* packet, size_t packet_size, uint16_t missed_packets)
{
    if (fec_stripe >= MAX_FEC_PACKETS_PER_BLOCK) // Looks like it is not correct
    {
        fec_recovery_data(fd, oobfd, packet_size);
    }

    fec_blocks[fec_stripe++] = packet;

    if (missed == 1 && (fec_stripe >= MAX_FEC_PACKETS_PER_BLOCK))
    {
        fec_recovery_data(fd, oobfd, packet_size);
    }
    else if (fec_stripe >= MAX_FEC_PACKETS_PER_BLOCK)
    {
        clean_data_stripe();
        clean_fec_stripe();
    }

    return get_pool_item();
}

static inline uint16_t fill_in_missed(uint16_t missed_packets)
{
    for (; (missed_packets > 0) && (data_stripe < MAX_DATA_PACKETS_PER_BLOCK); missed_packets--)
    {
        data_blocks[data_stripe++] = 0x0;
        missed = 1;
    }
    
    for (; (missed_packets > 0) && (fec_stripe < MAX_FEC_PACKETS_PER_BLOCK); missed_packets--)
    {
        fec_blocks[fec_stripe++] = 0x0;
    }

    return missed_packets;
}


U8* process_fec_data(int fd, int oobfd, uint8_t *data, size_t packet_size, uint16_t missed_packets)
{
    uint16_t left = fill_in_missed(missed_packets);
    
    if (left > 0)
    {
        fec_recovery_data(fd, oobfd, packet_size);
        do 
        {
            clean_fec_stripe();
            clean_data_stripe();
            left = fill_in_missed(left);
        }
        while(left > 0);
    }

    if (data[0] & FEC_PACKET_MASK) // FEC packet
    {
        return store_fec_packet(fd, oobfd, data, packet_size, missed_packets);
    }

    if (!missed)
    {
        ++last_data_written_block;
        data_write(fd, oobfd, data);
    }
    if (data_stripe>= MAX_DATA_PACKETS_PER_BLOCK)
    {
        // At least one fec missed, do recovery
        fec_recovery_data(fd, oobfd, packet_size);
    }

    data_blocks[data_stripe++] = data;

    return get_pool_item();
}

void fec_recovery_data(int fd, int oobfd, size_t packet_size)
{
    U8 *r_blocks[MAX_DATA_PACKETS_PER_BLOCK];
    U8 *f_blocks[MAX_FEC_PACKETS_PER_BLOCK];
    unsigned int erased_blocks[MAX_DATA_PACKETS_PER_BLOCK];
    unsigned int fec_block_nos[MAX_FEC_PACKETS_PER_BLOCK];

    int recovering_possible = 1;
    int i;
    int fec_pos = 0;
    int nr_fec_blocks = 0;
    
    for (i = 0; i < MAX_DATA_PACKETS_PER_BLOCK; i++) 
    {
        if (data_blocks[i] == 0x0)
        {
            if (fec_blocks[fec_pos] == 0x0)
            {
                // First FEC missed, find next
                while (fec_blocks[++fec_pos] == 0x0 && fec_pos < MAX_FEC_PACKETS_PER_BLOCK);
                if (fec_pos == MAX_FEC_PACKETS_PER_BLOCK)
                {
                    fprintf(stderr, "Cannot recover data, lost too much fec packets\n");
                    recovering_possible = 0;
                    break;
                }
            }
            data_blocks[i] = get_pool_item();

            f_blocks[nr_fec_blocks] = fec_blocks[fec_pos] + CSEQ_DATA_SIZE;

            erased_blocks[nr_fec_blocks] = i;
            fec_block_nos[nr_fec_blocks] = fec_pos;

            if (fec_pos < MAX_FEC_PACKETS_PER_BLOCK - 1)
            {
                fec_pos++;
                nr_fec_blocks++;
            }
        }
        r_blocks[i] = data_blocks[i] + CSEQ_DATA_SIZE;
    }

    if (recovering_possible && nr_fec_blocks > 0)
    {
        fec_decode(packet_size,
                   r_blocks, 
                   MAX_DATA_PACKETS_PER_BLOCK,
                   f_blocks, 
                   fec_block_nos, 
                   erased_blocks, 
                   nr_fec_blocks);
    }

    for (int i = last_data_written_block; i < MAX_DATA_PACKETS_PER_BLOCK; ++i)
        data_write(fd, oobfd, data_blocks[i]);

    clean_data_stripe();
    clean_fec_stripe();
}

static 
int decrypt_packet(U8* packet)
{
    U8 tmp[packet_size];
    size_t len = packet_size - CSEQ_DATA_SIZE;
    uint16_t cseq = get_cseq(packet);

    if (-1 == update_iv(&ctx, session_id, cseq))
    {
        return -1;
    }

    if (-1 == aes_decrypt(&ctx, packet + CSEQ_DATA_SIZE, len, tmp))
    {
        return -1;
    }

    memcpy(packet + CSEQ_DATA_SIZE, tmp, len);

    return 0;
}

int data_write(int write_fd, int write_oob, U8* buffer)
{
    int ret = 0;
    size_t oob_sz = 0;

    if (buffer == 0x0)
        return 0;


    if (use_aes)
        decrypt_packet(buffer);

    size_t pkt_sz = buffer[2] & PACKET_SIZE_MASK;

    if (pkt_sz > data_size) // ERROR in data, try to fix it... hope lost only a few bytes
        pkt_sz = data_size;

    int oob = ((write_oob != -1) && (buffer[0] & OOB_DATA_MASK));

    size_t start  = oob ? PACKET_PREFIX_SIZE + 1 : PACKET_PREFIX_SIZE;
    if (oob)
    {
        oob_sz = buffer[PACKET_PREFIX_SIZE] & PACKET_SIZE_MASK;
        pkt_sz -= oob_sz;
    }

    ret = write(write_fd, buffer + start, pkt_sz);
    if (oob)
    {
        start = PACKET_PREFIX_SIZE + 1 + pkt_sz;

        if (start + oob_sz <= packet_size) 
        {
            int ret2 = write(write_oob, buffer + start, oob_sz);
            if (ret2 == -1)
                fprintf(stderr, "OOB data write failed, error %d\n", errno);
        }
    }

    return ret;
}

int receiver(int write_fd, si4463_t si, int using_fec)
{
    U8 buffer[MAX_USER_PACKET_LENGTH];
    int total_data = 0;
    int ret;

    // Start RX
    SI4463_rx_mode(si, RADIO_channel);
    int pos = 0;    

    while (! quit)
    {
        if (0 < (ret = SI4463_check_rx(si, buffer + pos, packet_size - pos)))
        {
            pos += ret;
            if (pos >= packet_size)
            {
                write(pipes[1], buffer, packet_size);
                ++packet_counter;

                //eof = buffer[0] & EOF_PACKET_MASK;
                total_data += packet_size;
                pos = 0;
            }
        }
        else if (ret == -2) // CRC error
        {
//            ++cseq;
            ++packet_counter;
            ++bad_crc_packet_counter;
        }     
        else if (ret < 0)
        {
            fprintf(stderr,  "ERROR, code return  %d\n", ret);
//            break;
        }
    }

    close(pipes[1]);

    return total_data;
}


void* thread_function(void* arg)
{
    int pkt_sz = 0;
    int eof = 0;
    uint16_t cseq = 0;
    uint16_t missed_packets = 0;

    int ret = 1;
    uint16_t cseq_new;
    U8* buffer;

    buffer = get_pool_item();
    while (! quit)
    {
        ret = read(pipes[0], buffer, packet_size);
        if (ret < 0)
        {
            perror("Pipe write:");
            continue;
        }
        cseq_new = get_cseq(buffer);

        if ((use_aes) && cseq == cseq_new)
            continue;

        if ((1 + cseq) < cseq_new)
        {
            if ((((cseq_new >> 8) & 0x30) == 0x0) && ((cseq >> 8) & 0x30))
            {
                missed_packets = 0x3FFF-cseq + cseq_new-1;
            }
            else
            {
                missed_packets = cseq_new - cseq - 1;
            }
            lost_packet_counter += missed_packets;
        }
        
        cseq = cseq_new;
#if USE_STAT_FILE
        if (statf)  // DEBUG
        {
            struct timeval tv;  // DEBUG
            gettimeofday(&tv, NULL);  // DEBUG
            fprintf(statf, "%lu:\t%u\t%lld\t%d\n", tv.tv_sec, cseq, lost_packet_counter, use_fec); // DEBUG
        }
#endif

        if (use_fec)
        {
            buffer = process_fec_data(fd, oob_fd, buffer, packet_size, missed_packets);
            missed_packets = 0;
        }
        else
            if  (-1 == data_write(fd, oob_fd, buffer))
            {
                perror("write error:");
                break;
            }
    }

    close(pipes[0]);

    return NULL;
}

void help(const char* name)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "\t%s [kKcsSpgirfmFhE]\n", name);
    fprintf(stderr, "\tSPI configuration:\n");
    fprintf(stderr, "\t\t-c [0|1] \t defines SPI channel\n");
    fprintf(stderr, "\t\t-s [%d..%d] \t defines SPI bus speed, kHz\n", MIN_SPEED, MAX_SPEED);
    fprintf(stderr, "\tGPIO configuration:\n");
    fprintf(stderr, "\t\t-p PIN \t defines RPi pin connected to power down control GPIO pin\n");
    fprintf(stderr, "\t\t-g PIN \t defines RPi pin connected to GPIO1 of Si4463\n");
    fprintf(stderr, "\t\t-i PIN \t defines RPi pin connected to interrupt output pin\n");
    fprintf(stderr, "\tRadio configuration:\n");
    fprintf(stderr, "\t\t-r CHANNEL \t defines radio channel\n");
    fprintf(stderr, "\t\t-m [%d,%d] \t defines modulation type, supported 2fsk (1), 2gfsk (2),"
                    " 4fsk (3) or 4gfsk (4)\n", 
                    MIN_MODULATION, MAX_MODULATION);
    fprintf(stderr, "\tEncryption:\n");
    fprintf(stderr, "\t\t-E use encryption\n");
    fprintf(stderr, "\t\t-S session ID\n");
    fprintf(stderr, "\t\t-k define key as hex string, key should be 256 bits\n");
    fprintf(stderr, "\t\t-K define salt as hex string\n");
    fprintf(stderr, "\tMiscellaneous:\n");
    fprintf(stderr, "\t\t-F use FEC\n");
    fprintf(stderr, "\t\t-f FILE \t defines input/output file, use '-' for stdin/stdout\n");
    fprintf(stderr, "\t\t-u FILE \t defines console output file\n");
    fprintf(stderr, "\t\t-U FILE \t creates FIFO to be used as console output file\n");
    fprintf(stderr, "\tHelp:\n");
    fprintf(stderr, "\t\t-h \t shows this help\n");
}

static
int open_file(const char* name)
{
    int flag = O_CREAT | O_WRONLY;

    int m = S_IRWXU;

    if (strcmp(name, "-") == 0)
    {
        setvbuf(stdout, NULL, _IONBF, 0);
        return fileno(stdout);
    }

    return open(name, flag, m);
}

int main(int argc, char** argv)
{
    char* file_name = NULL;
    char* oob_file_name = NULL;
    char* fifo_file_name = NULL;
    si4463_t si;
    int c;
    struct SI4463_Part_Info* info;
    opterr = 0;

    signal(SIGINT, quit_handler);
    signal(SIGQUIT, quit_handler);

    while ((c = getopt (argc, argv, "k:K:u:U:c:s:S:p:g:i:r:f:m:FhE")) != -1)
        switch (c)
        {
            case 'h': // help
                help(argv[0]);
                return -1;
            case 'c': // SPI channel
                SPI_Channel = atoi(optarg);
                if (SPI_Channel != 0 && SPI_Channel != 1) 
                {
                    fprintf(stderr, "Wrong spi channel\n\n");
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'k': // key
                if (-1 == hex_to_bin(optarg, &key, &key_len))
                {
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'K': // salt
                if (-1 == hex_to_bin(optarg, &salt, &salt_len))
                {
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'S': // session ID
            {
                char* err = NULL;
                session_id = strtoull(optarg, &err, 16);
                if (err && err[0] != 0)
                {
                    fprintf(stderr, "Error: Bad session ID\n");
                    help(argv[0]);
                    return -1;
                }
            }
            break;
            case 'E': // use encryption
                use_aes = 1;
                init_crypto();
                break;
            case 'f': // input/output file
                file_name = strdup(optarg);
                break;
            case 'u': // input/output file
                oob_file_name = strdup(optarg);
                break;
            case 'U': // input/output FIFO
                fifo_file_name = strdup(optarg);
                break;
            case 'F': // input/output file
                use_fec = 1;
                fec_init();
                break;
            case 'm': // SPI speed
                Modulation = atoi(optarg);
                if (Modulation < MIN_MODULATION || Modulation > MAX_MODULATION)
                {
                    fprintf(stderr, "Wrong modulation type\n\n");
                    help(argv[0]);
                    return -1;
                }
                break;
            case 's': // SPI speed
                SPI_Speed = atoi(optarg);
                if (SPI_Speed < MIN_SPEED || SPI_Speed > MAX_SPEED)
                {
                    fprintf(stderr, "Wrong spi speed\n\n");
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'p': // power down control
                GPIO_PowerDown = atoi(optarg);
                if (GPIO_PowerDown < 0 || GPIO_PowerDown > MAX_PIN)
                {
                    fprintf(stderr, "Wrong GPIO number of the pin connected to power down control GPIO pin\n\n");
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'g': // GPIO1 of Si4463
                GPIO_gpio1 = atoi(optarg);
                if (GPIO_gpio1 < 0 || GPIO_gpio1 > MAX_PIN)
                {
                    fprintf(stderr, "Wrong GPIO number of the pin connected to GPIO1 of Si4463\n\n");
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'i': // interrupt pin
                GPIO_interrupt = atoi(optarg);
                if (GPIO_interrupt < 0 || GPIO_interrupt > MAX_PIN)
                {
                    fprintf(stderr, "Wrong GPIO number of the pin connected to interrupt output pin\n\n");
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'r': // Radio channel
                RADIO_channel = atoi(optarg);
                if (RADIO_channel < 0 || RADIO_channel >32767) 
                {
                    fprintf(stderr, "Wrong radio channel\n\n");
                    help(argv[0]);
                    return -1;
                }
                RADIO_channel *= 2;
                break;

            case '?':
                if (optopt == 'c')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                help(argv[0]);
                return -1;
            default:
                abort ();
        }

    if (use_aes)
    {
        if (-1 == init_ctx_data(&ctx, (unsigned char*)key, key_len, (unsigned char*)salt, salt_len))
        {
            destroy_crypto();
            help(argv[0]);
            return -1;
        }
    }

    if (oob_file_name != NULL && fifo_file_name != NULL)
    {
        fprintf(stderr, "only one option should be used, can use -u or -U but not both");
        return 1;
    }

    if (oob_file_name != NULL)
    {
         if (strcmp(oob_file_name, "-") == 0)
         {
             fprintf(stderr, "wrong file name for console");
             return 1;
         }
         oob_fd = open_file(oob_file_name);
         if (oob_fd == -1)
         {
             perror("Open file:");
             return 1;
         }
         free(oob_file_name);
    }

    if (fifo_file_name != NULL)
    {
        int flag = O_CREAT | O_WRONLY | O_NONBLOCK;
        int rflag = O_CREAT | O_RDONLY | O_NONBLOCK;
        int m = S_IRWXU;

        struct stat st;

        if (stat(fifo_file_name, &st) != 0)
            mkfifo(fifo_file_name, 0660);

        oob_fd_f = open(fifo_file_name, rflag, m);
        oob_fd = open(fifo_file_name, flag, m);
        if (oob_fd == -1)
            perror("Open console file:");

    }

    if (file_name == NULL)
    {
        file_name = strdup("-");
    }

    fd = open_file(file_name);
    if (fd == -1)
    {
        perror("Open file:");
        return 1;
    }
    free(file_name);

    wiringPiSetup();

    si = SI4463_init(SPI_Channel, GPIO_gpio1, GPIO_PowerDown, GPIO_interrupt, SPI_Speed);
    if (si == (si4463_t) -1)
    {
        fprintf(stderr, "Device init failed...\n");
        return -1;
    }
    SI4463_reset(si);

    info = SI4463_part_info(si);
    if (info == NULL)
    {
        perror("Info failed!\n");
        return 1;
    }

    if (info->part != SUPPORTED_CHIP_REV)
    {
        fprintf(stderr, "Wrong chip revision or chip is not connected\n");
        fprintf(stderr, 
                "Got  %04X, expected %04X\nTerminating...\n", 
                info->part, 
                SUPPORTED_CHIP_REV);
        free(info);
        return -1;
    }

    free(info);
    info = NULL;

    int cmds;
    switch (Modulation)
    {
        case 1: // fsk
            packet_size = packet_size_2fsk_rx();
            cmds = load_2fsk_config_rx(si);
            break;
        case 2: // gfsk
            packet_size = packet_size_2gfsk_rx();
            cmds = load_2gfsk_config_rx(si);
            break;
        case 3:
            packet_size = packet_size_4fsk_rx();
            cmds = load_4fsk_config_rx(si);
            break;
        case 4:
            packet_size = packet_size_4gfsk_rx();
            cmds = load_4gfsk_config_rx(si);
            break;
        default:
            fprintf(stderr, "Wrong modulation type, exit...\n");
            SI4463_free(si);
            close(fd);
            return -1;
    }
    data_size = packet_size - 1;
    fprintf(stdout, "Loaded %d params\n", cmds);

    if (cmds == -1) 
    {
        SI4463_free(si);
        close(fd);

        return -1;
    }

    int s = pipe2(pipes, O_CLOEXEC | O_DIRECT);
    if (s == -1)
    {
        perror("Pipe:");
        SI4463_free(si);
        close(fd);

        return -1;

    }

#if USE_STAT_FILE
    statf = fopen("stat.log", "a"); //DEBUG
    if (statf)
    {
        fprintf(statf, "Receiver modulation %d\n\nTime\tCSEQ\tLost\tFEC\n", Modulation); // DEBUG
    }
#endif

    set_real_time();

    pthread_t thread_id;

    s = pthread_create(&thread_id, NULL, &thread_function, NULL);
    if (s != 0)
    {
        perror("Thread:");
        SI4463_free(si);
        close(fd);

        return -1;
    }

    receiver(fd, si, use_fec);

    pthread_join(thread_id, NULL);

#if USE_STAT_FILE
    if (statf) {       // DEBUG
        fflush(statf); // DEBUG
        fclose(statf); // DEBUG
        statf = NULL;  // DEBUG
    }                 // DEBUG
#endif

    SI4463_free(si);

    close(fd);

    if (oob_fd != -1)
        close(oob_fd);

    if (oob_fd_f != -1)
        close(oob_fd_f);

    if (fifo_file_name)
    {
        unlink(fifo_file_name);
        free(fifo_file_name);
    }

    if (use_aes)
    {
        destroy_crypto();
    }

    printf("Received %lld packets\n", packet_counter);
    printf("Lost %lld packets\n", lost_packet_counter);
    printf("Bad CRC packets %lld packets\n", bad_crc_packet_counter);
    printf("Done\n");
    return 0;
}
