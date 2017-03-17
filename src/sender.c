#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include <signal.h>
#include <fcntl.h>
#include <sched.h>

#include <poll.h>

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

static int transfer(int read_fd, int read_oob, si4463_t si);
static int sending_data(si4463_t si, const U8* data_array);
static int fec_send_data(si4463_t si, size_t packet_length, U8* data);
static int send_data(si4463_t si, size_t packet_length, U8* data, int using_fec);

static volatile int quit = 0; // Flags should be set in signal handler

static int use_fec = 0;
static int use_aes = 0;

static size_t packet_size = 0;
static size_t data_size = 0;

static int SPI_Channel = 0;
static int SPI_Speed = MAX_SPEED;
static int GPIO_PowerDown = 0;
static int GPIO_gpio1 = 3;
static int GPIO_interrupt = 2;
static int RADIO_channel = 0;

static int Modulation = 1;

static int alowed_queue_size = 0;

static long long int packet_counter = 0;

static struct ctx_data ctx;

static unsigned long long int session_id = 0;

static uint16_t cseq = 0;

static char* key = NULL;
static size_t key_len = 0;

static char* salt = NULL;
static size_t salt_len = 0;


#define POOL_SIZE 2*(MAX_DATA_PACKETS_PER_BLOCK+MAX_FEC_PACKETS_PER_BLOCK)

#if USE_STAT_FILE
FILE* statf = NULL; // DEBUG
#endif

static U8 data_pool[POOL_SIZE][MAX_USER_PACKET_LENGTH];
static int pool_item = 0;
static inline U8* get_pool_item()
{
    if (pool_item >= POOL_SIZE)
        pool_item = 0;

    return data_pool[pool_item++];
}

static void quit_handler(int sig)
{
    quit = 1;
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

static inline uint16_t set_cseq(U8* buffer, int fec)
{
    cseq ++;
    if (cseq >= 0x3fff)
        cseq = 0x1;

    buffer[0] |= fec ? ((cseq >> 8) & 0x3f) | FEC_PACKET_MASK : (cseq >> 8) & 0x3f;

    buffer[1] = cseq & 0x00ff;

    return cseq;
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

int send_data(si4463_t si, size_t packet_length, U8* data, int using_fec)
{
    //uint16_t c_seq = set_cseq(data, 0);

    uint16_t cseq = set_cseq(data, 0);

    if (use_aes)
    {
        if(-1 == update_iv(&ctx, session_id, cseq))
        {
            fprintf(stderr, "Error: Cannot update IV\n");
        }
        else
        {
            unsigned char tmp[packet_length];
            size_t len = packet_length - CSEQ_DATA_SIZE;

            if (-1 == aes_encrypt(&ctx, data+CSEQ_DATA_SIZE, len, tmp))
            {
                fprintf(stderr, "Error: cannot ecrypt data\n");
                return -1;
            }

            memcpy(data+CSEQ_DATA_SIZE, tmp, len);
        }
    }
    if (using_fec)
        return fec_send_data(si, packet_length, data);

    int ret = sending_data(si, data);
    return ret;
}

int fec_send_data(si4463_t si, size_t packet_length, U8* data)
{
    static int stripe = 0;
    
    static uint8_t *data_pool[MAX_DATA_PACKETS_PER_BLOCK];

    uint8_t fec_pool[MAX_FEC_PACKETS_PER_BLOCK][MAX_USER_PACKET_LENGTH] = {{0}};
    uint8_t *data_blocks[MAX_DATA_PACKETS_PER_BLOCK];
    uint8_t *fec_blocks[MAX_FEC_PACKETS_PER_BLOCK];
    
    if (stripe >= MAX_DATA_PACKETS_PER_BLOCK)
    {
        stripe = 0;

        for(int i = 0; i < MAX_DATA_PACKETS_PER_BLOCK; ++i)
        {
            data_pool[i] = 0x0;
        }
    }
    data_pool[stripe] = data;

    int ret = sending_data(si, data);

    if (++stripe == MAX_DATA_PACKETS_PER_BLOCK)
    {
        // calculate FEC data
        for(int i = 0; i < MAX_DATA_PACKETS_PER_BLOCK; ++i)
        {
            data_blocks[i] = data_pool[i]+CSEQ_DATA_SIZE;
        }

        for(int i = 0; i < MAX_FEC_PACKETS_PER_BLOCK; ++i)
        {
            fec_blocks[i] = fec_pool[i]+CSEQ_DATA_SIZE;
            fec_pool[i][0] = 0x0;
            set_cseq(fec_pool[i], 1);
        }

        unsigned int data_packets_per_block = MAX_DATA_PACKETS_PER_BLOCK;
        unsigned int fec_packets_per_block = MAX_FEC_PACKETS_PER_BLOCK;

        fec_encode(packet_length - CSEQ_DATA_SIZE, 
                   data_blocks, 
                   data_packets_per_block, 
                   (unsigned char **)fec_blocks, 
                   fec_packets_per_block);

        for(int i = 0; i < MAX_FEC_PACKETS_PER_BLOCK; ++i)
        {
            ret = sending_data(si, fec_pool[i]);
        }
    }

    return ret;
}

int sending_data(si4463_t si, const U8* data_array)
{
    const U8* payload = data_array;

    int rest = packet_size;

    int send = 1;

#if USE_STAT_FILE
    if (statf)  // DEBUG
    {
        uint16_t cseq = data_array[0] & 0x3F;
        cseq <<= 8;
        cseq += data_array[1];

        struct timeval tv;  // DEBUG
        gettimeofday(&tv, NULL);  // DEBUG
        fprintf(statf, "%lu:\t%u\t%03u\t%d\t%c\n", 
                tv.tv_sec, 
                cseq, 
                data_array[2]& 0xff, 
                use_fec,
                (data_array[0] & FEC_PACKET_MASK)?'F':'D'); // DEBUG
    }
#endif

    while (1)
    {
        int ret = 0;
        size_t data = 0;
        if (rest <= 0) rest = 0;

        // Check if the radio packet sent successfully
        ret = SI4463_check_transmitted(si, payload, rest, &data);
        if (ret > 0)
        {
            // Data transmitted fully
            break;
        }
        else if (ret == 0)
        {
            rest -= data;
            if (rest <= 0) payload = NULL;
            else           payload += data;
        }
        else
        {
            // Error 
            return -1;
        }

        if (send)
        {
            if ((ret = SI4463_StartTx(si, RADIO_channel, payload, rest)))
            {
                // First part sent
                send = 0;
                payload += ret;
                rest -= ret;
                ++packet_counter;
            }
        }
    }

    return packet_size;
}

static inline void drop_incomming_data(int read_fd, int count)
{
    if (0 >= count)
        return;

    int data = count > MAX_DATA_TO_SKIP ? MAX_DATA_TO_SKIP : count;

    U8 buffer[data];

    if (-1 == read(read_fd, buffer, data))
    {
        perror ("Read:");
    }
}

/*
  transfer data, payload after it has the next format:
  |cseq|sz|data
  cseq - 2 bytes, packet sequence number
         2 first bits are flags for OOB and FEC
         packet[0] & 0x80 - OOB flag
         packet[0] & 0x40 - FEC flag
  sz - 1 byte, the whole data size
  data - original data
         data[0] - OOB size if OOB flag defined
 */
int transfer(int read_fd, int read_oob, si4463_t si)
{
    U8* buffer = get_pool_item(); //(U8*)calloc(1, packet_size+1); // released in sending_data function
    int count;
    int count_to_read;

    ssize_t real_data_size;
    ssize_t r;
    struct pollfd fds[2] = {{0}};
    int nfsd = 1;

    int oob_count;
    int oob = 0;

    while (!quit)
    {
        fds[0].fd = read_fd;
        fds[0].events = POLLIN;
        fds[0].revents = 0;

        if (read_oob != -1)
        {
            fds[1].fd = read_oob;
            fds[1].events = POLLIN;
            fds[1].revents = 0;
            
            nfsd = 2;
        }

        switch(poll(fds, nfsd, 30))
        {
            case 0:
                continue;
            case -1:
                fprintf (stderr, "Poll error, %s\n", strerror (errno));
                return -1;
            default:
                oob_count = 0;
                count_to_read = 0;
                oob = 0;
                real_data_size = 0;
                buffer[0] = 0x0;
        }
        
        if ((read_oob != -1) && (fds[1].revents & POLLIN))
        {
            oob = 1;
            if (0 != ioctl(read_oob, FIONREAD, &oob_count))
            {
                fprintf (stderr, "Cannot determinate data amount, %s\n", strerror (errno));
                return -1;
            }

            if (oob_count > MAX_ALLOWED_OOB)
                oob_count = MAX_ALLOWED_OOB;
        }

        if (fds[0].revents & POLLIN)
        {
            int data_start = PACKET_PREFIX_SIZE;
            if ( oob )
                data_start += 1;

            r = read(read_fd, buffer + data_start, 1);
            if (r == 0) 
            {// EOF
                buffer[2] = EOF_PACKET_MASK;
            }
            else
            {
                ++data_start; // One byte read already

                if (0 != ioctl(read_fd, FIONREAD, &count))
                {
                    fprintf (stderr, "Cannot determinate data amount, %s\n", strerror (errno));
                    return -1;
                }

                if (count > 0)
                {
                    count_to_read = (count + 1 > data_size) ? data_size - 1 : count;
                    if (oob && count_to_read + oob_count + 1> data_size)
                        count_to_read = data_size - oob_count - 2; // 1 is aready read, 1 for oob data size mark

                    real_data_size = read(read_fd, buffer + data_start, count_to_read);
                    if (-1 == real_data_size)
                    {
                        fprintf (stderr, "Cannot read data, %s\n", strerror (errno));
                        return -1;
                    }

                    if ((alowed_queue_size > 0) && (count > alowed_queue_size))
                    {
                        drop_incomming_data(read_fd, count - alowed_queue_size);
                    }

                    real_data_size += 1;
                }
                else
                    real_data_size = 1;
            }
        }
        if (oob)
        {
            int oob_start = PACKET_PREFIX_SIZE + 1 + real_data_size;
            //if (oob_start > data_size)
                
            buffer[0] |= OOB_DATA_MASK;
            buffer[PACKET_PREFIX_SIZE] = oob_count & 0xff;

            r = read(read_oob, buffer + oob_start, oob_count);
            if (-1 == r)
            {
                fprintf (stderr, "Cannot read data, %s\n", strerror (errno));
                return -1;
            }
            real_data_size += oob_count;
        }

        buffer[2] = real_data_size & 0xff;

        if (-1 == send_data(si, packet_size, buffer, use_fec))
        {
            fprintf(stderr, "Transfer failed, return...\n");
            break;
        }

        if (use_fec)
            buffer = get_pool_item(); //(U8*)calloc(1, packet_size+1);
    }

    return 0;
}

void help(const char* name)
{
    fprintf(stderr, "915 MHz version, usage:\n");
    fprintf(stderr, "\t%s [cspgizhrmfFESkK]\n", name);
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
                    " 4fsk(3) or 4gfsk (4)\n", 
                    MIN_MODULATION, MAX_MODULATION);
    fprintf(stderr, "\t\t-P level \t defines power level (1..127)\n");
    fprintf(stderr, "\tEncryption:\n");
    fprintf(stderr, "\t\t-E use encryption\n");
    fprintf(stderr, "\t\t-S session ID\n");
    fprintf(stderr, "\t\t-k define key as hex string, key should be 256 bits\n");
    fprintf(stderr, "\t\t-K define salt as hex string\n");
    fprintf(stderr, "\tMiscellaneous:\n");
    fprintf(stderr, "\t\t-F use FEC\n");
    fprintf(stderr, "\t\t-q COUNT \t defines max available packets in incoming queue, "
                    "packets exceeding the queue size will be dropped\n");
    fprintf(stderr, "\t\t-f FILE \t defines input/output file, use '-' for stdin/stdout\n");
    fprintf(stderr, "\t\t-u FILE \t defines console input file\n");
    fprintf(stderr, "\t\t-U FILE \t creates FIFO to be used as console input file\n");
    fprintf(stderr, "\tHelp:\n");
    fprintf(stderr, "\t\t-h \t shows this help\n");
}

static
int open_file(const char* name)
{
    int flag = O_RDONLY;
    int m = S_IRWXU;

    if (strcmp(name, "-") == 0)
    {
        setvbuf(stdin, NULL, _IONBF, 0);
        return fileno(stdin);
    }

    return open(name, flag, m);
}

int main(int argc, char** argv)
{
    char* file_name = NULL;
    char* oob_file_name = NULL;
    char* fifo_file_name = NULL;
    int fd = -1;
    int oob_fd = -1;
    si4463_t si;
    int c;

    U8 power = 0xff;

    struct SI4463_Part_Info* info;
    opterr = 0;

    signal(SIGINT, quit_handler);
    signal(SIGQUIT, quit_handler);


    while ((c = getopt (argc, argv, "U:u:k:K:c:S:s:p:g:i:r:m:q:f:P:FhE")) != -1)
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
            case 'f': // input/output file
                file_name = strdup(optarg);
                break;
            case 'u': // input/output file
                oob_file_name = strdup(optarg);
                break;
            case 'U': // input/output FIFO
                fifo_file_name = strdup(optarg);
                break;
            case 'E': // use encryption
                use_aes = 1;
                init_crypto();
                break;
            case 'F': // use FEC
                use_fec = 1;
                fec_init();
                break;
            case 'q': // Queue size
                alowed_queue_size = atoi(optarg);
                if (alowed_queue_size == 0)
                {
                    fprintf(stderr, "Wrong queue size %s\n\n", optarg);
                    help(argv[0]);
                    return -1;
                }
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
            case 'P': // power controll
            {
                unsigned int pwr = atoi(optarg);
                if (pwr < 1 || pwr > 0x7f)
                {
                    fprintf(stderr, "Wrong power level defined, acceptable 1..127\n\n");
                    help(argv[0]);
                    return -1;
                }
                power = pwr;
            }
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
        struct stat st;
        if (stat(fifo_file_name, &st) != 0)
            mkfifo(fifo_file_name, 0660);

        oob_fd = open_file(fifo_file_name);
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

#if USE_STAT_FILE
    statf = fopen("stat.log", "a"); //DEBUG
    if (statf)
    {
        fprintf(statf, "Sender modulation %d\n\nTime\tCSEQ\tLEN\tFEC\n", Modulation); // DEBUG
    }
#endif

    wiringPiSetup();

    si = SI4463_init(SPI_Channel, GPIO_gpio1, GPIO_PowerDown, GPIO_interrupt, SPI_Speed);
    if (si == (si4463_t) -1)
    {
        fprintf(stderr, "Device init failed...\n");
        return -1;
    }
//    SI4463_reset_with_cfg(si);
    int cmds;
    switch (Modulation)
    {
        case 1: // fsk
            packet_size = packet_size_2fsk_tx();
            cmds = load_2fsk_config_tx(si);
            break;
        case 2:
            packet_size = packet_size_2gfsk_tx();
            cmds = load_2gfsk_config_tx(si);
            break;
        case 3:
            packet_size = packet_size_4fsk_tx();
            cmds = load_4fsk_config_tx(si);
            break;
        case 4:
            packet_size = packet_size_4gfsk_tx();
            cmds = load_4gfsk_config_tx(si);
            break;
        default:
            fprintf(stderr, "Wrong modulation type, exit...\n");
            SI4463_free(si);
            close(fd);
            return -1;
    }

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

    if (power != 0xFF)
        SI4463_output_pwr_level(si, power);

    free(info);
    info = NULL;

    data_size = packet_size - PACKET_PREFIX_SIZE -2 ;// CRC inline
    fprintf(stdout, "Loaded %d params\n", cmds);
    if (cmds == -1) 
    {
        return -1;
    }

    set_real_time();

    alowed_queue_size *= packet_size;

    SI4463_Cleanup(si);

    transfer(fd, oob_fd, si);

    SI4463_free(si);

    close(fd);

    if (oob_fd != -1)
        close(oob_fd);

    if (fifo_file_name)
    {
        unlink(fifo_file_name);
        free(fifo_file_name);
    }

    if (use_aes)
    {
        destroy_crypto();
    }

#if USE_STAT_FILE
    if (statf) {       // DEBUG
        fflush(statf); // DEBUG
        fclose(statf); // DEBUG
        statf = NULL;  // DEBUG
    }                 // DEBUG
#endif

    printf("Sent %lld packets\n", packet_counter);
    printf("Done\n");
    return 0;
}
