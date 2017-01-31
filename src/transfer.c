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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>


#include "si4463.h"
#include "radio_config.h"

//#define   TRUE    (1==1)
//#define   FALSE   (!TRUE)

#define MAX_BUFFER_SIZE (1024*1024)
#define MAX_PIN 40

#define MIN_SPEED 500
#define MAX_SPEED 10000

#define SUPPORTED_CHIP_REV 0x4463

enum WORKING_MODE
{
    SEND,
    RECV
};

static int transfer(int read_fd, si4463_t si);
static int receiver(int write_fd, si4463_t si);

static volatile int quit = 0; // Flags should be set in signal handler

#define GET_LEN(CMD, GROUP, ITEMS, ID, PKT_LEN, \
                PKT_LEN_FIELD_SOURCE, PKT_LEN_ADJUST, \
                PKT_TX_THRESHOLD, PKT_RX_THRESHOLD, \
                PKT_FIELD_1_LENGTH_12_8, PKT_FIELD_1_LENGTH_7_0, \
                PKT_FIELD_1_CONFIG, PKT_FIELD_1_CRC_CONFIG, \
                PKT_FIELD_2_LENGTH_12_8, PKT_FIELD_2_LENGTH_7_0, \
                PKT_FIELD_2_CONFIG)                                (PKT_FIELD_1_LENGTH_12_8 << 8) + PKT_FIELD_1_LENGTH_7_0

#define PARSE__GET_LEN(A) GET_LEN(A)

#define PKT_LEN(A) GET_LEN(A)


#define PACKET_SIZE        PKT_LEN(RF_PKT_LEN_12)
#define PACKET_PREFIX_SIZE 1
#define DATA_SIZE          PACKET_SIZE - PACKET_PREFIX_SIZE
#define PACKET_SIZE_MASK   0x7f
#define EOF_PACKET_MASK    0x80


static int SPI_Channel = 0;
static int SPI_Speed = MAX_SPEED;
static int GPIO_PowerDown = 0;
static int GPIO_gpio1 = 3;
static int GPIO_interrupt = 2;
static int BUFFER_size = 1000;
static int RADIO_channel = 0;

U8 radio_config[] = RADIO_CONFIGURATION_DATA_ARRAY;

static void quit_handler(int sig)
{
    quit = 1;
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

int receiver(int write_fd, si4463_t si)
{
    int size ;
    unsigned char *buffer;
    int total_data = 0;

    size = BUFFER_size*1024;

    if ((buffer = malloc(size)) == NULL)
    {
        fprintf (stderr, "Unable to allocate buffer: %s\n", strerror (errno)) ;
        exit (EXIT_FAILURE) ;
    }

    // Start RX
    SI4463_rx_mode(si, RADIO_channel);
    int pos = 0;    
    int pkt_sz = 0;
    int eof = 0;
    int ret;

    while (! eof && ! quit)
    {
        if (0 < (ret = SI4463_check_rx(si, buffer + pos, PACKET_SIZE - pos)))
        {
            pkt_sz = buffer[0] & PACKET_SIZE_MASK;
            if (pkt_sz > PACKET_SIZE) // ERROR in data, try to fix it... hope lost only a few bytes
                pkt_sz = PACKET_SIZE;

            pos += ret;
            if (pos >= pkt_sz)
            {
                if (-1 == write(write_fd, buffer+1, pkt_sz))
                {
                    perror("Data write:");
                    break;
                }
                //        eof = buffer[0] & EOF_PACKET_MASK;
                total_data += pkt_sz;
                pos = 0;
            }
        }
        else if (ret < 0)
        {
            fprintf(stderr,  "ERROR, code return  %d\n", ret);
            break;
        }
    }

    free(buffer);

    return total_data;
}

int sent_data(si4463_t si, const unsigned char* buffer, size_t total_data, int eof)
{
    size_t total = 0;
    U8 data_array[PACKET_SIZE];
    U8* payload = &data_array[0];

    while(total < total_data)
    {
        int rest = total_data > total + DATA_SIZE ? DATA_SIZE : total_data - total;

        memcpy(payload+1, buffer + total, rest);

        payload[0] = eof ? EOF_PACKET_MASK : 0;
        payload[0] += rest;

        total += rest;

        rest += 1; // Count eof and size
        
        int send = 1;
        while (1)
        {
            int ret = 0;
            size_t data = 0;
            if (rest <= 0) rest = 0;

            // Check if the radio packet sent successfully
            if (SI4463_check_transmitted(si, payload, rest, &data))
            {
                // Data transmitted fully
                payload = &data_array[0];
                break;
            }
            else
            {
                payload += data;
                rest -= data;
            }

            if (send)
            {
                if ((ret = SI4463_StartTx(si, RADIO_channel, payload, rest)))
                {
                    // First part sent
                    send = 0;
                    payload += ret;
                    rest -= ret;
                }
            }
        }
        delay(1);
    }
    return total;
}


int transfer(int read_fd, si4463_t si)
{
    int size ;
    unsigned char buffer[PACKET_SIZE + 1];
    int count;
    ssize_t real_data_size;

    size = DATA_SIZE;
    
    while (!quit)
    {
        ssize_t r = read(read_fd, buffer, 1);
        if (r == 0) 
        {// EOF
            sent_data(si, buffer, 1, 1);
            break;
        }
        if (0 != ioctl(read_fd, FIONREAD, &count))
        {
            fprintf (stderr, "Cannot determinate data amount, %s\n", strerror (errno));
            return -1;
        }

        if (count > 0)
        {
            if (count + 1 > size)
                count = size - 1;

            real_data_size = read(read_fd, buffer + 1, count);
            if (-1 == real_data_size)
            {
                fprintf (stderr, "Cannot read data, %s\n", strerror (errno));
                return -1;
            }

            real_data_size += 1;
        }
        else
            real_data_size = 1;

        if (-1 == sent_data(si, buffer, real_data_size, 0))
        {
            fprintf(stderr, "Transfer failed, return...\n");
            break;
        }
        
    }

    return 0;
}

void help(const char* name)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "\t%s [cspgizhrmf]\n", name);
    fprintf(stderr, "\tSPI configuration:\n");
    fprintf(stderr, "\t\t-c [0|1] \t defines SPI channel\n");
    fprintf(stderr, "\t\t-s [%d..%d] \t defines SPI bus speed, kHz\n", MIN_SPEED, MAX_SPEED);
    fprintf(stderr, "\tGPIO configuration:\n");
    fprintf(stderr, "\t\t-p PIN \t defines RPi pin connected to power down control GPIO pin\n");
    fprintf(stderr, "\t\t-g PIN \t defines RPi pin connected to GPIO1 of Si4463\n");
    fprintf(stderr, "\t\t-i PIN \t defines RPi pin connected to interrupt output pin\n");
    fprintf(stderr, "\tBuffer configuration:\n");
    fprintf(stderr, "\t\t-z BUFFER \t defines internal buffer size, kB\n");
    fprintf(stderr, "\tRadio configuration:\n");
    fprintf(stderr, "\t\t-r CHANNEL \t defines radio channel\n");
    fprintf(stderr, "\tWorking mode\n");
    fprintf(stderr, "\t\t-m \t [send|recv] mode\n");
    fprintf(stderr, "\tMiscellaneous:\n");
    fprintf(stderr, "\t\t-f \t FILE \t defines input/output file, use '-' for stdin/stdout\n");
    fprintf(stderr, "\tHelp:\n");
    fprintf(stderr, "\t\t-h \t shows this help\n");
}

static
int open_file(const char* name, int mode)
{
    int flag = (mode == SEND) ? O_RDONLY : O_CREAT | O_WRONLY;
    int m = S_IRWXU;

    if (strcmp(name, "-") == 0)
    {
        setvbuf((mode == SEND) ? stdin : stdout, NULL, _IONBF, 0);
        return (mode == SEND) ? fileno(stdin) : fileno(stdout);
    }

    return open(name, flag, m);
}

int main(int argc, char** argv)
{
    char* file_name = NULL;
    int mode = -1;
    int fd = -1;
    si4463_t si;
    int c;
    struct SI4463_Part_Info* info;
    opterr = 0;

    signal(SIGINT, quit_handler);
    signal(SIGQUIT, quit_handler);


    while ((c = getopt (argc, argv, "c:s:p:g:i:z:r:m:f:h")) != -1)
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
            case 'f': // input/output file
                file_name = strdup(optarg);
                break;
            case 'm': // working mode
            {
                char* m = strdup(optarg);
                for(int i = 0; m[i] != 0x0; i++)
                    m[i] = tolower(m[i]);
                if (strcmp(m, "send") == 0)
                    mode = SEND;
                else if (strcmp(m, "recv") == 0)
                    mode = RECV;
                else {
                    free(m);
                    fprintf(stderr, "Wrong working mode %s\n", m);
                    help(argv[0]);
                    return -1;
                }
                free(m);
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
            case 'z': // internal buffer size
                BUFFER_size = atoi(optarg);
                if (BUFFER_size < 0 || BUFFER_size > MAX_BUFFER_SIZE)
                {
                    fprintf(stderr, "Wrong internal buffer size\n\n");
                    help(argv[0]);
                    return -1;
                }
                break;
            case 'r': // SPI channel
                RADIO_channel = atoi(optarg);
                if (RADIO_channel < 0 || RADIO_channel >32767) 
                {
                    fprintf(stderr, "Wrong radio channel\n\n");
                    help(argv[0]);
                    return -1;
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

    if (mode == -1)
    {
        fprintf(stderr, "Working mode is not defined... Exit\n");
        return 1;
    }

    if (file_name == NULL)
    {
        file_name = strdup("-");
    }

    fd = open_file(file_name, mode);
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

    int cmds = SI4463_cfg_load(si, radio_config);
    fprintf(stdout, "Loaded %d params\n", cmds);
    if (cmds == -1) 
    {
        return -1;
    }

    if (mode == SEND)
        transfer(fd, si);
    else
    {
        set_real_time();
        receiver(fd, si);
    }
    SI4463_free(si);

    close(fd);

    printf("Done\n");
    return 0;
}
