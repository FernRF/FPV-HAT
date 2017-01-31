#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <string.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "si4463.h"

#include "si4463_cmd.h"

#define MAX_CMD_LEN 16u
#define MAX_ITERS 10
#define ITER_SLEEP_MS 1

#define CTS 0xFF

#define MAX_ALOWED_ITERS 1000

#if defined (_DEBUG )

#define STR(s) #s

#define WAIT_GPIO_CTS 1

const char* dbg_file_name = STR(DEBUG_FILE);
static FILE* dbg;

#define DBG(...) fprintf(dbg, __VA_ARGS__)

#define PRINT_CMD_BUF(si, stream)                       \
    for (int i = 0; i< MAX_CMD_LEN; i++)                \
    {                                                   \
        fprintf(stream, "%02X ", si->cmd_buf[i]&0xff);  \
    }

#define PRINT_IRQ_ITEM(item, stream)                    \
    do {                                                \
        for (int i=7; i>=0 ; i--)                       \
            fprintf(stream, "%01X ", (item >> i) & 1);  \
        fprintf(stream, "\n");                          \
    } while(0)

#define PRINT_IRQ_TABLE(si, stream)                             \
    do {                                                        \
        PRINT_IRQ_ITEM(si->irq_status.int_pend, stream);        \
        PRINT_IRQ_ITEM(si->irq_status.int_status, stream);      \
        PRINT_IRQ_ITEM(si->irq_status.ph_pend, stream);         \
        PRINT_IRQ_ITEM(si->irq_status.ph_status, stream);       \
        PRINT_IRQ_ITEM(si->irq_status.modem_pend, stream);      \
        PRINT_IRQ_ITEM(si->irq_status.modem_status, stream);    \
        PRINT_IRQ_ITEM(si->irq_status.chip_pend, stream);       \
        PRINT_IRQ_ITEM(si->irq_status.chip_status, stream);     \
    } while(0)


#define PRINT_DEVICE_STATUS(prefix, si, stream)                         \
    do{                                                                 \
        fprintf(stream, "%s, Device status: IRQ %d, GPIO1 %d, CMD buffer: ", \
                    prefix, digitalRead (si->irq), digitalRead (si->gpio1)); \
        PRINT_CMD_BUF(si, stream);                                      \
        fprintf(dbg, "\n");                                          \
    } while (0)
#else
#define DBG(...)
#define PRINT_CMD_BUF(si, stream)
#define PRINT_IRQ_ITEM(item, stream)
#define PRINT_IRQ_TABLE(si, stream)
#define PRINT_DEVICE_STATUS(prefix, si, stream)

#endif

enum SI4463_MODE
{
    SI4463_INIT,
    SI4463_TX,
    SI4463_RX,
    SI4463_ERROR
};

enum FIFO_TYPE
{
    FIFO_TX = 0x01,
    FIFO_RX = 0x02
};

struct SI4463_IRQ
{
    U8 int_pend;
    U8 int_status;
    U8 ph_pend;
    U8 ph_status; 
    U8 modem_pend;
    U8 modem_status;
    U8 chip_pend;
    U8 chip_status;

#define CHIP_INT_STATUS_PEND  int_pend & 0x04
#define MODEM_INT_STATUS_PEND int_pend & 0x02
#define PH_INT_STATUS_PEND    int_pend & 0x01

#define CHIP_INT_STATUS       int_status & 0x04
#define MODEM_INT_STATUS      int_status & 0x02
#define PH_INT_STATUS         int_status & 0x01

#define RX_FIFO_ALMOST_FULL_PEND  ph_pend & 0x01
#define TX_FIFO_ALMOST_EMPTY_PEND ph_pend & 0x02
#define CRC_ERROR_PEND            ph_pend & 0x08
#define PACKET_RX_PEND            ph_pend & 0x10
#define PACKET_SENT_PEND          ph_pend & 0x20
#define FILTER_MISS_PEND          ph_pend & 0x40
#define FILTER_MATCH_PEND         ph_pend & 0x80

#define RX_FIFO_ALMOST_FULL  ph_status & 0x01
#define TX_FIFO_ALMOST_EMPTY ph_status & 0x02
#define CRC_ERROR            ph_status & 0x08
#define PACKET_RX            ph_status & 0x10
#define PACKET_SENT          ph_status & 0x20
#define FILTER_MISS          ph_status & 0x40
#define FILTER_MATCH         ph_status & 0x80

#define WUT_PEND                  chip_pend & 0x01
#define LOW_BATT_PEND             chip_pend & 0x02
#define CHIP_READY_PEND           chip_pend & 0x04
#define CMD_ERROR_PEND            chip_pend & 0x08
#define STATE_CHANGE_PEND         chip_pend & 0x10
#define FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND chip_pend & 0x20
#define CAL_PEND                  chip_pend & 0xC0

#define WUT                  chip_status & 0x01
#define LOW_BATT             chip_status & 0x02
#define CHIP_READY           chip_status & 0x04
#define CMD_ERROR            chip_status & 0x08
#define STATE_CHANGE         chip_status & 0x10
#define FIFO_UNDERFLOW_OVERFLOW_ERROR chip_status & 0x20
#define CAL                  chip_status & 0xC0

#define POSTAMBLE_DETECT_PEND 	modem_pend & 0x40
#define INVALID_SYNC_PEND 	    modem_pend & 0x20
#define RSSI_JUMP_PEND 	        modem_pend & 0x10
#define RSSI_PEND 	            modem_pend & 0x08
#define INVALID_PREAMBLE_PEND 	modem_pend & 0x04
#define PREAMBLE_DETECT_PEND 	modem_pend & 0x02
#define SYNC_DETECT_PEND        modem_pend & 0x01

#define POSTAMBLE_DETECT        modem_status & 0x40
#define INVALID_SYNC            modem_status & 0x20
#define RSSI_JUMP               modem_status & 0x10
#define RSSI                    modem_status & 0x08
#define INVALID_PREAMBLE        modem_status & 0x04
#define PREAMBLE_DETECT         modem_status & 0x02
#define SYNC_DETECT             modem_status & 0x01
};


struct SI4463
{
    int channel;
    int gpio1;
    int sdn;
    int irq;
    int fd;
    U8 cmd_buf[MAX_CMD_LEN];
    int mode;
    U8 cmd_error_status;
    U8 cmd_error;
    struct SI4463_IRQ irq_status;
};

static int SI4463_CLI_interrupt(struct SI4463* si);

static int SPI_write_bytes(struct SI4463* si, U8* bytes, size_t len)
{
#if defined (_DEBUG )
    DBG("Write %d bytes directly: ", len);
    for (int i = 0; i< len; i++)
        DBG("%02X ", bytes[i]&0xff);
#endif
//    return wiringPiSPIDataRW(si->channel, bytes, len);
    return write(si->fd, bytes, len);
}

static int SPI_read_bytes(struct SI4463* si, U8* bytes, size_t len)
{
//    memset(bytes, 0x44, len);
    return wiringPiSPIDataRW(si->channel, bytes, len);
}

static int SPI_write_cmd(struct SI4463* si, size_t len)
{
    return write(si->fd, si->cmd_buf, len);
}

static int SPI_read_resp(struct SI4463* si, size_t len)
{
    U8 cmd[MAX_CMD_LEN+2];
    int r = 0;
    U8 c[2] = {0x44, 0x44};
    //int iters = 0;
    do
    {
#ifdef WAIT_GPIO_CTS
        r = digitalRead (si->gpio1);
#else
        c[0] = 0x44;
        c[1] = 0x44;
        wiringPiSPIDataRW(si->channel, c, 2);
        if (c[1] == 0xff) break; // Wait for CTS
#endif
        /* if (++iters > MAX_ALOWED_ITERS) */
        /*     return -1; */
    }
    while(r == 0); // HIGH - command completed

    PRINT_DEVICE_STATUS("READ:", si, dbg);

#if 0
    memset(si->cmd_buf, 0x44, len+1);
    r =  wiringPiSPIDataRW(si->channel, si->cmd_buf, len+1);
#endif
    memset(cmd, 0x44, len+2);
    r =  wiringPiSPIDataRW(si->channel, cmd, len+2);
    if (r < 0)
    {
        perror("Read:");
        return -1;
    }

    if (len > 0)
        memcpy(si->cmd_buf, cmd+2, len);

    PRINT_DEVICE_STATUS("Final:", si, dbg);

    return (len == 0) ? 0 : r;
}

static int SPI_execute_cmd(struct SI4463* si, size_t cmd_len, size_t resp_len)
{
    DBG("CMD %02x, data %d, resp %d\n", si->cmd_buf[0]&0xff, cmd_len, resp_len);
    PRINT_DEVICE_STATUS("", si, dbg);

    if ( SPI_write_cmd(si, cmd_len) == -1)
        return -1;

    PRINT_DEVICE_STATUS("CMD passed via SPI, ", si, dbg);

    /* while(digitalRead (si->gpio1) == 0) */
    /* {    } */

    /* fprintf(dbg, "CMD executed, "); */
    /* PRINT_DEVICE_STATUS(si, dbg); */
    return SPI_read_resp(si, resp_len);
}


static int SI4463_fifo_info(struct SI4463* si, int fifo)
{
    int res;
    si->cmd_buf[0] = FIFO_INFO;
    si->cmd_buf[1] = fifo;

    res = SPI_execute_cmd(si, 2, 2);
    if (res < 0)
        return -1;

    return ((si->cmd_buf[1] & 0xff) << 8) + (si->cmd_buf[0] & 0xff);
}

static int SI4463_packet_info(struct SI4463* si)
{
    int res;
    si->cmd_buf[0] = PACKET_INFO;
    si->cmd_buf[1] = 0;
    si->cmd_buf[2] = 0;
    si->cmd_buf[3] = 0;
    si->cmd_buf[4] = 0;
    si->cmd_buf[5] = 0;

    res = SPI_execute_cmd(si, 6, 2);
    if (res < 0)
        return -1;

    return ((si->cmd_buf[0] & 0xff) << 8) + (si->cmd_buf[1] & 0xff);
}

static int SI4463_Tx_Data_buffer(struct SI4463* si, const U8* buf, size_t sz)
{
    U8 cmd[MAX_TX_DATA_BUFF+1];

    if (sz > MAX_TX_DATA_BUFF)
        return -1;

    cmd[0] = WRITE_TX_FIFO;
    memcpy(cmd+1, buf, sz);

    return SPI_write_bytes(si, cmd, sz+1);
}

static int SI4463_Rx_Data_buffer(struct SI4463* si, U8* buf, size_t sz)
{
    int r;
    U8 cmd[MAX_TX_DATA_BUFF+1];

    if (sz > MAX_TX_DATA_BUFF)
        return -1;

    cmd[0] = READ_RX_FIFO;

    if (-1 == (r = SPI_read_bytes(si, cmd, sz+1)))
    {
        DBG("Command read (RX) operation failed\n");
        return -1;
    }

    memcpy(buf, cmd+1, sz);

    return r;
}


static int SI4463_CmdReady(struct SI4463* si, int ms)
{
    waitForInterrupt(si->gpio1, ms);
    int level = digitalRead (si->gpio1);
    return level;
}

int SI4463_chip_status(si4463_t device)
{
    struct SI4463* si = (struct SI4463*)device;

    memset(si->cmd_buf, 0, 16);

    si->cmd_buf[0] = GET_CHIP_STATUS;

    si->cmd_buf[1] = 0;   // clr  PH pending
    
    if (-1 == SPI_execute_cmd(si, 2, 4))
        return -1;
   
    si->irq_status.chip_pend    = si->cmd_buf[0];
    si->irq_status.chip_status  = si->cmd_buf[1];
    si->cmd_error_status = si->cmd_buf[2];
    si->cmd_error        = si->cmd_buf[3];

    PRINT_IRQ_TABLE(si, dbg);
    U16 r = (si->cmd_error_status << 8) + si->cmd_error;

    return (int)r;
}


static int SI4463_CLI_interrupt(struct SI4463* si)
{
    memset(si->cmd_buf, 0, 16);

    si->cmd_buf[0] = GET_INT_STATUS;
    si->cmd_buf[1] = 0;   // clr  PH pending
    si->cmd_buf[2] = 0;   // clr modem_pending
    si->cmd_buf[3] = 0;   // clr chip pending
    
    if (-1 == SPI_execute_cmd(si, 4, 8))
        return -1;
   
    si->irq_status.int_pend   = si->cmd_buf[0];
    si->irq_status.int_status = si->cmd_buf[1];
    si->irq_status.ph_pend    = si->cmd_buf[2];
    si->irq_status.ph_status  = si->cmd_buf[3];
    si->irq_status.modem_pend   = si->cmd_buf[4];
    si->irq_status.modem_status = si->cmd_buf[5];
    si->irq_status.chip_pend    = si->cmd_buf[6];
    si->irq_status.chip_status  = si->cmd_buf[7];

    PRINT_IRQ_TABLE(si, dbg);

    return 0;
}

int SI4463_IRQ_status(si4463_t si)
{
    struct SI4463* si_device = (struct SI4463*)si;
    if (si == NULL)
        return -1;

    return digitalRead (si_device->irq);
}

void SI4463_shutdown(si4463_t si)
{
    struct SI4463* si_device = (struct SI4463*)si;
    if (si != NULL)
        digitalWrite(si_device->sdn, HIGH);
    delay(10);
}

int SI4463_powerup(si4463_t si)
{
    struct SI4463* si_device = (struct SI4463*)si;
    if (si != NULL)
        digitalWrite(si_device->sdn, LOW);

    return SI4463_CmdReady(si_device, 10);
}

int SI4463_reset(si4463_t si)
{
    const U8 cmd[] = { RF_POWER_UP };
    struct SI4463* si_device = (struct SI4463*)si;
    int ready = 0;
    if (si == NULL)
        return -1;

    int irq = SI4463_IRQ_status(si);
    DBG("Initial irq status %d\n", irq);

    digitalWrite(si_device->sdn, HIGH);
    delay(10);

    irq = SI4463_IRQ_status(si);
    DBG("Power down irq status %d\n", irq);
    irq = irq; // Hide the warning

    digitalWrite(si_device->sdn, LOW);
    delay(10);

    PRINT_DEVICE_STATUS("Device booted\n", si_device, dbg);

    ready = SI4463_CmdReady(si_device, 10); // ignore ready here
    DBG( "Ready state: %d\n", ready);

    // send power up command
    memcpy(si_device->cmd_buf, cmd, POWER_UP_CMD_SIZE);         // parameter of power on
    ready = SPI_execute_cmd(si_device, POWER_UP_CMD_SIZE, 0);
    if (ready < 0)
    {
        perror("Power UP:");
        return -1;
    }

    PRINT_DEVICE_STATUS("Device is UP\n", si_device, dbg);

    ready = SI4463_CmdReady(si_device, 10); // ignore ready here, it must be LOW
    if (ready != LOW)
    {
        DBG("WARNING: reset protocol fail, GPIO1 is LOW level failed\n");
    }
    ready = SI4463_CmdReady(si_device, 10); // ignore ready here, it must be HIGH
    if (ready != HIGH)
    {
        DBG("WARNING: reset protocol fail, GPIO1 is HIGH level failed\n");
    }
    SI4463_CLI_interrupt(si_device);
    ready = SI4463_CmdReady(si_device, 10); // ignore ready here, it must be HIGH
    if (ready != HIGH)
    {
        DBG("WARNING: reset protocol fail, GPIO1 is HIGH level failed, reset failed\n");
    }
    irq = SI4463_IRQ_status(si);
    DBG("irq status %d\n", irq);

    si_device->mode = SI4463_INIT;

    return 0;
}

int SI4463_reset_with_cfg(si4463_t si, const U8* cfg_array)
{
    struct SI4463* si_device = (struct SI4463*)si;
    int ready = 0;
    if (si == NULL)
        return -1;

    int irq = SI4463_IRQ_status(si);
    DBG("Initial irq status %d\n", irq);

    digitalWrite(si_device->sdn, HIGH);
    delay(10);

    irq = SI4463_IRQ_status(si);
    DBG("Power down irq status %d\n", irq);
    irq = irq; // Hide the warning

    digitalWrite(si_device->sdn, LOW);
    delay(10);

    PRINT_DEVICE_STATUS("Device booted\n", si_device, dbg);

    ready = SI4463_CmdReady(si_device, 10); // ignore ready here
    DBG( "Ready state: %d\n", ready);
    ready = ready;

    // send power up command
    int ret = SI4463_cfg_load(si_device, cfg_array);

    irq = SI4463_IRQ_status(si);
    DBG("irq status %d\n", irq);

    si_device->mode = SI4463_INIT;

    return ret;
}


si4463_t SI4463_init(int ch, int gpio1, int sdn, int irq, int speed)
{
    struct SI4463 *si;

#if defined (_DEBUG )
#if defined (DEBUG_FILE)
    dbg = fopen(dbg_file_name, "w");
#else
    dbg = stderr;
#endif
#endif
    if (speed > 10000)
    {
        DBG("si4463 working with 10 MHz max\n");
        return (si4463_t)-1;
    }

    si = calloc(1, sizeof(struct SI4463));
    if (si == NULL)
    {
        return (si4463_t)-1;
    }

    si->channel = ch;
    si->gpio1 = gpio1;
    si->sdn = sdn;
    si->irq = irq;

    if ((si->fd = wiringPiSPISetup (ch, speed*1000)) < 0)
    {
        free(si);
        return (si4463_t)-1;
    }

    pinMode(si->gpio1, INPUT);
    pinMode(si->irq, INPUT);
    pinMode(si->sdn, OUTPUT);
    
    si->mode = SI4463_INIT;

    return si;
}        

void SI4463_free(si4463_t si)
{
    struct SI4463* si_device = si;
    if (si == NULL)
        return;

    SI4463_shutdown(si);

    free(si_device);
}

#if 0
int SI4463_transfer(si4463_t si, int channel, U8* buff, size_t len)
{
    size_t sent = 0;
    int fifo_size = 0;
    struct SI4463* si_device = si;
    if (si == NULL)
        return -1;

    if (-1 == SI4463_set_device_state(si_device, READY_STATE))
    {
        DBG("Cannot change state of the device\n");
        return -1;
    }

    SI4463_CLI_interrupt(si_device);

    if (-1 == (fifo_size = SI4463_fifo_info(si_device, FIFO_TX)))
    {
        DBG("Cannot reset FIFO\n");
        return -1;
    }
    fifo_size >>= 8;
    DBG("FIFO accept up to %d bytes\n", fifo_size);
    
    // Ready to send
    while(sent < len)
    {
        int wait = 0;
        U8* buffer = buff + sent;

        size_t to_send = len - sent;
        if (to_send > fifo_size)
            to_send = fifo_size;
        
        if (-1 == SI4463_Tx_Data_buffer(si_device, buffer, to_send))
        {
            DBG("Failed to write TX fifo\n");
            break;
        }
        if (-1 == (fifo_size = SI4463_fifo_info(si_device, 0)))
        {
            DBG("Cannot reset FIFO\n");
            return -1;
        }
        fifo_size>>=8;
        DBG("FIFO buffer after write %d\n", fifo_size);

        if (si_device->mode != SI4463_TX)
        {
            if (-1 == SI4463_tx_mode(si_device, channel, 0x80, 0))
            {
                DBG("Cannot start TX mode\n");
                return -1;
            }
        }
        
        PRINT_DEVICE_STATUS("TX on, wait for interrupt:", si_device, dbg);

        while(1) // wait for interrupt, it shows when data is sent
        {
            SI4463_CLI_interrupt(si_device);
            if (si_device->irq_status.TX_FIFO_ALMOST_EMPTY_PEND) // OK, need more data
                break;

            if (si_device->irq_status.FIFO_UNDERFLOW_OVERFLOW_ERROR || si_device->irq_status.CMD_ERROR)
            {
                DBG("Fatal error: chip return error\n");
                return -1;
            }
            ++wait;
            if (wait > 3) break;
            delay(5);
            DBG("Wait...");
        }
        DBG("Block sent\n");
        if (-1 == (fifo_size = SI4463_fifo_info(si_device, 0)))
        {
            DBG("Cannot reset FIFO\n");
            return -1;
        }
        fifo_size>>=8;
        DBG("FIFO buffer %d\n", fifo_size);

        sent += to_send;
    }

    return sent;
}
#endif


int SI4463_rx_mode(si4463_t device, int channel)
{
    int res;
    struct SI4463* si = (struct SI4463*)device;
    if (si == NULL)
        return -1;

    /* if (si->mode != SI4463_INIT) */
    /*     return -1; */

    si->mode = SI4463_RX;
    si->channel = channel;

    SI4463_CLI_interrupt(si);

    SI4463_fifo_info(si, FIFO_RX);

    si->cmd_buf[0] = START_RX;
    si->cmd_buf[1] = channel & 0xff; // channel
    si->cmd_buf[2] = 0x00; //start immediatelly
    si->cmd_buf[3] = 0x00; //use packet handler field
    si->cmd_buf[4] = 0x00; //use packet handler field
    si->cmd_buf[5] = 0x00; // RXTIMEOUT_STATE, no change
    si->cmd_buf[6] = 0x03; // RXVALID_STATE, Ready state.
    si->cmd_buf[7] = 0x08; // RXINVALID_STATE, Ready state.

    res = SPI_execute_cmd(si, 8, 0);
    return res;
}

int SI4463_start_rx(si4463_t device, int channel, size_t packet_len)
{
    int res;
    struct SI4463* si = (struct SI4463*)device;
    if (si == NULL)
        return -1;

    if (packet_len > MAX_TX_DATA_BUFF)
    {
        DBG("Wrong packet length, it should be not more than FIFO size\n");
        return -1;
    }

    si->mode = SI4463_RX;
    si->channel = channel;

    SI4463_CLI_interrupt(si);

    SI4463_fifo_info(si, FIFO_RX);

    si->cmd_buf[0] = START_RX;
    si->cmd_buf[1] = channel & 0xff; // channel
    si->cmd_buf[2] = 0x00; //start immediatelly
    si->cmd_buf[3] = (packet_len >> 8) & 0xFF; // packet len
    si->cmd_buf[4] = packet_len & 0xff;        // packet len
    si->cmd_buf[5] = 0x00; // RXTIMEOUT_STATE, no change
    si->cmd_buf[6] = 0x08; // RXVALID_STATE, Ready state.
    si->cmd_buf[7] = 0x08; // RXINVALID_STATE, Ready state.

    res = SPI_execute_cmd(si, 8, 0);
    return res;
}


int SI4463_tx_mode(si4463_t device, int channel, U8 next, U16 len)
{
    int res;
    struct SI4463* si = (struct SI4463*)device;
    if (si == NULL)
        return -1;

    /* if (si->mode != SI4463_INIT) */
    /*     return -1; */

    si->mode = SI4463_TX;
    si->channel = channel;

    int state = SI4463_device_state(si, NULL);
//    fprintf(stderr, "Current device state %d\n", state);
    if (state == TX_STATE)
    {
        fprintf(stderr, "Device already in TX state\n");
        SI4463_CLI_interrupt(si);
        return 0;
    }

    SI4463_CLI_interrupt(si);

//    SI4463_fifo_info(si, FIFO_RX);

    si->cmd_buf[0] = START_TX;
    si->cmd_buf[1] = channel & 0xff; // channel
    si->cmd_buf[2] = next; // Next state TX, not a retransmit, start immediatelly
    si->cmd_buf[3] = (len>> 8 ) & 0xff; // Set len to 0
    si->cmd_buf[4] = len  & 0xff; //
    si->cmd_buf[5] = 1;
    si->cmd_buf[6] = 0;

    res = SPI_execute_cmd(si, 7, 0);
    return res;
}

int SI4463_receive_available(si4463_t device, U8* buff, size_t maxlen)
{
    int recv = 0;
    struct SI4463* si = (struct SI4463*)device;
    if (si == NULL)
        return -1;

    if (si->mode != SI4463_RX)
    {
        DBG("Wrong device mode\n");
        return -1;
    }

    if (maxlen < MAX_TX_DATA_BUFF)
    {
        DBG("Buffer too small\n");
        return -1;
    }

    while(1) // wait for interrupt, it shows when data is sent
    {
        SI4463_CLI_interrupt(si);
        if (si->irq_status.RX_FIFO_ALMOST_FULL || si->irq_status.PACKET_RX ||
            si->irq_status.PACKET_RX || si->irq_status.RX_FIFO_ALMOST_FULL_PEND)
            break; // There is a pcket waiting in the FIFO

        if (si->irq_status.FIFO_UNDERFLOW_OVERFLOW_ERROR || si->irq_status.CMD_ERROR)
        {
            DBG("Fatal error: chip return error\n");
            return -1;
        }
    }

    recv = SI4463_packet_info(si);
    if (recv == -1)
    {
        DBG("Cannot receive packet info\n");
        return -1;
    }

    if (recv > maxlen)
        recv = maxlen;

    if (recv == 0) // Is it bug in chip or code?
    {
        U16 len = SI4463_fifo_info(si, 0);
        DBG("FIFO data: RX->%d, TX->%d\n", len & 0xff, (len >>8)& 0xff);
        len = len;
    }

    if (-1 == SI4463_Rx_Data_buffer(si, buff, recv))
    {
        DBG("Failed to receive RX fifo\n");
        return -1;
    }

    return recv;
}

int SI4463_receive(si4463_t device, U8* buff, size_t len)
{
    size_t recv = 0;
    struct SI4463* si = (struct SI4463*)device;
    if (si == NULL)
        return -1;

    if (si->mode != SI4463_RX)
        return -1;

    while(recv < len)
    {
        U8* buffer = buff + recv;
        int to_recv;

        while(1) // wait for interrupt, it shows when data is sent
        {
            SI4463_CLI_interrupt(si);
            if (si->irq_status.RX_FIFO_ALMOST_FULL || si->irq_status.PACKET_RX ||
                si->irq_status.PACKET_RX || si->irq_status.RX_FIFO_ALMOST_FULL_PEND)
                break; // There is a pcket waiting in the FIFO

            if (si->irq_status.FIFO_UNDERFLOW_OVERFLOW_ERROR || si->irq_status.CMD_ERROR)
            {
                DBG("Fatal error: chip return error\n");
                return -1;
            }
        }

        to_recv = SI4463_packet_info(si);
        if (to_recv == -1)
        {
            DBG("Cannot receive packet info\n");
            return -1;
        }

        if (recv + to_recv > len)
        {
            to_recv = len - recv;
        }

        if (-1 == SI4463_Rx_Data_buffer(si, buffer, to_recv))
        {
            DBG("Failed to receive TX fifo\n");
            break;
        }

        recv += to_recv;
    }

    return recv;
}

void SI4463_interrupt_handler(si4463_t si, void (*handler)(void))
{
    struct SI4463* si_device = si;
    if (si == NULL)
        return;

    wiringPiISR(si_device->irq, INT_EDGE_BOTH, handler);
}

struct SI4463_Part_Info* SI4463_part_info(si4463_t si)
{
    int r;
    struct SI4463_Part_Info* info;
    struct SI4463* si_device = si;

    if (si == NULL)
        return NULL;

    si_device->cmd_buf[0] = PART_INFO;

    r = SPI_execute_cmd(si_device, 1, 8);
    if (r < 0)
        return NULL;

    info = calloc(1, sizeof(struct SI4463_Part_Info));
    if (info == NULL)
        return NULL;

    info->chip_rev = si_device->cmd_buf[0];
    info->part     = (si_device->cmd_buf[1] << 8) + si_device->cmd_buf[2];
    info->build    = si_device->cmd_buf[3];
    info->id       = (si_device->cmd_buf[4] << 8) + si_device->cmd_buf[5];
    info->customer = si_device->cmd_buf[6];
    info->romid    = si_device->cmd_buf[7];

    return info;
}

struct SI4463_Func_Info*  SI4463_funct_info(si4463_t si)
{
    int r;
    struct SI4463_Func_Info* info;
    struct SI4463* si_device = si;

    if (si == NULL)
        return NULL;

    si_device->cmd_buf[0] = FUNC_INFO;

    r = SPI_execute_cmd(si_device, 1, 6);
    if (r < 0)
        return NULL;

    info = calloc(1, sizeof(struct SI4463_Func_Info));
    if (info == NULL)
        return NULL;

    info->revext = si_device->cmd_buf[0];
    info->revbranch = si_device->cmd_buf[1];
    info->revint    = si_device->cmd_buf[2];
    info->patch     = ((si_device->cmd_buf[3]&0xff) << 8) + (si_device->cmd_buf[4] & 0xff);
    info->func      = si_device->cmd_buf[5];

    return info;
}

int SI4463_device_state(si4463_t device, int* channel)
{
    int res;
    struct SI4463* si = device;
    si->cmd_buf[0] = REQUEST_DEVICE_STATE;

    res = SPI_execute_cmd(si, 1, 2);
    if (res < 0)
        return -1;
    
    if (channel != NULL)
        *channel = si->cmd_buf[1] & 0xff;

    return si->cmd_buf[0] & 0x0f;
}

int SI4463_set_device_state(si4463_t device, int next_state)
{
    struct SI4463* si = device;
    si->cmd_buf[0] = CHANGE_STATE;
    si->cmd_buf[1] = 0xf & next_state; 

    return SPI_execute_cmd(si, 2, 0);

}

int SI4463_cfg_load(si4463_t si, const U8* cfg_array)
{
    struct SI4463* si_device = si;

    U8 col;
    U8 numOfBytes;
    const U8 *cmd = cfg_array;
    int total = 0;

    while (*cmd != 0x00)
    {
        numOfBytes = *cmd++;

        if (numOfBytes > MAX_CMD_LEN)
        {
            DBG("Number of command bytes exceeds maximal allowable length");
            return -1;
        }
        for (col = 0u; col < numOfBytes; col++)
        {
            si_device->cmd_buf[col] = *cmd;
            cmd++;
        }
    
        if (-1 == SPI_execute_cmd(si_device,  numOfBytes, 0))
        {
            DBG("SPI command execution failed");
            return -1;
        }
        total ++;
    }

    return total;
}

int SI4463_set_device_prop(si4463_t device, U8 group, U8 num, U8 start, U8 values[PROP_COUNT])
{
    struct SI4463* si = device;
    si->cmd_buf[0] = SET_PROPERTY;
    si->cmd_buf[1] = group;
    si->cmd_buf[2] = num;
    si->cmd_buf[3] = start;
    for (int i = 0; i < num; ++i)
        si->cmd_buf[i+4] = values[i];

    return SPI_execute_cmd(si, 2, 0);

}

//void radio_send_variable_packet(si4463_t device, U8 channel, uint8_t *packet, uint16_t length)
int SI4463_transfer(si4463_t device, int channel, U8* packet, size_t length)
{
    U8 data[length+2];
    struct SI4463* si = (struct SI4463*)device;
#ifdef DEBUG
    U8 status_ph[0xfff];
    U8 status_mod[0xfff];
    U8 status_chip[0xfff];
    U16 i = 0;

    memset(status_ph, 0, 0xfff);
    memset(status_mod, 0, 0xfff);
    memset(status_chip, 0, 0xfff);
#endif

    // Leave RX state
    SI4463_set_device_state(device, READY_STATE);

    SI4463_CLI_interrupt(si);

    // Reset the Tx Fifo
    SI4463_fifo_info(si, FIFO_TX);

    // copy payload data to send buffer
    memcpy(data+2, packet, length);

    // Field 1 length
    U8 props[PROP_COUNT];
    props[0] = 0x00;
    props[1] = 0x02;

    SI4463_set_device_prop(device, 0x12, 2, 0x0D, props);


    // Field 2 length
    props[0] = length >> 8;
    props[1] = length & 0xFF;
    SI4463_set_device_prop(device, 0x12, 2, 0x11, props);

    data[0] = length >> 8;
    data[1] = length & 0xFF;
    
    int16_t remaining = length+2;
    uint8_t* ptr = data;

    volatile int first_run = 1;
    while(remaining > 0) {
        uint8_t nowLength;
        if (first_run) 
        {
            nowLength = MAX_TX_DATA_BUFF;
        }
        else if (TX_ALMOST_EMPTY < remaining) {
            nowLength = TX_ALMOST_EMPTY;
        }
        else {
            nowLength = remaining;
        }

        // Fill the TX fifo with datas
        SI4463_Tx_Data_buffer(si, ptr, nowLength);
        ptr += nowLength;
        remaining -= nowLength;
        
        if (first_run) {
            SI4463_tx_mode(device, channel, 0x80, 0x00);
            first_run = 0;
        }

        while (1) {
            SI4463_CLI_interrupt(si);

#ifdef DEBUG
            // DEBUG
            if (Si446xCmd.GET_INT_STATUS.PH_PEND > 0) {
                status_ph[i] = Si446xCmd.GET_INT_STATUS.PH_PEND;
                status_mod[i] = Si446xCmd.GET_INT_STATUS.MODEM_STATUS;
                status_chip[i] = Si446xCmd.GET_INT_STATUS.CHIP_STATUS;
                ++i;
            }
#endif

            if (si->irq_status.FIFO_UNDERFLOW_OVERFLOW_ERROR || si->irq_status.CMD_ERROR) {
                DBG("Error, RF chip reported error in sending mode\n");

                /* // reset chip to assure correct behaviour next time */
                /* radio_shutdown(); */
                /* msDelayActive(50); */
                /* msDelay(100); */
                /* radio_init(); // also reenables interrupts */
                /* radio_reset_packet_size(); // reset size of Field 2 */
                return -1;
            }
            else if (si->irq_status.PACKET_SENT_PEND){
                if (remaining > 0) {
                    // ERROR CASE! PACKET_SENT interrupt occurred,
                    // even if not all remaining bytes have been put to FIFO
                    DBG("[ERROR] PACKET_SENT, but remaining: %d\n", remaining);
                    // clear remaining bytes to avoid pushing further bytes to FIFO
                    remaining = 0;
                }

                break;
            }
            else if (si->irq_status.TX_FIFO_ALMOST_EMPTY_PEND){
                // not all bytes sent yet, but chip is ready to receive more
                if (remaining > 0) break;
            }
        }
    }
    DBG("remaining: %d\n", remaining);

#ifdef DEBUG
    for (int j = 0; j < i; ++j) {
        DBG("status[%d] = %d %d %d\n", j, status_ph[j], status_mod[j], status_chip[j]);
    }
#endif

    // Reset field 2 length
    props[0] = 0x1F;
    props[1] = 0xFF;
    SI4463_set_device_prop(device, 0x12, 2, 0x11, props);
    return length;
}

int SI4463_check_rx_fixed(si4463_t device, U8 *fix_packet, size_t len)
{
    struct SI4463* si = (struct SI4463*)device;

    /* Read ITs, clear pending ones */
    SI4463_CLI_interrupt(si);

    if (si->irq_status.PACKET_RX_PEND)
    {
        /* Packet RX */
        SI4463_Rx_Data_buffer(si, fix_packet, len);
        return len;
    }

    if (si->irq_status.CRC_ERROR_PEND)
    {
        /* Reset FIFO */
        DBG("Reset FIFO\n");
        fprintf(stderr, "WARNING: CRC check failed\n");

        SI4463_fifo_info(si, FIFO_RX);
    }

    return 0;
}

int SI4463_check_rx(si4463_t device, U8* pbuffer, size_t len)
{
    struct SI4463* si = (struct SI4463*)device;
    int ret = 0;

    /* Read ITs, clear pending ones */
    SI4463_CLI_interrupt(si);

    /* check the reason for the IT */
    if (si->irq_status.SYNC_DETECT_PEND)
    {
        /* Blink once LED2 to show Sync Word detected */
        DBG("Sync Word detected\n");
    }

    if (si->irq_status.PACKET_RX)
    {
        DBG("CRC OK or not enabled\n" );

        ssize_t to_read = len >= MAX_TX_DATA_BUFF ? MAX_TX_DATA_BUFF : len;

        /* Read the RX FIFO with the number of RX FIFO size */
        if (-1 == SI4463_Rx_Data_buffer(si, pbuffer, to_read))
        {
            perror("SI4463_Rx_Data_buffer:");
            to_read = -1;
        }

        SI4463_rx_mode(device, si->channel);

        return to_read;
    }

    if (si->irq_status.CRC_ERROR_PEND)
    {
        DBG("Reset FIFO\n");
        fprintf(stderr, "WARNING: CRC check failed\n");

        SI4463_fifo_info(si, FIFO_RX);
        return -2;
    }

    return ret;
}

int SI4463_check_transmitted(si4463_t device, const U8* pbuffer, size_t count,  size_t* sent)
{
    int ret = 0;

    struct SI4463* si = (struct SI4463*)device;

    /* Read ITs, clear pending ones */
    SI4463_CLI_interrupt(si);

    /* check the reason for the IT */
    if (si->irq_status.PACKET_SENT_PEND)
    {
        /* Nothing is sent to TX FIFO */
        if (sent) *sent = 0;
        return  1;
    }

    if (pbuffer != NULL && si->irq_status.TX_FIFO_ALMOST_EMPTY_PEND)
    {
        /* Calculate the number of remaining bytes has to be sent to TX FIFO */
    
        if(count > RADIO_TX_ALMOST_EMPTY_THRESHOLD)
        { // remaining byte more than threshold

            /* Fill TX FIFO with the number of THRESHOLD bytes */
            SI4463_Tx_Data_buffer(si, pbuffer, RADIO_TX_ALMOST_EMPTY_THRESHOLD);

            ret = RADIO_TX_ALMOST_EMPTY_THRESHOLD;

        }
        else
        { // remaining byte less or equal than threshold

            /* Fill TX FIFO with the number of rest bytes */
            SI4463_Tx_Data_buffer(si, pbuffer, count);

            /* Calculate how many bytes are sent to TX FIFO */
            ret = count;
        }
    }

    if (sent) *sent = ret;

    return 0;
}

int SI4463_check_tx(si4463_t device)
{
    struct SI4463* si = (struct SI4463*)device;

    /* Read ITs, clear pending ones */
    SI4463_CLI_interrupt(si);

    return (si->irq_status.PACKET_SENT_PEND);
}


void SI4463_Cleanup(si4463_t device)
{
    struct SI4463* si = (struct SI4463*)device;

    /* Reset TX FIFO */
    SI4463_fifo_info(si, FIFO_TX);

    // Read ITs, clear pending ones
    SI4463_CLI_interrupt(si);
}

int SI4463_StartTx(si4463_t device, U8 channel, const U8 *FixRadioPacket, size_t count)
{
    int ret;
    struct SI4463* si = (struct SI4463*)device;

    /* Fill the TX fifo with datas */
    if(MAX_TX_DATA_BUFF  < count)
    {
        /* Data to be sent is more than the size of TX FIFO */
        SI4463_Tx_Data_buffer(si, FixRadioPacket, MAX_TX_DATA_BUFF);

        /* Calculate how many bytes are sent to TX FIFO */
        ret = MAX_TX_DATA_BUFF;
    }
    else
    {
        // Data to be sent is less or equal than the size of TX FIFO
        SI4463_Tx_Data_buffer(si, FixRadioPacket, count);
        ret = count;
    }

    /* Start sending packet, channel 0, START immediately, Packet length according to PH, go READY when done */
    SI4463_tx_mode(si, channel, 0x30, count);

    return ret;
}

int SI4463_StartTxFixed(si4463_t device, U8 channel, const U8 *FixRadioPacket, size_t packet_len)
{
    struct SI4463* si = (struct SI4463*)device;

    if(MAX_TX_DATA_BUFF < packet_len)
    {
        DBG("Wrong packet len\n");
        return -1;
    }

    /* Reset TX FIFO */
    SI4463_fifo_info(si, FIFO_TX);

    // Read ITs, clear pending ones
    SI4463_CLI_interrupt(si);

    /* Fill the TX fifo with datas */
    SI4463_Tx_Data_buffer(si, FixRadioPacket, packet_len);

    /* Start sending packet, channel 0, START immediately, Packet length according to PH, go READY when done */
    SI4463_tx_mode(si, channel, 0x30, packet_len);

    return packet_len;
}

void SI4463_output_pwr_level(si4463_t device, U8 level)
{
    if (level > 0x7f) level = 0x7f;
    struct SI4463* si = (struct SI4463*)device;
    if (si == NULL)
        return;

    U8 values[] = {level};

    SI4463_set_device_prop(si, 0x22, 1, 0x1, values);
}
