#ifndef SI_4463_INIT_H_
#define SI_4463_INIT_H_

#include <stdint.h>

#define si4463_t  void*

#define U8  unsigned char 
#define U16 uint16_t
#define PROP_COUNT 16

#if defined (HALF_DUPLEX_FIFO)
    #define MAX_TX_DATA_BUFF 128
#else
    #define MAX_TX_DATA_BUFF 64
#endif
#define TX_ALMOST_EMPTY 0x10
#define RADIO_RX_ALMOST_FULL_THRESHOLD 30u
#define RADIO_TX_ALMOST_EMPTY_THRESHOLD 30u

struct SI4463_Part_Info
{
    unsigned int chip_rev;
    unsigned int part;
    unsigned int build;
    unsigned int id;
    unsigned int customer;
    unsigned int romid;
};

struct SI4463_Func_Info
{
    U8  revext;
    U8  revbranch;
    U8  build;
    U8  revint;
    U16 patch;
    U8  func;
};


enum SI4463_STATE
{
    NO_CHANGE_STATE  = 0,
    SLEEP_STATE      = 1,
    SPI_ACTIVE_STATE = 2,
    READY_STATE      = 3,
    ANOTHER_ENUMERATION_FOR_READY_STATE = 4,
    TUNE_FOR_TX_STATE = 5,
    TUNE_FOR_RX_STATE = 6,
    TX_STATE = 7,
    RX_STATE = 8
};

si4463_t SI4463_init(int channel, int gpio1, int sdn, int irq, int speed);
void SI4463_free(si4463_t);

int SI4463_reset(si4463_t si);
int SI4463_reset_with_cfg(si4463_t si, const U8* cfg_array);

void SI4463_shutdown(si4463_t si);
int SI4463_powerup(si4463_t si);
int SI4463_IRQ_status(si4463_t si);

struct SI4463_Part_Info* SI4463_part_info(si4463_t si);
struct SI4463_Func_Info* SI4463_funct_info(si4463_t si);

// return channel as well if *channel is not null
int SI4463_device_state(si4463_t si, int* channel);

void SI4463_interrupt_handler(si4463_t si, void (*handler)(void));

int SI4463_transfer(si4463_t si, int channel, U8* buff, size_t len);
//int SI4463_tx_mode(si4463_t device, int channel);
int SI4463_tx_mode(si4463_t device, int channel, U8 next, U16 len);
int SI4463_rx_mode(si4463_t device, int channel);
int SI4463_start_rx(si4463_t device, int channel, size_t packet_len);
int SI4463_receive(si4463_t si, U8* buff, size_t len);
// The same as SI4463_receive but returns immediatelly after data receive and 
// copied to buffer, 
int SI4463_receive_available(si4463_t si, U8* buff, size_t maxlen);

int SI4463_set_device_state(si4463_t device, int next_state);
int SI4463_set_device_prop(si4463_t device, U8 group, U8 num, U8 start, U8 values[PROP_COUNT]);

// cfg_array has the next structure:
// LEN | <LEN length of data>
// 0x00
int SI4463_cfg_load(si4463_t si, const U8* cfg_array);

int SI4463_check_rx(si4463_t device, U8* pbuffer, size_t len);
int SI4463_check_rx_fixed(si4463_t device, U8 *fix_packet, size_t len);

int SI4463_check_transmitted(si4463_t device, const U8* pbuffer, size_t count,  size_t* sent);
int SI4463_check_tx(si4463_t device);

void SI4463_Cleanup(si4463_t device);

int SI4463_StartTx(si4463_t device, U8 channel, const U8 *FixRadioPacket, size_t count);
int SI4463_StartTxFixed(si4463_t device, U8 channel, const U8 *FixRadioPacket, size_t packet_len);

// it configures the PA output power level.Values 0..7f
void SI4463_output_pwr_level(si4463_t device, U8 level);

#endif //SI_4463_INIT_H_
