#ifndef SI4463_TR_SUIT_H
#define SI4463_TR_SUIT_H

#define MAX_BUFFER_SIZE (1024*1024)
#define MAX_PIN 40

#define MIN_SPEED 500
#define MAX_SPEED 10000

#define MIN_MODULATION 1
#define MAX_MODULATION 4

#define SUPPORTED_CHIP_REV 0x4463

#define PACKET_SIZE_MASK   0x7f
#define EOF_PACKET_MASK    0x80
#define FEC_PACKET_MASK    0x40
#define OOB_DATA_MASK      0x80


#define CSEQ_DATA_SIZE 2
#define SZ_DATA_SIZE  1
#define PACKET_PREFIX_SIZE CSEQ_DATA_SIZE + SZ_DATA_SIZE

#define MAX_ALLOWED_OOB 10

#define MAX_DATA_PACKETS_PER_BLOCK 6
#define MAX_FEC_PACKETS_PER_BLOCK 2
#define MAX_USER_PACKET_LENGTH 128

#define MAX_DATA_TO_SKIP 1024

extern int load_2fsk_config_tx(si4463_t si);
extern int load_2gfsk_config_tx(si4463_t si);
extern int load_4fsk_config_tx(si4463_t si);
extern int load_4gfsk_config_tx(si4463_t si);

extern size_t packet_size_2fsk_tx();
extern size_t packet_size_2gfsk_tx();
extern size_t packet_size_4fsk_tx();
extern size_t packet_size_4gfsk_tx();

extern int load_2fsk_config_rx(si4463_t si);
extern int load_2gfsk_config_rx(si4463_t si);
extern int load_4fsk_config_rx(si4463_t si);
extern int load_4gfsk_config_rx(si4463_t si);

extern size_t packet_size_2fsk_rx();
extern size_t packet_size_2gfsk_rx();
extern size_t packet_size_4fsk_rx();
extern size_t packet_size_4gfsk_rx();


#endif //SI4463_TR_SUIT_H
