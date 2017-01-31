#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include <sys/ioctl.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>


#include "si4463.h"

#include "fec.h"

#define MAX_DATA_OR_FEC_PACKETS_PER_BLOCK 8
#define MAX_USER_PACKET_LENGTH 128

int main()
{
    char message[MAX_USER_PACKET_LENGTH]; // = "Test message";
    char long_message[MAX_USER_PACKET_LENGTH]; // = "Test message\tTest message\tTest message\tTest message\tTest message\tTest message\t";
    char long_message2[MAX_USER_PACKET_LENGTH];
    char long_message3[MAX_USER_PACKET_LENGTH];
    char long_message4[MAX_USER_PACKET_LENGTH];
    char long_message5[MAX_USER_PACKET_LENGTH];

    uint8_t d_blocks[MAX_DATA_OR_FEC_PACKETS_PER_BLOCK][MAX_USER_PACKET_LENGTH];
    uint8_t fec_pool[MAX_DATA_OR_FEC_PACKETS_PER_BLOCK][MAX_USER_PACKET_LENGTH];


    uint8_t *data_blocks[MAX_DATA_OR_FEC_PACKETS_PER_BLOCK];
    uint8_t *fec_blocks[MAX_DATA_OR_FEC_PACKETS_PER_BLOCK];

    memset(message , 'I', MAX_USER_PACKET_LENGTH);
    message[MAX_USER_PACKET_LENGTH-1] = 0;
    
    memset(long_message , 'X', MAX_USER_PACKET_LENGTH);
    long_message[MAX_USER_PACKET_LENGTH-1] = 0;

    memset(long_message2 , 'Y', MAX_USER_PACKET_LENGTH);
    long_message2[MAX_USER_PACKET_LENGTH-1] = 0;

    memset(long_message3 , 'O', MAX_USER_PACKET_LENGTH);
    long_message3[MAX_USER_PACKET_LENGTH-1] = 0;

    memset(long_message4 , '1', MAX_USER_PACKET_LENGTH);
    long_message4[MAX_USER_PACKET_LENGTH-1] = 0;

    memset(long_message5 , '2', MAX_USER_PACKET_LENGTH);
    long_message5[MAX_USER_PACKET_LENGTH-1] = 0;

    memset(fec_pool, 0, MAX_DATA_OR_FEC_PACKETS_PER_BLOCK*MAX_USER_PACKET_LENGTH);
    memset(d_blocks, 0, MAX_DATA_OR_FEC_PACKETS_PER_BLOCK*MAX_USER_PACKET_LENGTH);
    
    data_blocks[0] = (uint8_t*)long_message; //message;
    data_blocks[1] = (uint8_t*)message;
    data_blocks[2] = (uint8_t*)long_message2;
    data_blocks[3] = (uint8_t*)long_message3;
    data_blocks[4] = (uint8_t*)long_message4;
    data_blocks[5] = (uint8_t*)long_message5;


    fec_blocks[0] = fec_pool[0];
    fec_blocks[1] = fec_pool[1];
    fec_blocks[2] = fec_pool[2];
    fec_blocks[3] = fec_pool[3];

    unsigned int packet_length = MAX_USER_PACKET_LENGTH;//12;
    unsigned int data_packets_per_block = 6;
    unsigned int fec_packets_per_block = 2;

    fec_init();

    fprintf(stderr, "data blocks %p, fec blocks %p\n", data_blocks, fec_blocks);
    fprintf(stderr, "Encode data\n");
        
    fec_encode(packet_length, 
               data_blocks, 
               data_packets_per_block, 
               (unsigned char **)fec_blocks, 
               fec_packets_per_block);

    fprintf(stderr, "Encode done\n");
    int i;
    for (i = 0; i < fec_packets_per_block; ++i)
        fprintf(stderr, "fec block %d, data len %d,\nfec [%s]\n", i, strlen((char*)fec_blocks[i]), fec_blocks[i]);


    // Decode part
    // No data just FEC 
    uint8_t *r_blocks[MAX_DATA_OR_FEC_PACKETS_PER_BLOCK];
    unsigned int fec_block_nos[MAX_DATA_OR_FEC_PACKETS_PER_BLOCK];
    unsigned int erased_blocks[MAX_DATA_OR_FEC_PACKETS_PER_BLOCK];
    unsigned int nr_fec_blocks = 2;

    r_blocks[0] = d_blocks[0];
    r_blocks[1] = d_blocks[1];
    r_blocks[2] = d_blocks[2];
    r_blocks[3] = d_blocks[3];
    r_blocks[4] = d_blocks[4];
    r_blocks[5] = d_blocks[5];


    fec_block_nos[0] = 0;
    fec_block_nos[1] = 1;
    memset(long_message3 , '-', MAX_USER_PACKET_LENGTH/2);
    memset(long_message , 0, MAX_USER_PACKET_LENGTH);

    fprintf(stderr, "Damaged data\n");
    for (i = 0; i < data_packets_per_block; ++i)
        fprintf(stderr, "Block %d, data len %d,\ndata [%s]\n", i, strlen((char*)data_blocks[i]), data_blocks[i]);


    erased_blocks[0] = 0;
    erased_blocks[1] = 3;

    fec_blocks[0] = fec_pool[0];
//    fec_blocks[1] = fec_pool[1];
//    fec_blocks[2] = fec_pool[2];
    fec_blocks[1] = fec_pool[1];


    fec_decode((unsigned int) packet_length, 
               data_blocks, 
               data_packets_per_block,  // One frame in the whole packet
               fec_blocks, 
               fec_block_nos, 
               erased_blocks, 
               nr_fec_blocks);

    fprintf(stderr, "Decode done\n");
    for (i = 0; i < data_packets_per_block; ++i)
        fprintf(stderr, "Block %d, data len %d,\ndata [%s]\n", i, strlen((char*)data_blocks[i]), data_blocks[i]);


    return 0;
}
