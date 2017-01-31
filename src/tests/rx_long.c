#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <string.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "si4463.h"

#include "radio_config_long.h"

/*! Maximal packet length definition (FIFO size) */
#define RADIO_MAX_PACKET_LENGTH     64u

/*! Maximal long packet length */
#define RADIO_MAX_LONG_PACKET_LENGTH (4u * RADIO_MAX_PACKET_LENGTH)

/*! Threshold for TX FIFO */
#define RADIO_RX_ALMOST_FULL_THRESHOLD 30u

/*****************************************************************************
 *  Global Typedefs & Enums
 *****************************************************************************/
U8 fixRadioPacket[RADIO_MAX_LONG_PACKET_LENGTH];

U8* pPositionInPayload =  &fixRadioPacket[0u];


static int SPI_Channel = 0;
static int SPI_Speed = 100;
static int GPIO_PowerDown = 0;
static int GPIO_gpio1 = 3;
static int GPIO_interrupt = 2;
static int RADIO_channel = 0;

U8 radio_config[] = RADIO_CONFIGURATION_DATA_ARRAY;


int main(void)
{
    si4463_t si;

    int ret = 0;

    wiringPiSetup();

    si = SI4463_init(SPI_Channel, GPIO_gpio1, GPIO_PowerDown, GPIO_interrupt, SPI_Speed);

    SI4463_reset(si);

    int cmds = SI4463_cfg_load(si, radio_config);
    fprintf(stdout, "Load %d params\n", cmds);
    if (cmds == -1) 
    {
        return -1;
    }

    // Start RX
    SI4463_rx_mode(si, RADIO_channel);
    int pos = 0;    

    while (1)
    {
        // Demo Application Poll-Handler function
        if (0<(ret = SI4463_check_rx(si, pPositionInPayload+pos, RADIO_MAX_LONG_PACKET_LENGTH-pos)))
        {
            int i;
            fprintf(stdout, "Data received: total size %d, value ", ret);
            for (i = 0; i< ret; i++)
            {
                fprintf(stdout, "%02X ", pPositionInPayload[i]&0xff);
            }
            fprintf(stdout, "\n");
            pos += ret;
            if (pos + RADIO_RX_ALMOST_FULL_THRESHOLD >= RADIO_MAX_LONG_PACKET_LENGTH)
                pos = 0;
            
        }
        else if (ret < 0)
            fprintf(stdout,  "ERROR, code return  %d\n", ret);
    }

    return 0;
}
