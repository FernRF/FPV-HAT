#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <string.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "si4463.h"

#include "radio_config_fix_rx.h"

/*! Maximal packet length definition (FIFO size) */
#define RADIO_PACKET_LENGTH     64u

U8 fixRadioPacket[RADIO_PACKET_LENGTH];

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
    SI4463_start_rx(si, RADIO_channel, RADIO_PACKET_LENGTH);

    while (1)
    {
        ret = SI4463_check_rx_fixed(si, fixRadioPacket, RADIO_PACKET_LENGTH);
        if (ret < 0)
        {
            fprintf(stderr, "ERROR, terminating...\n");
            break;
        }
        else if (ret > 0)
        {
            int i;
            fprintf(stdout, "Data received: total size %d, value ", ret);
            for (i = 0; i< ret; i++)
            {
                fprintf(stdout, "%02X ", pPositionInPayload[i]&0xff);
            }
            fprintf(stdout, "\n");
        }
    }

    return 0;
}
