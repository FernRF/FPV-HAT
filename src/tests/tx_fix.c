#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <string.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "si4463.h"

#include "radio_config_fix_tx.h"

#define PACKET_SEND_INTERVAL  250u
#define RADIO_PACKET_LENGTH   64u

static int SPI_Channel = 0;
static int SPI_Speed = 100;
static int GPIO_PowerDown = 0;
static int GPIO_gpio1 = 3;
static int GPIO_interrupt = 2;
static int RADIO_channel = 0;

U8 radio_config[] = RADIO_CONFIGURATION_DATA_ARRAY;

#define PAYLOAD {0x00, 'T','H','I','S',' ','S','I','M','P','L','E',' ','T','E','S','T',' ','T','R','A','N','S','M','I','T','T','I','O','N', \
                 't','h','i','s',' ','s','i','m','p','l','e',' ','t','e','s','t',' ','t','r','a','n','s','m','i','t','t','i','o','n'} 


U8 radio_payload[RADIO_PACKET_LENGTH] = PAYLOAD;

int main(void)
{
    // Initialize the Hardware and Radio
    si4463_t si;

    wiringPiSetup();

    si = SI4463_init(SPI_Channel, GPIO_gpio1, GPIO_PowerDown, GPIO_interrupt, SPI_Speed);

    SI4463_reset(si);

    int cmds = SI4463_cfg_load(si, radio_config);
    fprintf(stdout, "Loaded %d params\n", cmds);
    if (cmds == -1) 
    {
        return -1;
    }

    U8* payload = &radio_payload[0];
    size_t total = RADIO_PACKET_LENGTH;
    fprintf(stdout, "Total payload size %d\n", total);

    int send  = 1;
    while (1)
    {
        // Check if the radio packet sent successfully
        if (SI4463_check_tx(si))
        {
            send = 1;
            delay(PACKET_SEND_INTERVAL);
        }

        if (send)
        {
            SI4463_StartTxFixed(si, RADIO_channel, payload, total);

            send = 0;

        }
    }


    return 0;
}
