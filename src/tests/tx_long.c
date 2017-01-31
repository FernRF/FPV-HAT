#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include <string.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "si4463.h"

#include "radio_config_long.h"

static int SPI_Channel = 0;
static int SPI_Speed = 100;
static int GPIO_PowerDown = 0;
static int GPIO_gpio1 = 3;
static int GPIO_interrupt = 2;
static int RADIO_channel = 0;

U8 radio_config[] = RADIO_CONFIGURATION_DATA_ARRAY;

#define PAYLOAD {0x00, 'T','H','I','S',' ','S','I','M','P','L','E',' ','T','E','S','T',' ','T','R','A','N','S','M','I','T','T','I','O','N', \
                 't','h','i','s',' ','s','i','m','p','l','e',' ','t','e','s','t',' ','t','r','a','n','s','m','i','t','t','i','o','n'} 


U8 radio_payload[] = PAYLOAD;

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
    size_t total;
    size_t rest = total = sizeof(radio_payload);
    fprintf(stdout, "Total payload size %d\n", total);

    int send = 1;
    while (1)
    {
        int ret = 0;
        size_t data = 0;
        if (rest <= 0)
            rest = 0;

        // Check if the radio packet sent successfully
        if (SI4463_check_transmitted(si, payload, rest, &data))
        {
            fprintf(stdout, "Data transmitted fully\n");
            rest = total;
            ++radio_payload[0]; // seq
            payload =  &radio_payload[0];
            delay(100);
            send  = 1;
        }
        else
        {
            payload += data;
            rest -= data;
        }

        if (send)
            if ((ret = SI4463_StartTx(si, RADIO_channel, payload, rest)))
            {
                fprintf(stdout, "First part (%d) sent\n", ret);
                send = 0;
                payload += ret;
                rest -= ret;
            }
    }

    SI4463_shutdown(si);
    SI4463_free(si);

    return 0;
}
