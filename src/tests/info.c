#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

//#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <linux/spi/spidev.h>

#include <sys/ioctl.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>


#include "si4463.h"
int main()
{
    static int SPI_Channel = 0;
    static int SPI_Speed = 10000;
    static int GPIO_PowerDown = 0;
    static int GPIO_gpio1 = 3;
    static int GPIO_interrupt = 2;

    si4463_t si;
    struct SI4463_Part_Info* info;
    struct SI4463_Func_Info* info2;

    wiringPiSetup();

    si = SI4463_init(SPI_Channel, GPIO_gpio1, GPIO_PowerDown, GPIO_interrupt, SPI_Speed);
    if (si == (si4463_t) -1)
    {
        fprintf(stderr, "Init failed!\n");
        return -1;
    }

    SI4463_reset(si);

    info = SI4463_part_info(si);
    if (info == NULL)
    {
        perror("Info failed!\n");
        return 1;
    }

    printf("SI 4463 Info:\n\t"
           "Chip Mask Revision: %02X\n\t"
           "Part Number: %04X\n\t"
           "Part Build: %02X\n\t"
           "ID: %04X\n\t"
           "Customer ID: %02X\n\t"
           "ROM ID: %02X\n",
           info->chip_rev,
           info->part,
           info->build,
           info->id,
           info->customer,
           info->romid);
    free(info);

    info2 = SI4463_funct_info(si);
    if (info2 == NULL)
    {
        perror("Info failed!\n");
        return 1;
    }
    printf("SI 4463 FUNC Info:\n\t"
           "External revision number: %02X\n\t"
           "Branch revision number: %02X\n\t"
           "Internal revision number: %02X\n\t"
           "ID of applied patch: %04X\n\t"
           "Current functional mode: %02X\n",
           info2->revext,
           info2->revbranch,
           info2->revint,
           info2->patch,
           info2->func);

    free(info2);


    SI4463_shutdown(si);
    SI4463_free(si);

    return 0;
}
