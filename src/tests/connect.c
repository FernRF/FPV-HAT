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
    static int SPI_Speed = 100;
    static int GPIO_PowerDown = 0;
    static int GPIO_gpio1 = 3;
    static int GPIO_interrupt = 2;

    si4463_t si;

    wiringPiSetup();

//    spiSetup();
//    gpioSetup();

    si = SI4463_init(SPI_Channel, GPIO_gpio1, GPIO_PowerDown, GPIO_interrupt, SPI_Speed);
    if (si == (si4463_t) -1)
    {
        fprintf(stderr, "Init failed!\n");
        return -1;
    }

    SI4463_reset(si);
    SI4463_free(si);

    return 0;
}
