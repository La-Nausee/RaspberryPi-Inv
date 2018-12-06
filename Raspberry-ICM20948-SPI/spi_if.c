#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "spi_if.h"

#define SPI_CE0           0
#define SPI_CE1           1
#define SPI_SCLK_ICM	  7000000 //Hz

void spi_init()
{
	int fd;
	int spiMode = 3;
     
	printf("Setup WiringPi\r\n");
	wiringPiSetup () ;

	fd = wiringPiSPISetup(SPI_CE1, SPI_SCLK_ICM);
	ioctl(fd,SPI_IOC_WR_MODE,&spiMode);
	
	printf("ICM20948 spi setup result: %d\r\n",fd);
}

int icm_read_reg(uint8_t reg, const uint8_t * rbuffer, uint32_t rlen)
{
	uint8_t *buffer = (uint8_t*) malloc( (size_t)rlen+1);
	
	memset(buffer,0,rlen+1);
	buffer[0] = reg|0x80;

	wiringPiSPIDataRW(SPI_CE1,buffer,rlen+1);

	memcpy(rbuffer,buffer+1,rlen);
	free(buffer);

	return 0;
} 

int icm_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{

	uint8_t *buffer = (uint8_t*)malloc( (size_t)wlen+1);
	
	memset(buffer,0,wlen+1);
	buffer[0] = reg;
	memcpy(buffer+1,wbuffer,wlen);

	wiringPiSPIDataRW(SPI_CE1,buffer,wlen+1);

	free(buffer);

	return 0;
}
