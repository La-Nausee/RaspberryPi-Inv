#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "spi_if.h"

#define ADIS_RST_PIN      8
#define SPI_CE0           0
#define SPI_CE1           1
#define SPI_ICM_CE        2
#define SPI_SCLK_ADIS     ADIS16448_SPI_BURST //Hz
#define SPI_SCLK_ICM      ICM20948_SPI_SPEED//Hz

using namespace std;

void spi_init()
{
	int fd;
	int spiMode = 3;
     
	cout<< "Setup WiringPi" <<endl;	
	wiringPiSetup () ;
	
	pinMode(SPI_ICM_CE,OUTPUT);
	digitalWrite(SPI_ICM_CE, HIGH);

	fd = wiringPiSPISetup(SPI_CE1, SPI_SCLK_ICM);
	ioctl(fd,SPI_IOC_WR_MODE,&spiMode);
	
	cout << "ICM20948 spi setup result: " << fd << endl;
	
}

static int icm_spi_init(void)
{
	return 0;
}

static int icm_read_reg(uint8_t reg, const uint8_t * rbuffer, uint32_t rlen)
{
	uint8_t *buffer = (uint8_t*) malloc( (size_t)rlen+1);
	
	memset(buffer,0,rlen+1);
	buffer[0] = reg|0x80;

	digitalWrite(SPI_ICM_CE, LOW);
	wiringPiSPIDataRW(SPI_CE1,buffer,rlen+1);
	digitalWrite(SPI_ICM_CE, HIGH);
	
	memcpy(rbuffer,buffer+1,rlen);
	free(buffer);

	return 0;
} 

static int icm_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{

	uint8_t *buffer = (uint8_t*)malloc( (size_t)wlen+1);
	
	memset(buffer,0,wlen+1);
	buffer[0] = reg;
	memcpy(buffer+1,wbuffer,wlen);

	digitalWrite(SPI_ICM_CE, LOW);
	wiringPiSPIDataRW(SPI_CE1,buffer,wlen+1);
	digitalWrite(SPI_ICM_CE, HIGH);
	
	free(buffer);

	return 0;
} 

static const inv_host_serif_t serif_instance_spi = {
	icm_spi_init,
	0,
	icm_read_reg,
	icm_write_reg,
	0,
	1024*32,
	1024*32,
	INV_HOST_SERIF_TYPE_SPI,
};

const inv_host_serif_t* icm_get_serif_instance_spi(void)
{
	return &serif_instance_spi;
}
