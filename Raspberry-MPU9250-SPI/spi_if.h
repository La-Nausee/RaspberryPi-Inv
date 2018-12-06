#ifndef SPI_IF_H
#define SPI_IF_H

#include <stdio.h>
#include <stdint.h>

void spi_init();
int mpu_read_reg(uint8_t reg, const uint8_t * rbuffer, uint32_t rlen);
int mpu_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
#endif
