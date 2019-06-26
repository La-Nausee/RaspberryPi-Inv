#ifndef SPI_IF_H
#define SPI_IF_H

#include <stdio.h>
#include <stdint.h>
#include "Invn/Devices/HostSerif.h"

#define ADIS16448_FLASH_CNT  0x00 /* Flash memory write count */
#define ADIS16448_XGYRO_OUT 0x04 /* X-axis gyroscope output */
#define ADIS16448_YGYRO_OUT 0x06 /* Y-axis gyroscope output */
#define ADIS16448_ZGYRO_OUT 0x08 /* Z-axis gyroscope output */
#define ADIS16448_XACCL_OUT 0x0A /* X-axis accelerometer output */
#define ADIS16448_YACCL_OUT 0x0C /* Y-axis accelerometer output */
#define ADIS16448_ZACCL_OUT 0x0E /* Z-axis accelerometer output */
#define ADIS16448_XMAGN_OUT 0x10 /* X-axis magnetometer measurement */
#define ADIS16448_YMAGN_OUT 0x12 /* Y-axis magnetometer measurement */
#define ADIS16448_ZMAGN_OUT 0x14 /* Z-axis magnetometer measurement */
#define ADIS16448_BARO_OUT  0x16 /* Barometer pressure output */
#define ADIS16448_TEMP_OUT  0x18 /* Temperature output */

/* Calibration parameters */
#define ADIS16448_XGYRO_OFF 0x1A /* X-axis gyroscope bias offset factor */
#define ADIS16448_YGYRO_OFF 0x1C /* Y-axis gyroscope bias offset factor */
#define ADIS16448_ZGYRO_OFF 0x1E /* Z-axis gyroscope bias offset factor */
#define ADIS16448_XACCL_OFF 0x20 /* X-axis acceleration bias offset factor */
#define ADIS16448_YACCL_OFF 0x22 /* Y-axis acceleration bias offset factor */
#define ADIS16448_ZACCL_OFF 0x24 /* Z-axis acceleration bias offset factor */
#define ADIS16448_XMAGN_HIF 0x26 /* X-axis magnetometer, hard-iron factor */
#define ADIS16448_YMAGN_HIF 0x28 /* Y-axis magnetometer, hard-iron factor */
#define ADIS16448_ZMAGN_HIF 0x2A /* Z-axis magnetometer, hard-iron factor */
#define ADIS16448_XMAGN_SIF 0x2C /* X-axis magnetometer, soft-iron factor */
#define ADIS16448_YMAGN_SIF 0x2E /* Y-axis magnetometer, soft-iron factor */
#define ADIS16448_ZMAGN_SIF 0x30 /* Z-axis magnetometer, soft-iron factor */

#define ADIS16448_GPIO_CTRL 0x32 /* Auxiliary digital input/output control */
#define ADIS16448_MSC_CTRL  0x34 /* Miscellaneous control */
#define ADIS16448_SMPL_PRD  0x36 /* Internal sample period (rate) control */
#define ADIS16448_SENS_AVG  0x38 /* Dynamic range and digital filter control */
#define ADIS16448_SLP_CNT   0x3A /* Sleep mode control */
#define ADIS16448_DIAG_STAT 0x3C /* System status */

/* Alarm functions */
#define ADIS16448_GLOB_CMD  0x3E /* System command */
#define ADIS16448_ALM_MAG1  0x40 /* Alarm 1 amplitude threshold */
#define ADIS16448_ALM_MAG2  0x42 /* Alarm 2 amplitude threshold */
#define ADIS16448_ALM_SMPL1 0x44 /* Alarm 1 sample size */
#define ADIS16448_ALM_SMPL2 0x46 /* Alarm 2 sample size */
#define ADIS16448_ALM_CTRL  0x48 /* Alarm control */

#define ADIS16448_LOT_ID1   0x52 /* Lot identification code 1 */
#define ADIS16448_LOT_ID2   0x54 /* Lot identification code 2 */
#define ADIS16448_PRODUCT_ID 0x56 /* Product identifier */
#define ADIS16448_SERIAL_NUMBER 0x58 /* Serial number, lot specific */

#define ADIS16448_ERROR_ACTIVE			(1<<14)
#define ADIS16448_NEW_DATA			(1<<14)

/* MSC_CTRL */
#define ADIS16448_MSC_CTRL_MEM_TEST		(1<<11)
#define ADIS16448_MSC_CTRL_INT_SELF_TEST	(1<<10)
#define ADIS16448_MSC_CTRL_ACCL_ALIGN		(1<<6)
#define ADIS16448_MSC_CTRL_INC_CRC_BUSRT	(1<<4)
#define ADIS16448_MSC_CTRL_DATA_RDY_EN		(1<<2)
#define ADIS16448_MSC_CTRL_DATA_RDY_POL_HIGH	(1<<1)
#define ADIS16448_MSC_CTRL_DATA_RDY_DIO2	(1<<0)

/* SMPL_PRD */
#define ADIS16400_SMPL_PRD_TIME_BASE	(1<<7)
#define ADIS16400_SMPL_PRD_DIV_MASK	0x7F

/* DIAG_STAT */
#define ADIS16448_DIAG_STAT_ZACCL_FAIL	15
#define ADIS16448_DIAG_STAT_YACCL_FAIL	14
#define ADIS16448_DIAG_STAT_XACCL_FAIL	13
#define ADIS16448_DIAG_STAT_XGYRO_FAIL	12
#define ADIS16448_DIAG_STAT_YGYRO_FAIL	11
#define ADIS16448_DIAG_STAT_ZGYRO_FAIL	10
#define ADIS16448_DIAG_STAT_ALARM2	9
#define ADIS16448_DIAG_STAT_ALARM1	8
#define ADIS16448_DIAG_NEW_DATA		7
#define ADIS16448_DIAG_STAT_FLASH_CHK	6
#define ADIS16448_DIAG_STAT_SELF_TEST	5
#define ADIS16448_DIAG_STAT_OVERFLOW	4
#define ADIS16448_DIAG_STAT_SPI_FAIL	3
#define ADIS16448_DIAG_STAT_FLASH_UPT	2
#define ADIS16448_DIAG_STAT_BARO_TEST	1
#define ADIS16448_DIAG_STAT_MAG_TEST	0

/* GLOB_CMD */
#define ADIS16448_GLOB_CMD_SW_RESET	(1<<7)
#define ADIS16448_GLOB_CMD_FLASH_UPD	(1<<3)
#define ADIS16448_GLOB_CMD_FAC_CALIB	(1<<1)
#define ADIS16448_GLOB_CMD_AUTO_NULL	(1<<0)

/* SLP_CNT */
#define ADIS16400_SLP_CNT_POWER_OFF	(1<<8)

#define ADIS16448_SPI_SLOW	(300 * 1000)
#define ADIS16448_SPI_BURST	(1000 * 1000)
#define ADIS16448_SPI_FAST	(2000 * 1000)

#define ADIS16448_HAS_PROD_ID		BIT(0)
#define ADIS16448_NO_BURST		BIT(1)
#define ADIS16448_HAS_SLOW_MODE		BIT(2)
#define ADIS16448_HAS_SERIAL_NUMBER	BIT(3)
#define ADIS16448_BURST_DIAG_STAT	BIT(4)

#define ICM20948_SPI_SPEED	(7000*1000)

typedef struct{
	uint16_t diag_stat;
	int16_t xgyro_out;
	int16_t ygyro_out;
	int16_t zgyro_out;
	int16_t xaccl_out;
	int16_t yaccl_out;
	int16_t zaccl_out;
	int16_t xmagn_out;
	int16_t ymagn_out;
	int16_t zmagn_out;
	uint16_t baro_out;
	uint16_t temp_out;
}adis_raw_t;

typedef struct{
	float ax,ay,az;
	float gx,gy,gz;
	float mx,my,mz;
	float baro,temp;
}adis_data_t;

void spi_init();
const inv_host_serif_t * icm_get_serif_instance_spi(void);
#endif
