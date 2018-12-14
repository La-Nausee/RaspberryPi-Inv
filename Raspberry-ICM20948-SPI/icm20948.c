#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <wiringPi.h>

#include "Icm20948Defs.h"

#include "icm20948.h"
#include "spi_if.h"

static int gyro_bias[3] = {0,0,0}, accel_bias[3] = {0,0,0};
static unsigned char gyro_smpdiv = 0; //sampling rate = 1125/(1+smpdiv), 8bits
static unsigned int  acc_smpdiv  = 0; //sampling rate = 1125/(1+smpdiv), 12 bits
static unsigned char accel_fsr, gyro_fsr;
static unsigned char accel_lpf = ACCEL_DLPF_50;
static unsigned char gyro_lpf = GYRO_DLPF_51;
static float gres, ares, mres;
static float magXcoef , magYcoef , magZcoef ;

void calibrateIMU();

static void selectBank(unsigned char bank)
{
	unsigned char value = bank<<4;
	icm_write_reg(REG_BANK_SEL,&value,1);
}

void readAK09916Register(unsigned char reg, unsigned char count, unsigned char* buffer)
{
	unsigned char value;
	selectBank(3);
	delay(1);
	value = AK09916_ADDRESS|0x80;
	icm_write_reg(REG_I2C_SLV0_ADDR,&value,1);
	delay(1);
	value = reg;
	icm_write_reg(REG_I2C_SLV0_REG,&value,1);
	delay(1);
	value = 0xff;
	icm_write_reg(REG_I2C_SLV0_DO,&value,1);
	value = 0x80|count;
	icm_write_reg(REG_I2C_SLV0_CTRL,&value,1);
	delay(1);
	selectBank(0);
	icm_read_reg(REG_EXT_SLV_SENS_DATA_00,buffer,count);
	delay(1);
}

void writeAK09916Register(unsigned char reg, unsigned char data)
{
	unsigned char value;
	unsigned char buf;
	selectBank(3);
	delay(1);
	value = AK09916_ADDRESS;
	icm_write_reg(REG_I2C_SLV0_ADDR,&value,1);
	delay(1);
	value = reg;
	icm_write_reg(REG_I2C_SLV0_REG,&value,1);
	delay(1);
	value = data;
	icm_write_reg(REG_I2C_SLV0_DO,&value,1);
	delay(1);
	value = 0x81;
	icm_write_reg(REG_I2C_SLV0_CTRL,&value,1);
	delay(1);
}

void configMPU()
{
	unsigned char value = 0x00;

	//calibrateIMU();
	
	selectBank(0);
	//sleep off
	value = 1<<6;
	icm_write_reg(REG_PWR_MGMT_1,&value,1);
	delay(100);
	//auto select clock source
	value = 0x01;
	icm_write_reg(REG_PWR_MGMT_1,&value,1);
	delay(100);
	selectBank(2);
	//gyro sample rate divider
	value = gyro_smpdiv;
	icm_write_reg(REG_GYRO_SMPLRT_DIV,&value,1);
	//DLPF_CFG and gyro full scale select, GYRO_FCHOICE=1
	gyro_fsr = GFS_CONFIG;
	value = (GYRO_DLPF_361<<3)|(gyro_fsr<<1)|0x01;
	icm_write_reg(REG_GYRO_CONFIG_1,&value,1);
	switch(GFS_CONFIG)
	{
		case GFS_250: 
		gres = 250.0/32768.0;
		break;
		case GFS_500: 
		gres = 500.0/32768.0;
		break;
		case GFS_1000: 
		gres = 1000.0/32768.0;
		break;
		case GFS_2000: 
		gres = 2000.0/32768.0;
		break;
	}
	//accel sample rate divider
	value = (acc_smpdiv>>8)&0x0F;
	icm_write_reg(REG_ACCEL_SMPLRT_DIV_1,&value,1);
	value = acc_smpdiv&0xFF;
	icm_write_reg(REG_ACCEL_SMPLRT_DIV_2,&value,1);
	//DLPF and accel full scale select, ACCEL_FCHOICE=1
	accel_fsr = AFS_CONFIG;
	switch(AFS_CONFIG)
	{
		case AFS_2G: 
		ares = 2.0/32768.0;
		break;
		case AFS_4G: 
		ares = 4.0/32768.0;
		break;
		case AFS_8G: 
		ares = 8.0/32768.0;
		break;
		case AFS_16G: 
		ares = 16.0/32768.0;
		break;
	}
	value = (ACCEL_DLPF_50<<3)|(AFS_CONFIG<<1)|0x01;
	icm_write_reg(REG_ACCEL_CONFIG,&value,1);
	selectBank(0);
	//Setup interrupt, 50 us pulse
	value = 0x00;
	icm_write_reg(REG_INT_PIN_CFG,&value,1);
	//Enable data ready (bit 0 ) interrupt
	value = 0x01;
	icm_write_reg(REG_INT_ENABLE_1,&value,1);
	delay(10);
}

void configAK09916()
{
	unsigned char value = 0x00;
	unsigned char buffer[8];
	mres = 4912.0/32752.0;
	magXcoef = magYcoef = magZcoef = 1.0;
	selectBank(0);
	//reset I2C Master
	value = 0x02;
	icm_write_reg(REG_USER_CTRL,&value,1);
	delay(10);
	//enable I2C Master mode
	value = 0x20;
	icm_write_reg(REG_USER_CTRL,&value,1);
	selectBank(3);
	//I2C configuration multi-master  IIC 400KHz
    value = 0x07;
	icm_write_reg(REG_I2C_MST_CTRL,&value,1);
	
	//REG_I2C_MST_ODR_CONFIG
    //value = 0x04;
	//icm_write_reg(REG_I2C_MST_ODR_CONFIG,&value,1);
	//I2C_SLV0 _DLY_ enable
    //value = 0x01;
	//icm_write_reg(REG_I2C_MST_DELAY_CTRL,&value,1);
	//reset AK8963
	writeAK09916Register(AK_REG_CNTL3,0x01);
	delay(100);
	//Continuous measurement mode  1, 10Hz
	//Continuous measurement mode  2, 20Hz
	//Continuous measurement mode  3, 50Hz
	//Continuous measurement mode  4, 100Hz
	writeAK09916Register(AK_REG_CNTL2,0x08);

	readAK09916Register(AK_REG_WIA2,1,&value);
	printf("AK WIA is 0x%2X\r\n",value);
	//ST2 register must be read in order to update sensor data
	readAK09916Register(AK_REG_HXL,8,buffer);
}

unsigned char checkDataReady()
{
	unsigned char value;
	selectBank(0);
	icm_read_reg(REG_INT_STATUS_1,&value,1);

	return value&0x01;
}

void readSensor(float* ax, float* ay, float* az,float* gx, float* gy, float* gz,float* mx, float* my, float* mz,float* temp)
{
	unsigned char buffer[22];
	int16_t iax,iay,iaz;
	int16_t iwx,iwy,iwz;
	int16_t imx,imy,imz;
	int16_t itemp;
	selectBank(0);
	icm_read_reg(REG_ACCEL_XOUT_H_SH,buffer,22);
	iax = buffer[1] | (buffer[0]<<8); 
	iay = buffer[3] | (buffer[2]<<8); 
	iaz = buffer[5] | (buffer[4]<<8); 
	*ax = iax * ares;
	*ay = iay * ares;
	*az = iaz * ares;
	iwx = buffer[7] | (buffer[6]<<8); 
	iwy = buffer[9] | (buffer[8]<<8); 
	iwz = buffer[11] | (buffer[10]<<8); 
	*gx = iwx * gres;
	*gy = iwy * gres;
	*gz = iwz * gres;
	itemp = buffer[13] | (buffer[12]<<8); 
	*temp = itemp / 333.87 + 21.0;
	imx = buffer[14] | (buffer[15]<<8); 
	imy = buffer[16] | (buffer[17]<<8); 
	imz = buffer[18] | (buffer[19]<<8); 
	*mx = imx * mres * magXcoef;
	*my = imy * mres * magYcoef;
	*mz = imz * mres * magZcoef;
}

void setAccelFSR(unsigned char fsr)
{
	unsigned char value;
	switch(fsr)
	{
		case AFS_2G: 
		ares = 2.0/32768.0;
		break;
		case AFS_4G: 
		ares = 4.0/32768.0;
		break;
		case AFS_8G: 
		ares = 8.0/32768.0;
		break;
		case AFS_16G: 
		ares = 16.0/32768.0;
		break;
	}
	
	accel_fsr = fsr;
	value = (ACCEL_DLPF_50<<3)|(fsr<<1)|0x01;
	selectBank(2);
	icm_write_reg(REG_ACCEL_CONFIG,&value,1);
}

void setGyroFSR(unsigned char fsr)
{
	unsigned char value;
	switch(fsr)
	{
		case GFS_250: 
		gres = 250.0/32768.0;
		break;
		case GFS_500: 
		gres = 500.0/32768.0;
		break;
		case GFS_1000: 
		gres = 1000.0/32768.0;
		break;
		case GFS_2000: 
		gres = 2000.0/32768.0;
		break;
	}
	gyro_fsr = fsr;
	value = (GYRO_DLPF_51<<3)|(fsr<<1)|0x01;
	selectBank(2);
	icm_write_reg(REG_GYRO_CONFIG_1,&value,1);
}

void setLPF(unsigned char acc_lpf, unsigned char gyro_lpf)
{
	unsigned char value;
	
	accel_lpf = acc_lpf;
	gyro_lpf = gyro_lpf;
	
	selectBank(2);

	value = (accel_lpf<<3)|(accel_fsr<<1)|0x01;
	icm_write_reg(REG_ACCEL_CONFIG,&value,1);
	
	value = (gyro_lpf<<3)|(gyro_fsr<<1)|0x01;
	icm_write_reg(REG_GYRO_CONFIG_1,&value,1);
}

//output data rate, 1~1125Hz
void setAccelODR(unsigned int odr)
{
	unsigned char value;
	if(odr>1125)
	{
		odr = 1125;
	}
	else if(odr<1)	
	{
		odr = 1;
	}
	
	selectBank(2);
	
	acc_smpdiv = 1125/odr - 1;
	value = (acc_smpdiv>>8)&0x0F;
	icm_write_reg(REG_ACCEL_SMPLRT_DIV_1,&value,1);
	value = acc_smpdiv&0xFF;
	icm_write_reg(REG_ACCEL_SMPLRT_DIV_2,&value,1);
}

//output data rate, 5~1125Hz	
void setGyroODR(unsigned int odr)
{
	unsigned char value;
	if(odr>1125)
	{
		odr = 1125;
	}
	else if(odr<5)	
	{
		odr = 5;
	}
	
	selectBank(2);
	gyro_smpdiv = 1125/odr - 1;
	value = gyro_smpdiv;
	icm_write_reg(REG_GYRO_SMPLRT_DIV,&value,1);
}

void enableInterrupt()
{
	unsigned char value;
	
	selectBank(0);
	//Setup interrupt, 50 us pulse
	value = 0x00;
	icm_write_reg(REG_INT_PIN_CFG,&value,1);
	//Enable data ready (bit 0 ) interrupt
	value = 0x01;
	icm_write_reg(REG_INT_ENABLE_1,&value,1);
}

void disableInterrupt()
{
	unsigned char value;
	selectBank(0);
	//Disable data ready (bit 0 ) interrupt
	value = 0x00;
	icm_write_reg(REG_INT_ENABLE_1,&value,1);
}

//Do this before configMPU9250
void calibrateIMU()
{
	unsigned char value = 0x00;
	unsigned char buffer[12];
	unsigned int ii, packet_count, fifo_count;

	selectBank(0);
	//reset device
	value = 0x80;
	icm_write_reg(REG_PWR_MGMT_1,&value,1);
	delay(100);
	
	value = 0x01;
	icm_write_reg(REG_PWR_MGMT_1,&value,1);
	value = 0x00;
	icm_write_reg(REG_PWR_MGMT_2,&value,1);
	delay(200);
	
	//configure device for bias calculation
	value = 0x00;
	icm_write_reg(REG_INT_ENABLE,&value,1); //disable all interrupts
	icm_write_reg(REG_INT_ENABLE_1,&value,1); //disable all interrupts
	icm_write_reg(REG_INT_ENABLE_2,&value,1); //disable all interrupts
	icm_write_reg(REG_INT_ENABLE_3,&value,1); //disable all interrupts
	icm_write_reg(REG_FIFO_EN,&value,1);    //disable FIFO
	icm_write_reg(REG_FIFO_EN_2,&value,1);    //disable FIFO
	icm_write_reg(REG_PWR_MGMT_1,&value,1); //turn on internal clock source
	selectBank(3);
	icm_write_reg(REG_I2C_MST_CTRL,&value,1); //disable I2Cmaster
	selectBank(0);
	icm_write_reg(REG_USER_CTRL,&value,1); //disable FIFO and I2C master mode
	value = 0x1F;
	icm_write_reg(REG_FIFO_RST,&value,1); //reset FIFO
	value = 0x08;
	icm_write_reg(REG_USER_CTRL,&value,1); //reset DMP
	
	//configure MPU6050 gyro and accel for bias calculation
	//set low passs filter to 188Hz 
	// set gyro full scale to 250 dps
	//set accel full scale to 2 g
	selectBank(2);
	value = (ACCEL_DLPF_246<<3)|(AFS_2G<<1)|0x01;
	icm_write_reg(REG_ACCEL_CONFIG,&value,1);
	value = (GYRO_DLPF_196<<3)|(GFS_250<<1)|0x01;
	icm_write_reg(REG_GYRO_CONFIG_1,&value,1);
	
	value = 0x00;
	icm_write_reg(REG_GYRO_SMPLRT_DIV,&value,1);//set sample rate to 1.125KHz
	icm_write_reg(REG_ACCEL_SMPLRT_DIV_1,&value,1);
	icm_write_reg(REG_ACCEL_SMPLRT_DIV_2,&value,1);
	
	unsigned int gyro_res = 131; // 131LSB /dps
	unsigned int accel_res = 16384; // 16384LSB/g
	
	//configure FIFO to capture accel and gyro for bias calculation
	selectBank(3);
	value = 0x40;
	icm_write_reg(REG_USER_CTRL,&value,1); //Enable FIFO
	selectBank(0);
	value = 0x1E;
	icm_write_reg(REG_FIFO_EN_2,&value,1);  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	
	delay(40);
	
	//At end of sample accumulation, turn off FIFO sensor read
	value = 0x00;
	icm_write_reg(REG_FIFO_EN_2,&value,1); // Disable gyro and accelerometer sensors for FIFO
	icm_read_reg(REG_FIFO_COUNT_H,buffer,2); // read FIFO sample count
	fifo_count = ((uint16_t)buffer[0] << 8) | buffer[1];
	printf("FIFO count %d\r\n",fifo_count);
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	  
	for (ii = 0; ii < packet_count; ii++) 
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		icm_read_reg(REG_FIFO_R_W,buffer,12); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)buffer[0] << 8) | buffer[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)buffer[2] << 8) | buffer[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)buffer[4] << 8) | buffer[5]  ) ;    
		gyro_temp[0]  = (int16_t) (((int16_t)buffer[6] << 8) | buffer[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)buffer[8] << 8) | buffer[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)buffer[10] << 8) | buffer[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];		
	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
	
	if(accel_bias[2] > 0L) 
	{
		accel_bias[2] -= (int32_t) accel_res;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else 
	{
		accel_bias[2] += (int32_t) accel_res;
	}
	
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	buffer[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	buffer[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	buffer[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	buffer[3] = (-gyro_bias[1]/4)       & 0xFF;
	buffer[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	buffer[5] = (-gyro_bias[2]/4)       & 0xFF;
	selectBank(2);
	icm_write_reg(REG_XG_OFFS_USRH,buffer,1);
	icm_write_reg(REG_XG_OFFS_USRL,buffer+1,1);
	icm_write_reg(REG_YG_OFFS_USRH,buffer+2,1);
	icm_write_reg(REG_YG_OFFS_USRL,buffer+3,1);
	icm_write_reg(REG_ZG_OFFS_USRH,buffer+4,1);
	icm_write_reg(REG_ZG_OFFS_USRL,buffer+5,1);
	
	printf("\r\nscaled gyro bias: X:%f, Y:%f, Z:%f\r\n",(float)gyro_bias[0]/gyro_res,(float)gyro_bias[1]/gyro_res,(float)gyro_bias[2]/gyro_res);
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	selectBank(1);
	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	icm_read_reg(REG_XA_OFFS_H,buffer,2); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)buffer[0] << 8) | buffer[1]);
	icm_read_reg(REG_YA_OFFS_H,buffer,2);
	accel_bias_reg[1] = (int32_t) (((int16_t)buffer[0] << 8) | buffer[1]);
	icm_read_reg(REG_ZA_OFFS_H,buffer,2);
	accel_bias_reg[2] = (int32_t) (((int16_t)buffer[0] << 8) | buffer[1]);
	
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
	for(ii = 0; ii < 3; ii++) 
	{
		if((accel_bias_reg[ii] & mask)) 
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}
	
	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	buffer[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	buffer[1] = (accel_bias_reg[0])      & 0xFF;
	buffer[1] = buffer[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	buffer[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	buffer[3] = (accel_bias_reg[1])      & 0xFF;
	buffer[3] = buffer[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	buffer[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	buffer[5] = (accel_bias_reg[2])      & 0xFF;
	buffer[5] = buffer[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers

	/*  writeByte(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, buffer[0]);
	  writeByte(MPU9250_ADDRESS, MPU9250_XA_OFFSET_L, buffer[1]);
	  writeByte(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, buffer[2]);
	  writeByte(MPU9250_ADDRESS, MPU9250_YA_OFFSET_L, buffer[3]);
	  writeByte(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, buffer[4]);
	  writeByte(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_L, buffer[5]);
	*/
	icm_write_reg(REG_XA_OFFS_H,buffer,1);
	icm_write_reg(REG_XA_OFFS_L,buffer+1,1);
	icm_write_reg(REG_YA_OFFS_H,buffer+2,1);
	icm_write_reg(REG_YA_OFFS_L,buffer+3,1);
	icm_write_reg(REG_ZA_OFFS_H,buffer+4,1);
	icm_write_reg(REG_ZA_OFFS_L,buffer+5,1);
	printf("\r\nscaled accel bias: X:%f, Y:%f, Z:%f\r\n",(float)accel_bias[0]/accel_res,(float)accel_bias[1]/accel_res,(float)accel_bias[2]/accel_res);

	/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	*/
}

