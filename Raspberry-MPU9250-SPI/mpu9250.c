#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <wiringPi.h>

#include "MPU9250_RegisterMap.h"

#include "mpu9250.h"
#include "spi_if.h"

static unsigned char smpdiv = 0; //sampling rate = 1000/(1+smpdiv)
static unsigned char accel_fsr, gyro_fsr;
static unsigned char accel_lpf = ACCEL_DLPF_41;
static unsigned char gyro_lpf = GYRO_DLPF_41;
static float gres, ares, mres;
static float magXcoef, magYcoef, magZcoef;

void calibrateIMU();

void readAK8963Register(unsigned char reg, unsigned char count, unsigned char* buffer)
{
	unsigned char value;
	value = AK8963_ADDRESS|0x80;
	mpu_write_reg(MPU9250_I2C_SLV0_ADDR,&value,1);
	value = reg;
	mpu_write_reg(MPU9250_I2C_SLV0_REG,&value,1);
	value = 0x80|count;
	mpu_write_reg(MPU9250_I2C_SLV0_CTRL,&value,1);
	delayMicroseconds(50);
	mpu_read_reg(MPU9250_EXT_SENS_DATA_00,buffer,count);
}

void writeAK8963Register(unsigned char reg, unsigned char data)
{
	unsigned char value;
	unsigned char buf;
	value = AK8963_ADDRESS;
	mpu_write_reg(MPU9250_I2C_SLV0_ADDR,&value,1);
	value = reg;
	mpu_write_reg(MPU9250_I2C_SLV0_REG,&value,1);
	value = data;
	mpu_write_reg(MPU9250_I2C_SLV0_DO,&value,1);
	value = 0x81;
	mpu_write_reg(MPU9250_I2C_SLV0_CTRL,&value,1);
	delay(100);
	//read back and confirm
	readAK8963Register(reg,1,&buf);
	if(buf != data)
	{
	//	printf("error: write AK9863 register\r\n");
	}
}

void configMPU9250()
{
	unsigned char value = 0x00;
	calibrateIMU();
	//calibrateIMU();
	//sleep off
	mpu_write_reg(MPU9250_PWR_MGMT_1,&value,1);
	delay(100);
	//auto select clock source
	value = 0x01;
	mpu_write_reg(MPU9250_PWR_MGMT_1,&value,1);
	delay(100);
	//DLPF_CFG
	value = GYRO_DLPF_41;
	mpu_write_reg(MPU9250_CONFIG,&value,1);
	//sample rate divider
	value = smpdiv;
	mpu_write_reg(MPU9250_SMPLRT_DIV,&value,1);
	//gyro full scale select
	gyro_fsr = GFS_CONFIG;
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
	value = GFS_CONFIG<<3;
	mpu_write_reg(MPU9250_GYRO_CONFIG,&value,1);
	//accel full scale select
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
	value = AFS_CONFIG<<3;
	mpu_write_reg(MPU9250_ACCEL_CONFIG,&value,1);
	//A_DLPFCFG
	value = ACCEL_DLPF_41;
	mpu_write_reg(MPU9250_ACCEL_CONFIG_2,&value,1);
	//Setup interrupt, 50 us pulse
	value = 0x00;
	mpu_write_reg(MPU9250_INT_PIN_CFG,&value,1);
	//Enable data ready (bit 0 ) interrupt
	value = 0x01;
	mpu_write_reg(MPU9250_INT_ENABLE,&value,1);
	delay(100);
}

void configAK8963()
{
	unsigned char value = 0x00;
	unsigned char buffer[7];
	switch(AK8963_BIT_CONFIG)
	{
		case AK8963_BIT_14:
		mres = 4912.0/8190.0;
		break;
		case AK8963_BIT_16:
		mres = 4912.0/32760.0;
		break;
	}
	//enable I2C Master mode
	value = 0x20;
	mpu_write_reg(MPU9250_USER_CTRL,&value,1);
	//I2C configuration multi-master  IIC 400KHz
	value = 0x0D;
	mpu_write_reg(MPU9250_I2C_MST_CTRL,&value,1);
	//set AK8963 Power Down
	writeAK8963Register(AK8963_CNTL,0x00);
	delay(100);
	//reset AK8963
	writeAK8963Register(AK8963_RSV,0x01);
	//enable I2C Master mode
	value = 0x20;
	mpu_write_reg(MPU9250_USER_CTRL,&value,1);
	//set AK8963 Power Down
	writeAK8963Register(AK8963_CNTL,0x00);
	//set FUSE ROM access
	writeAK8963Register(AK8963_CNTL,0x0F);
	delay(100);
	readAK8963Register(AK8963_ASAX,3,buffer);
	
	magXcoef = (buffer[0] - 128) / 256.0 + 1.0;
	magYcoef = (buffer[1] - 128) / 256.0 + 1.0;
	magZcoef = (buffer[2] - 128) / 256.0 + 1.0;
	
	//set AK8963 Power Down
	writeAK8963Register(AK8963_CNTL,0x00);
	delay(100);
	//set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL,0x16);
	delay(100);
	readAK8963Register(AK8963_HXL,7,buffer);
}

unsigned char checkDataReady()
{
	unsigned char value;
	mpu_read_reg(MPU9250_INT_STATUS,&value,1);

	return value&0x01;
}

void readSensor(float* ax, float* ay, float* az,float* gx, float* gy, float* gz,float* mx, float* my, float* mz,float* temp)
{
	unsigned char buffer[21];
	int16_t iax,iay,iaz;
	int16_t iwx,iwy,iwz;
	int16_t imx,imy,imz;
	int16_t itemp;
	mpu_read_reg(MPU9250_ACCEL_XOUT_H,buffer,21);
	iax = buffer[1] | (buffer[0]<<8); 
	iay = buffer[3] | (buffer[2]<<8); 
	iaz = buffer[5] | (buffer[4]<<8); 
	*ax = iax * ares;
	*ay = iay * ares;
	*az = iaz * ares;
	itemp = buffer[7] | (buffer[6]<<8); 
	*temp = itemp / 333.87 + 21.0;
	iwx = buffer[9] | (buffer[8]<<8); 
	iwy = buffer[11] | (buffer[10]<<8); 
	iwz = buffer[13] | (buffer[12]<<8); 
	*gx = iwx * gres;
	*gy = iwy * gres;
	*gz = iwy * gres;
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
	value = fsr<<3;
	mpu_write_reg(MPU9250_ACCEL_CONFIG,&value,1);
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
	value = fsr<<3;
	mpu_write_reg(MPU9250_GYRO_CONFIG,&value,1);
}

void setLPF(unsigned char lpf)
{
	if(lpf == 184)
	{
		accel_lpf = ACCEL_DLPF_184;
		gyro_lpf = GYRO_DLPF_184;
	}
	else if(lpf == 92)
	{
		accel_lpf = ACCEL_DLPF_92;
		gyro_lpf = GYRO_DLPF_92;
	}
	else if(lpf == 41)
	{
		accel_lpf = ACCEL_DLPF_41;
		gyro_lpf = GYRO_DLPF_41;
	}
	else if(lpf == 20)
	{
		accel_lpf = ACCEL_DLPF_20;
		gyro_lpf = GYRO_DLPF_20;
	}
	else if(lpf == 10)
	{
		accel_lpf = ACCEL_DLPF_10;
		gyro_lpf = GYRO_DLPF_10;
	}
	else if(lpf == 5)
	{
		accel_lpf = ACCEL_DLPF_5;
		gyro_lpf = GYRO_DLPF_5;
	}
	mpu_write_reg(MPU9250_ACCEL_CONFIG_2,&accel_lpf,1);
	mpu_write_reg(MPU9250_CONFIG,&gyro_lpf,1);
}

//output data rate, 4~1000Hz
void setODR(unsigned int odr)
{
	unsigned char div = 0;
	unsigned char value;
	unsigned char buffer[7];
	if(odr>1000)
	{
		odr = 1000;
	}
	else if(odr<4)	
	{
		odr = 4;
	}
	
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	div = 19;
	mpu_write_reg(MPU9250_SMPLRT_DIV,&div,1);
	
	div = 1000/odr - 1;
	
	if(div > 9)
	{
		//set AK8963 Power Down
		writeAK8963Register(AK8963_CNTL,0x00);
		delay(100);
		//set AK8963 to 14 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL,0x12);
		delay(100);
		readAK8963Register(AK8963_HXL,7,buffer);
	}
	else
	{
		//set AK8963 Power Down
		writeAK8963Register(AK8963_CNTL,0x00);
		delay(100);
		//set AK8963 to 14 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL,0x16);
		delay(100);
		readAK8963Register(AK8963_HXL,7,buffer);
	}
	//sample rate divider
	smpdiv = div;
	value = div;
	mpu_write_reg(MPU9250_SMPLRT_DIV,&value,1);
}

void enableInterrupt()
{
	unsigned char value;
	//Setup interrupt, 50 us pulse
	value = 0x00;
	mpu_write_reg(MPU9250_INT_PIN_CFG,&value,1);
	//Enable data ready (bit 0 ) interrupt
	value = 0x01;
	mpu_write_reg(MPU9250_INT_ENABLE,&value,1);
}

void disableInterrupt()
{
	unsigned char value;
	//Disable data ready (bit 0 ) interrupt
	value = 0x00;
	mpu_write_reg(MPU9250_INT_ENABLE,&value,1);
}

//Do this before configMPU9250
void calibrateIMU()
{
	unsigned char value = 0x00;
	unsigned char buffer[12];
	unsigned int ii, packet_count, fifo_count;
	int gyro_bias[3] = {0,0,0}, accel_bias[3] = {0,0,0};
	
	//reset device
	value = 0x80;
	mpu_write_reg(MPU9250_PWR_MGMT_1,&value,1);
	delay(100);
	
	value = 0x01;
	mpu_write_reg(MPU9250_PWR_MGMT_1,&value,1);
	value = 0x00;
	mpu_write_reg(MPU9250_PWR_MGMT_2,&value,1);
	delay(200);
	
	//configure device for bias calculation
	value = 0x00;
	mpu_write_reg(MPU9250_INT_ENABLE,&value,1); //disable all interrupts
	mpu_write_reg(MPU9250_FIFO_EN,&value,1);    //disable FIFO
	mpu_write_reg(MPU9250_PWR_MGMT_1,&value,1); //turn on internal clock source
	mpu_write_reg(MPU9250_I2C_MST_CTRL,&value,1); //disable I2Cmaster
	mpu_write_reg(MPU9250_USER_CTRL,&value,1); //disable FIFO and I2C master mode
	value = 0x0C;
	mpu_write_reg(MPU9250_USER_CTRL,&value,1); //reset FIFO and DMP
	
	//configure MPU6050 gyro and accel for bias calculation
	value = 0x01;
	mpu_write_reg(MPU9250_CONFIG,&value,1); //set low passs filter to 188Hz
	value = 0x00;
	mpu_write_reg(MPU9250_SMPLRT_DIV,&value,1); //set sample rate to 1KHz
	mpu_write_reg(MPU9250_GYRO_CONFIG,&value,1);  // set gyro full scale to 250 dps
	mpu_write_reg(MPU9250_ACCEL_CONFIG,&value,1); //set accel full scale to 2 g
	
	unsigned int gyro_res = 131; // 131LSB /dps
	unsigned int accel_res = 16384; // 16384LSB/g
	
	//configure FIFO to capture accel and gyro for bias calculation
	value = 0x40;
	mpu_write_reg(MPU9250_USER_CTRL,&value,1); //Enable FIFO
	value = 0x78;
	mpu_write_reg(MPU9250_FIFO_EN,&value,1);  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	
	delay(40);
	
	//At end of sample accumulation, turn off FIFO sensor read
	value = 0x00;
	mpu_write_reg(MPU9250_FIFO_EN,&value,1); // Disable gyro and accelerometer sensors for FIFO
	mpu_read_reg(MPU9250_FIFO_COUNTH,buffer,2); // read FIFO sample count
	fifo_count = ((uint16_t)buffer[0] << 8) | buffer[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	  
	for (ii = 0; ii < packet_count; ii++) 
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		mpu_read_reg(MPU9250_FIFO_R_W,buffer,12); // read data for averaging
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
	mpu_write_reg(MPU9250_XG_OFFSET_H,buffer,1);
	mpu_write_reg(MPU9250_XG_OFFSET_L,buffer+1,1);
	mpu_write_reg(MPU9250_YG_OFFSET_H,buffer+2,1);
	mpu_write_reg(MPU9250_YG_OFFSET_L,buffer+3,1);
	mpu_write_reg(MPU9250_ZG_OFFSET_H,buffer+4,1);
	mpu_write_reg(MPU9250_ZG_OFFSET_L,buffer+5,1);
	
	printf("\r\nscaled gyro bias: X:%f, Y:%f, Z:%f\r\n",(float)gyro_bias[0]/gyro_res,(float)gyro_bias[1]/gyro_res,(float)gyro_bias[2]/gyro_res);
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	mpu_read_reg(MPU9250_XA_OFFSET_H,buffer,2); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)buffer[0] << 8) | buffer[1]);
	mpu_read_reg(MPU9250_YA_OFFSET_H,buffer,2);
	accel_bias_reg[1] = (int32_t) (((int16_t)buffer[0] << 8) | buffer[1]);
	mpu_read_reg(MPU9250_ZA_OFFSET_H,buffer,2);
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
	mpu_write_reg(MPU9250_XA_OFFSET_H,buffer,1);
        mpu_write_reg(MPU9250_XA_OFFSET_L,buffer+1,1);
        mpu_write_reg(MPU9250_YA_OFFSET_H,buffer+2,1);
        mpu_write_reg(MPU9250_YA_OFFSET_L,buffer+3,1);
        mpu_write_reg(MPU9250_ZA_OFFSET_H,buffer+4,1);
        mpu_write_reg(MPU9250_ZA_OFFSET_L,buffer+5,1);
	printf("\r\nscaled accel bias: X:%f, Y:%f, Z:%f\r\n",(float)accel_bias[0]/accel_res,(float)accel_bias[1]/accel_res,(float)accel_bias[2]/accel_res);
}

