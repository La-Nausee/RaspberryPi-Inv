#ifndef MPU9250_H
#define MPU9250_H

#include <stdio.h>
#include <stdint.h>

#define MPU_INTERRUPT_PIN       29
#define AK8963_ADDRESS          0x0C
#define DEVICE_ID		0x71

//Gyro Full Scale Select 250dps
#define GFS_250   0x00
//Gyro Full Scale Select 500dps
#define GFS_500   0x01
//Gyro Full Scale Select 1000dps
#define GFS_1000  0x02
//Gyro Full Scale Select 2000dps
#define GFS_2000  0x03
//Accel Full Scale Select 2G
#define AFS_2G    0x00
//Accel Full Scale Select 4G
#define AFS_4G    0x01
//Accel Full Scale Select 8G
#define AFS_8G    0x02
//Accel Full Scale Select 16G
#define AFS_16G   0x03

#define ACCEL_DLPF_184   0x01
#define ACCEL_DLPF_92    0x02
#define ACCEL_DLPF_41    0x03
#define ACCEL_DLPF_20    0x04
#define ACCEL_DLPF_10    0x05
#define ACCEL_DLPF_5     0x06
#define GYRO_DLPF_184    0x01
#define GYRO_DLPF_92     0x02
#define GYRO_DLPF_41     0x03
#define GYRO_DLPF_20     0x04
#define GYRO_DLPF_10     0x05
#define GYRO_DLPF_5      0x06

#define AK8963_BIT_14   0x00
#define AK8963_BIT_16   0x01
//Continous data output 8Hz
#define AK8963_MODE_C8HZ     0x02
//Continous data output 100Hz
#define AK8963_MODE_C100HZ   0x06

#define GFS_CONFIG   GFS_2000
#define AFS_CONFIG   AFS_2G
#define AK8963_BIT_CONFIG AK8963_BIT_16
#define AK8963FASTMODE

void configMPU9250();
void configAK8963();
unsigned char checkDataReady();
void readSensor(float* ax, float* ay, float* az,float* gx, float* gy, float* gz,float* mx, float* my, float* mz,float* temp);
void setAccelFSR(unsigned char fsr);
void setGyroFSR(unsigned char fsr);
void setLPF(unsigned char lpf);
void setODR(unsigned int odr);//output data rate, 4~1000Hz
void enableInterrupt();
void disableInterrupt();

#endif
