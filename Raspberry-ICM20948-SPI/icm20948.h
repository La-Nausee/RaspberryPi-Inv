#ifndef ICM20948_H
#define ICM20948_H

#include <stdio.h>
#include <stdint.h>

#define ICM_INTERRUPT_PIN        29
#define AK09916_ADDRESS          0x0C

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

//#define ACCEL_DLPF_246   0x00
#define ACCEL_DLPF_246   0x01
#define ACCEL_DLPF_111   0x02
#define ACCEL_DLPF_50    0x03
#define ACCEL_DLPF_23    0x04
#define ACCEL_DLPF_11    0x05
#define ACCEL_DLPF_5     0x06
#define ACCEL_DLPF_473   0x07
#define GYRO_DLPF_196    0x00
#define GYRO_DLPF_151    0x01
#define GYRO_DLPF_119    0x02
#define GYRO_DLPF_51     0x03
#define GYRO_DLPF_23     0x04
#define GYRO_DLPF_11     0x05
#define GYRO_DLPF_5      0x06
#define GYRO_DLPF_361    0x07

//Continous data output 8Hz
#define AK09916_MODE_C8HZ     0x02
//Continous data output 100Hz
#define AK09916_MODE_C100HZ   0x06

#define GFS_CONFIG   GFS_2000
#define AFS_CONFIG   AFS_2G
#define AK09916FASTMODE

void configMPU();
void configAK09916();
unsigned char checkDataReady();
void readSensor(float* ax, float* ay, float* az,float* gx, float* gy, float* gz,float* mx, float* my, float* mz,float* temp);
void setAccelFSR(unsigned char fsr);
void setGyroFSR(unsigned char fsr);
void setLPF(unsigned char acc_lpf, unsigned char gyro_lpf);
void setAccelODR(unsigned int odr);
void setGyroODR(unsigned int odr);
void enableInterrupt();
void disableInterrupt();

#endif
