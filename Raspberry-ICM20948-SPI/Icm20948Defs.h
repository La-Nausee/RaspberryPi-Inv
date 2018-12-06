/*
* ________________________________________________________________________________________________________
* Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/

#ifndef _INV_ICM20948_DEFINES_H_
#define _INV_ICM20948_DEFINES_H_

#include <string.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// compass chip list
#define HW_AK8963 0x20
#define HW_AK8975 0x21
#define HW_AK8972 0x22
#define HW_AK09911 0x23
#define HW_AK09912 0x24
#define HW_AK09916 0x25

#define HW_ICM20648 0x01
#define HW_ICM20948 0x02

#define USE_ICM20948 1

#if defined USE_ICM20648
#define MEMS_CHIP  HW_ICM20648
#endif

#if defined USE_ICM20948
#define MEMS_CHIP  HW_ICM20948
#endif

#if !defined(MEMS_CHIP)
    #error "MEMS_CHIP is not defined"
#elif MEMS_CHIP != HW_ICM20648 \
        && MEMS_CHIP != HW_ICM20948
    #error "Unknown value for MEMS_CHIP"
#endif

#define DMP_LOAD_START 0x90

#define MPU_SUCCESS (0)
#define MPU_COMPASS_NOT_FOUND (int)0x00ABCDEF

#define MSEC_PER_SEC 1000
#define NSEC_PER_MSEC 1000000
#define NSEC_PER_SEC NSEC_PER_MSEC * MSEC_PER_SEC

#define FIFO_DIVIDER 19

#define REG_BANK_0 0x00
#define REG_BANK_1 0x01

#define DIAMOND_I2C_ADDRESS     0x68
#define BANK_0                  (0 << 7)
#define BANK_1                  (0 << 7)
#define BANK_2                  (0 << 7)
#define BANK_3                  (0 << 7)

/*register and associated bit definition*/
/* bank 0 register map */
#define REG_WHO_AM_I            (BANK_0 | 0x00)
#define REG_LPF                 (BANK_0 | 0x01)

#define REG_USER_CTRL           (BANK_0 | 0x03)
#define BIT_DMP_EN                      0x80
#define BIT_FIFO_EN                     0x40
#define BIT_I2C_MST_EN                  0x20
#define BIT_I2C_IF_DIS                  0x10
#define BIT_DMP_RST                     0x08
#define BIT_DIAMOND_DMP_RST			    0x04

#define REG_LP_CONFIG           (BANK_0 | 0x05)
#define BIT_I2C_MST_CYCLE               0x40
#define BIT_ACCEL_CYCLE                 0x20
#define BIT_GYRO_CYCLE                  0x10

#define REG_PWR_MGMT_1          (BANK_0 | 0x06)
#define BIT_H_RESET                     0x80
#define BIT_SLEEP                       0x40
#define BIT_LP_EN                       0x20
#define BIT_CLK_PLL                     0x01

#define REG_PWR_MGMT_2          (BANK_0 | 0x07)
#define BIT_PWR_PRESSURE_STBY           0x40
#define BIT_PWR_ACCEL_STBY              0x38
#define BIT_PWR_GYRO_STBY               0x07
#define BIT_PWR_ALL_OFF                 0x7f

#define REG_INT_PIN_CFG         (BANK_0 | 0x0F)
#define BIT_INT_LATCH_EN                0x20
#define BIT_BYPASS_EN                   0x02

#define REG_INT_ENABLE          (BANK_0 | 0x10)
#define BIT_DMP_INT_EN                  0x02

#define REG_INT_ENABLE_1        (BANK_0 | 0x11)
#define BIT_DATA_RDY_3_EN               0x08
#define BIT_DATA_RDY_2_EN               0x04
#define BIT_DATA_RDY_1_EN               0x02
#define BIT_DATA_RDY_0_EN               0x01

#define REG_INT_ENABLE_2        (BANK_0 | 0x12)
#define BIT_FIFO_OVERFLOW_EN_0          0x1

#define REG_INT_ENABLE_3        (BANK_0 | 0x13)

#define REG_DMP_INT_STATUS      (BANK_0 | 0x18)
#define BIT_WAKE_ON_MOTION_INT          0x08
#define BIT_MSG_DMP_INT                 0x0002
#define BIT_MSG_DMP_INT_0               0x0100  // CI Command

#define BIT_MSG_DMP_INT_2               0x0200  // CIM Command - SMD
#define BIT_MSG_DMP_INT_3               0x0400  // CIM Command - Pedometer

#define BIT_MSG_DMP_INT_4               0x1000  // CIM Command - Pedometer binning
#define BIT_MSG_DMP_INT_5               0x2000  // CIM Command - Bring To See Gesture
#define BIT_MSG_DMP_INT_6               0x4000  // CIM Command - Look To See Gesture

#define REG_INT_STATUS          (BANK_0 | 0x19)
#define BIT_DMP_INT                     0x02 

#define REG_INT_STATUS_1        (BANK_0 | 0x1A)
#define REG_INT_STATUS_2        (BANK_0 | 0x1B)

#define REG_SINGLE_FIFO_PRIORITY_SEL        (BANK_0 | 0x26)	

#define REG_GYRO_XOUT_H_SH      (BANK_0 | 0x33)

#define REG_TEMPERATURE         (BANK_0 | 0x39)
#define REG_TEMP_CONFIG         (BANK_0 | 0x53)

#define REG_EXT_SLV_SENS_DATA_00 (BANK_0 | 0x3B)
#define REG_EXT_SLV_SENS_DATA_08 (BANK_0 | 0x43)
#define REG_EXT_SLV_SENS_DATA_09 (BANK_0 | 0x44)
#define REG_EXT_SLV_SENS_DATA_10 (BANK_0 | 0x45)

#define REG_FIFO_EN             (BANK_0 | 0x66)
#define BIT_SLV_0_FIFO_EN               0x01

#define REG_FIFO_EN_2           (BANK_0 | 0x67)
#define BIT_PRS_FIFO_EN                 0x20
#define BIT_ACCEL_FIFO_EN               0x10
#define BITS_GYRO_FIFO_EN               0x0E

#define REG_FIFO_RST            (BANK_0 | 0x68)

#define REG_FIFO_COUNT_H        (BANK_0 | 0x70)
#define REG_FIFO_COUNT_L        (BANK_0 | 0x71)
#define REG_FIFO_R_W            (BANK_0 | 0x72)

#define REG_HW_FIX_DISABLE      (BANK_0 | 0x75)

#define REG_FIFO_CFG            (BANK_0 | 0x76)
#define BIT_MULTI_FIFO_CFG              0x01
#define BIT_SINGLE_FIFO_CFG             0x00

#define REG_ACCEL_XOUT_H_SH     (BANK_0 | 0x2D)
#define REG_ACCEL_XOUT_L_SH     (BANK_0 | 0x2E)
#define REG_ACCEL_YOUT_H_SH     (BANK_0 | 0x2F)
#define REG_ACCEL_YOUT_L_SH     (BANK_0 | 0x30)
#define REG_ACCEL_ZOUT_H_SH     (BANK_0 | 0x31)
#define REG_ACCEL_ZOUT_L_SH     (BANK_0 | 0x32)

#define REG_MEM_START_ADDR      (BANK_0 | 0x7C)
#define REG_MEM_R_W             (BANK_0 | 0x7D)
#define REG_MEM_BANK_SEL        (BANK_0 | 0x7E)

/* bank 1 register map */
#define REG_TIMEBASE_CORRECTION_PLL   (BANK_1 | 0x28)
#define REG_TIMEBASE_CORRECTION_RCOSC (BANK_1 | 0x29)
#define REG_SELF_TEST1                (BANK_1 | 0x02)
#define REG_SELF_TEST2                (BANK_1 | 0x03)
#define REG_SELF_TEST3                (BANK_1 | 0x04)
#define REG_SELF_TEST4                (BANK_1 | 0x0E)
#define REG_SELF_TEST5                (BANK_1 | 0x0F)
#define REG_SELF_TEST6                (BANK_1 | 0x10)
#define REG_XA_OFFS_H				  (BANK_1 | 0x14)
#define REG_XA_OFFS_L				  (BANK_1 | 0x15)
#define REG_YA_OFFS_H				  (BANK_1 | 0x17)
#define REG_YA_OFFS_L				  (BANK_1 | 0x18)
#define REG_ZA_OFFS_H				  (BANK_1 | 0x1A)
#define REG_ZA_OFFS_L				  (BANK_1 | 0x1B)
/* bank 2 register map */
#define REG_GYRO_SMPLRT_DIV     (BANK_2 | 0x00)

#define REG_GYRO_CONFIG_1       (BANK_2 | 0x01)
#define SHIFT_GYRO_FS_SEL               1
#define SHIFT_GYRO_DLPCFG               3

#define REG_GYRO_CONFIG_2       (BANK_2 | 0x02)
#define BIT_GYRO_CTEN                   0x38
#define REG_XG_OFFS_USRH				  (BANK_2 | 0x03)
#define REG_XG_OFFS_USRL				  (BANK_2 | 0x04)
#define REG_YG_OFFS_USRH				  (BANK_2 | 0x05)
#define REG_YG_OFFS_USRL				  (BANK_2 | 0x06)
#define REG_ZG_OFFS_USRH				  (BANK_2 | 0x07)
#define REG_ZG_OFFS_USRL				  (BANK_2 | 0x08)

#define REG_ACCEL_SMPLRT_DIV_1  (BANK_2 | 0x10)
#define REG_ACCEL_SMPLRT_DIV_2  (BANK_2 | 0x11)

#define REG_ACCEL_CONFIG        (BANK_2 | 0x14)
#define SHIFT_ACCEL_FS                  1

#define REG_ACCEL_CONFIG_2      (BANK_2 | 0x15)
#define BIT_ACCEL_CTEN                  0x1C

#define REG_PRS_ODR_CONFIG      (BANK_2 | 0x20)
#define REG_PRGM_START_ADDRH    (BANK_2 | 0x50)

#define REG_MOD_CTRL_USR        (BANK_2 | 0x54)
#define BIT_ODR_SYNC                    0x7

/* bank 3 register map */
#define REG_I2C_MST_ODR_CONFIG  (BANK_3 | 0x0)

#define REG_I2C_MST_CTRL        (BANK_3 | 0x01)
#define BIT_I2C_MST_P_NSR               0x10

#define REG_I2C_MST_DELAY_CTRL  (BANK_3 | 0x02)
#define BIT_SLV0_DLY_EN                 0x01
#define BIT_SLV1_DLY_EN                 0x02
#define BIT_SLV2_DLY_EN                 0x04
#define BIT_SLV3_DLY_EN                 0x08

#define REG_I2C_SLV0_ADDR       (BANK_3 | 0x03)
#define REG_I2C_SLV0_REG        (BANK_3 | 0x04)
#define REG_I2C_SLV0_CTRL       (BANK_3 | 0x05)
#define REG_I2C_SLV0_DO         (BANK_3 | 0x06)

#define REG_I2C_SLV1_ADDR       (BANK_3 | 0x07)
#define REG_I2C_SLV1_REG        (BANK_3 | 0x08)
#define REG_I2C_SLV1_CTRL       (BANK_3 | 0x09)
#define REG_I2C_SLV1_DO         (BANK_3 | 0x0A)

#define REG_I2C_SLV2_ADDR       (BANK_3 | 0x0B)
#define REG_I2C_SLV2_REG        (BANK_3 | 0x0C)
#define REG_I2C_SLV2_CTRL       (BANK_3 | 0x0D)
#define REG_I2C_SLV2_DO         (BANK_3 | 0x0E)

#define REG_I2C_SLV3_ADDR       (BANK_3 | 0x0F)
#define REG_I2C_SLV3_REG        (BANK_3 | 0x10)
#define REG_I2C_SLV3_CTRL       (BANK_3 | 0x11)
#define REG_I2C_SLV3_DO         (BANK_3 | 0x12)

#define REG_I2C_SLV4_CTRL       (BANK_3 | 0x15)

#define INV_MPU_BIT_SLV_EN      0x80
#define INV_MPU_BIT_BYTE_SW     0x40
#define INV_MPU_BIT_REG_DIS     0x20
#define INV_MPU_BIT_GRP         0x10
#define INV_MPU_BIT_I2C_READ    0x80

/* register for all banks */
#define REG_BANK_SEL            0x7F

#define COMPASS_SLAVEADDR_AKM_BASE      0x0C
#define COMPASS_SLAVEADDR_AKM           0x0E

#define AK_REG_WIA2				0x01
#define AK_REG_ST1				0x10
#define AK_REG_HXL				0x11
#define AK_REG_HXH				0x12
#define AK_REG_HYL				0x13
#define AK_REG_HYH				0x14
#define AK_REG_HZL				0x15
#define AK_REG_HZH				0x16
#define AK_REG_ST2				0x18
#define AK_REG_CNTL2			0x31
#define AK_REG_CNTL3			0x32
#define AK_REG_TS1				0x33
#define AK_REG_TS2				0x34
    
#define BIT(x) ( 1 << x )              

#define ENABLE  1
#define DISABLE 0
    
// interrupt configurations related to HW register
#define FSYNC_INT   BIT(7)
#define MOTION_INT  BIT(3)
#define PLL_INT     BIT(2)
#define DMP_INT     BIT(1)
#define I2C_INT     BIT(0)

#define CHIP_AWAKE          (0x01)
#define CHIP_LP_ENABLE      (0x02)

#ifndef min
#define min(x,y)    (((x)<(y))?(x):(y))
#endif

#ifndef max
#define max(x,y)    (((x)>(y))?(x):(y))
#endif

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _INV_ICM20948_DEFINES_H_ */

