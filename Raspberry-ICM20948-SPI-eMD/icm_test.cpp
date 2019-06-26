#include <fstream>
#include <iostream>
#include <cassert>
#include <cstring>
#include <queue>
#include <pthread.h>

#include <stdio.h>^M
#include <string.h>^M
#include <stdlib.h>
#include <errno.h>^M
#include <unistd.h>^M
#include <wiringPi.h>
#include <sys/time.h>
#include <assert.h>
#include "spi_if.h"

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

#define ICM_INT_PIN    24
#define ICM_INT_EVENT  0x01
#define ODR_NONE	   0


#define USE_RAW_ACC 0
#define USE_RAW_GYR 0
#define USE_GRV     0
#define USE_CAL_ACC 1
#define USE_CAL_GYR 1
#define USE_CAL_MAG 1
#define USE_UCAL_GYR 0
#define USE_UCAL_MAG 0
#define USE_RV      0    /* requires COMPASS*/
#define USE_GEORV   0    /* requires COMPASS*/
#define USE_ORI     0    /* requires COMPASS*/
#define USE_STEPC   0
#define USE_STEPD   0
#define USE_SMD     0
#define USE_BAC     0
#define USE_TILT    0
#define USE_PICKUP  0
#define USE_GRAVITY 0
#define USE_LINACC  0
#define USE_B2S     0

using namespace std;

static const struct {
	uint8_t  type;
	uint32_t period_us;
} sensor_list[] = {
#if USE_RAW_ACC
	{ INV_SENSOR_TYPE_RAW_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_RAW_GYR
	{ INV_SENSOR_TYPE_RAW_GYROSCOPE,     50000 /* 20 Hz */ },
#endif
#if USE_CAL_ACC
	{ INV_SENSOR_TYPE_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_GYR
	{ INV_SENSOR_TYPE_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_MAG
	{ INV_SENSOR_TYPE_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_GYR
	{ INV_SENSOR_TYPE_UNCAL_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_MAG
	{ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_GRV
	{ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_RV
	{ INV_SENSOR_TYPE_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_GEORV
	{ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_ORI
	{ INV_SENSOR_TYPE_ORIENTATION, 50000 /* 20 Hz */ },
#endif
#if USE_STEPC
	{ INV_SENSOR_TYPE_STEP_COUNTER, ODR_NONE },
#endif
#if USE_STEPD
	{ INV_SENSOR_TYPE_STEP_DETECTOR, ODR_NONE},
#endif
#if USE_SMD
	{ INV_SENSOR_TYPE_SMD, ODR_NONE},
#endif
#if USE_BAC
	{ INV_SENSOR_TYPE_BAC, ODR_NONE},
#endif
#if USE_TILT
	{ INV_SENSOR_TYPE_TILT_DETECTOR, ODR_NONE},
#endif
#if USE_PICKUP
	{ INV_SENSOR_TYPE_PICK_UP_GESTURE, ODR_NONE},
#endif
#if USE_GRA
	{ INV_SENSOR_TYPE_GRAVITY, 50000 /* 20 Hz */},
#endif
#if USE_LINACC
	{ INV_SENSOR_TYPE_LINEAR_ACCELERATION, 50000 /* 20 Hz */},
#endif
#if USE_B2S
	{ INV_SENSOR_TYPE_B2S, ODR_NONE},
#endif
};

static void sensor_event_cb(const inv_sensor_event_t * event, void * arg);

static const uint8_t dmp3_image[] = {
	#include "Invn/Images/icm20948_img.dmp3a.h"
};

static inv_device_icm20948_t device_icm20948;

static inv_device_t * device;

static const inv_sensor_listener_t sensor_listener = {
	sensor_event_cb,
	0
};

std::queue<char> m_queue;

void icm_handler(void)
{
	m_queue.push(ICM_INT_EVENT);
}

void icm_setup()
{
	int rc = 0;
	uint8_t i = 0;
	uint8_t whoami = 0xff;
	uint64_t available_sensor_mask = 0;

	/*Setup ICM20948*/
	if(wiringPiISR(ICM_INT_PIN,INT_EDGE_RISING,&icm_handler) < 0)
	{
		printf("[ERROR] Unable to setup ISR.\r\n");
	}

	inv_host_serif_open(icm_get_serif_instance_spi());

	inv_device_icm20948_init(&device_icm20948, icm_get_serif_instance_spi(), &sensor_listener, dmp3_image, sizeof(dmp3_image));

	device = inv_device_icm20948_get_base(&device_icm20948);

	inv_device_whoami(device, &whoami);

	printf("ICM WHOAMI = 0x%02X\r\n", whoami);

	cout<<"Setting-up ICM device"<<endl;

	inv_device_setup(device);

	inv_device_load(device,NULL,dmp3_image,sizeof(dmp3_image),true,NULL);

	//check sensor availibility
	for(i = 0; i< sizeof(sensor_list)/sizeof(sensor_list[0]); ++i){
		const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
		printf("Ping %s %s\r\n", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK" : "KO");
		if(rc == 0)
		{
			available_sensor_mask |= (1<<sensor_list[i].type);
		}
	}
	
	//start all available sensors from sensor list
	for(i = 0; i< sizeof(sensor_list)/sizeof(sensor_list[0]); ++i){
		//if(available_sensor_mask & (1<<sensor_list[i].type)){
			printf("Starting %s @ %u us\r\n", inv_sensor_2str(sensor_list[i].type), sensor_list[i].period_us);
			rc = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
			rc += inv_device_start_sensor(device, sensor_list[i].type);
		//}
	}
}

int main()
{
	int rc = 0;

	spi_init();

	icm_setup();

	while(1)
	{
		if(!m_queue.empty())
		{
			switch(m_queue.front())
			{
			case ICM_INT_EVENT:
			rc = inv_device_poll(device);
			//printf("rc = %d\r\n",rc);
			if(rc >= 0)
			{
				m_queue.pop();
			}

			break;
			default:break;
			}
			//m_queue.pop();
		}
	}
 
	return 0;
}



static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
	(void)arg;
	//printf("ICM INT.\r\n");
	
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {

		switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
			printf("data event %s (lsb): %llu %d %d %d\r\n", inv_sensor_str(event->sensor),
					event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);
			break;
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			printf("data event %s (lsb): %llu %d %d %d\r\n", inv_sensor_str(event->sensor),
					event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);
			break;
		
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
			printf("data event %s (mg): %d %d %d\r\n", inv_sensor_str(event->sensor),
					(int)(event->data.acc.vect[0]*1000),
					(int)(event->data.acc.vect[1]*1000),
					(int)(event->data.acc.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			printf("data event %s (mdps): %d %d %d\r\n", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_MAGNETOMETER:
			printf("data event %s (nT): %d %d %d\r\n", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			printf("data event %s (mdps): %d %d %d %d %d %d\r\n", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			printf("data event %s (nT): %d %d %d %d %d %d\r\n", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
			printf("data event %s (e-3): %d %d %d %d \r\n", inv_sensor_str(event->sensor),
					(int)(event->data.quaternion.quat[0]*1000),
					(int)(event->data.quaternion.quat[1]*1000),
					(int)(event->data.quaternion.quat[2]*1000),
					(int)(event->data.quaternion.quat[3]*1000));
			break;
		case INV_SENSOR_TYPE_ORIENTATION:
			printf("data event %s (e-3): %d %d %d %d \r\n", inv_sensor_str(event->sensor),
					(int)(event->data.orientation.x*1000),
					(int)(event->data.orientation.y*1000),
					(int)(event->data.orientation.z*1000));
			break;
		case INV_SENSOR_TYPE_BAC:
			//printf("data event %s : %d %s\r\n", inv_sensor_str(event->sensor),
					//event->data.bac.event, activityName(event->data.bac.event));
			break;
		case INV_SENSOR_TYPE_STEP_COUNTER:
			printf("data event %s : %lu\r\n", inv_sensor_str(event->sensor),
					(unsigned long)event->data.step.count);
			break;
		case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		
		default:
			printf("data event %s : ...\r\n", inv_sensor_str(event->sensor));
			break;
		}
	}

	return;
}
