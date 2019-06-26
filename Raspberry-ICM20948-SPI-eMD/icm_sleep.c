#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <wiringPi.h>

void inv_icm20948_sleep_us(int us)
{
	//usleep(us);
	delayMicroseconds(us);
}

uint64_t inv_icm20948_get_time_us(void)
{
	/*
	struct timespec tms;
	int64_t micros;

	clock_gettime(CLOCK_REALTIME,&tms);

	micros = tms.tv_sec * 1000000;
	micros += tms.tv_nsec/1000;

	if(tms.tv_nsec % 1000 >= 500)
		micros++;

	return micros;*/
	return (uint64_t) micros();
}

uint64_t inv_icm_get_dataready_interrupt_time_us(void)
{

	return 0;

}

uint64_t inv_ak0991x_get_time_us(void)
{
	return 0;
}
