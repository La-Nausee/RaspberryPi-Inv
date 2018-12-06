#include <fstream>
#include <iostream>
#include <sstream>
#include <cassert>
#include <cstring>
#include <queue>
#include <pthread.h>
#include <sched.h>
#include <wiringPi.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

extern "C" {
   #include "spi_if.h"
	#include "mpu9250.h"
}

#define ENABLE_INTERRUPUT	0

#define MPU_INT_EVENT     0x01
#define NEW_FILE_EVENT    0x03
#define FILE_CLOSE_EVENT  0x04
#define LOG_START_EVENT   0x05
#define LOG_STOP_EVENT    0x06
#define APP_EXIT_EVENT    0x07

using namespace std;

std::queue<char> mpu_queue;
ofstream mpu_logfile;
string mpu_filename;

#if ENABLE_INTERRUPUT
void mpu_handler(void)
{
	mpu_queue.push(MPU_INT_EVENT);
}
#endif

void *mpu_log_thread(void *threadid)
{
	long tid;
	float ax,ay,az,gx,gy,gz,mx,my,mz,temp;
	stringstream out;
	struct timeval tp;
	tid = (long)threadid;
	
	ax=ay=az=gx=gy=gz=mx=my=mz=temp=0;
	
	configMPU9250();
	configAK8963();
	
	while(1)
	{
#if ENABLE_INTERRUPUT == 0
		if(checkDataReady())
		{
			mpu_queue.push(MPU_INT_EVENT);
			//printf("data ready 1\r\n");
		}
#endif
		if(!mpu_queue.empty())
		{
			switch(mpu_queue.front())
			{
			case MPU_INT_EVENT:
			gettimeofday(&tp,NULL);
			readSensor(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz,&temp);
			
			/*
			out<<tp.tv_sec<<"."<<tp.tv_usec<<",";
			out<<gx<<","<<gy<<","<<gz<<",";
			out<<ax<<","<<ay<<","<<az<<",";
			out<<mx<<","<<my<<","<<mz<<",";
			out<<temp;
			out<<endl;
			*/
			
			if(mpu_logfile.is_open())
			{
				mpu_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<",";
				mpu_logfile<<gx<<","<<gy<<","<<gz<<",";
				mpu_logfile<<ax<<","<<ay<<","<<az<<",";
				mpu_logfile<<mx<<","<<my<<","<<mz<<",";
				mpu_logfile<<temp;
				mpu_logfile<<endl;
			}
			break;
			case NEW_FILE_EVENT:
			printf("\r\n[INFO] create new MPU log file.\r\n");
			out.clear();
			if(mpu_logfile.is_open())
				mpu_logfile.close();
			mpu_logfile.open(mpu_filename.c_str());
			assert(!mpu_logfile.fail());
			//write headers
			if(mpu_logfile.is_open())
			{
				mpu_logfile<<"seconds.milliseconds,";
				mpu_logfile<<"gx(dps),gy,gz,";
				mpu_logfile<<"ax(g),ay,az,";
				mpu_logfile<<"mx(uT),my,mz,";
				mpu_logfile<<"temperature(degree),";
				mpu_logfile<<endl;
			}	
			break;
			case FILE_CLOSE_EVENT:
			if(mpu_logfile.is_open())
			{
				//mpu_logfile<<out.str();
				mpu_logfile.close();
				out.clear();
			}
			printf("\r\n[INFO] close ICM log file.\r\n");
			break;
			case LOG_START_EVENT:
			printf("\r\n[INFO] MPU log start.\r\n");	
			break;
			case LOG_STOP_EVENT:
			printf("\r\n[INFO] MPU log stop.\r\n");	
			break;
			case APP_EXIT_EVENT:
			if(mpu_logfile.is_open())
			{
				//mpu_logfile<<out.str();
				mpu_logfile.close();
				out.clear();
			}
			printf("\r\n[INFO] exit MPU thread.\r\n");
			pthread_exit(NULL);
			break;
			default:

			break;
			}
			mpu_queue.pop();
		}
	}

}		


int main()
{
	char key = 0;
	pthread_t mpu_thread;
	pthread_attr_t tattr;
	sched_param param;
	
	spi_init();

#if ENABLE_INTERRUPUT
	if(wiringPiISR(MPU_INTERRUPT_PIN,INT_EDGE_RISING,&mpu_handler) < 0)
	{
		printf("[ERROR] Unable to setup MPU ISR.\r\n");
	}
#endif
	
	pthread_attr_init(&tattr);
	pthread_attr_getschedparam (&tattr, &param);
	param.sched_priority = 99;
	pthread_attr_setschedparam (&tattr, &param);
	pthread_create(&mpu_thread,&tattr,mpu_log_thread,(void *)NULL);

	while(1)
		{
			cout<<endl;
			cout<<"1: create new file for logging data"<<endl;
			cout<<"2: close and save data file"<<endl;
			cout<<"3: pause data logging"<<endl;
			cout<<"4: start data logging"<<endl;
			cout<<"5: save data file and exit the application"<<endl;
			cout<<"Input your selection:";
			key = getchar();
			if(key == '1')
			{	
				string str;
				cout<<"Input the new filename:";
				//getline(cin,filename);
				cin>>str;
				mpu_filename = "mpu_";
				mpu_filename.append(str);
				mpu_filename.append(".txt");
				mpu_queue.push(NEW_FILE_EVENT);
			}
			else if(key == '2')
			{
				mpu_queue.push(FILE_CLOSE_EVENT);
			}
			else if(key == '3')
			{
				mpu_queue.push(LOG_START_EVENT);
			}
			else if(key == '4')
			{
				mpu_queue.push(LOG_STOP_EVENT);
			}
			else if(key == '5')
			{
				mpu_queue.push(APP_EXIT_EVENT);
				pthread_join(mpu_thread, NULL);
				exit(0);
			}
		}
	
	return 0;
}
