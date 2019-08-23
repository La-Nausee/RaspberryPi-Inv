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
	#include "icm20948.h"
}

#define ENABLE_INTERRUPUT	0

#define ICM_INT_EVENT     0x01
#define NEW_FILE_EVENT    0x03
#define FILE_CLOSE_EVENT  0x04
#define LOG_START_EVENT   0x05
#define LOG_STOP_EVENT    0x06
#define APP_EXIT_EVENT    0x07

using namespace std;

std::queue<char> icm_queue;
ofstream icm_logfile;
string icm_filename;

#if ENABLE_INTERRUPUT
void icm_handler(void)
{
	icm_queue.push(ICM_INT_EVENT);
	//printf("data ready 1\r\n");
}
#endif

void *icm_log_thread(void *threadid)
{
	long tid;
	float ax,ay,az,gx,gy,gz,mx,my,mz,temp;
	stringstream out;
	struct timeval tp;
	tid = (long)threadid;
	
	ax=ay=az=gx=gy=gz=mx=my=mz=temp=0;
	
	configMPU();
	configAK09916();
	
	while(1)
	{
#if !ENABLE_INTERRUPUT
		if(checkDataReady())
		{
			icm_queue.push(ICM_INT_EVENT);
			//printf("data ready 1\r\n");
		}
#endif
		if(!icm_queue.empty())
		{
			switch(icm_queue.front())
			{
			case ICM_INT_EVENT:
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
			
			if(icm_logfile.is_open())
			{
				icm_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<",";
				icm_logfile<<gx<<","<<gy<<","<<gz<<",";
				icm_logfile<<ax<<","<<ay<<","<<az<<",";
				icm_logfile<<mx<<","<<my<<","<<mz<<",";
				icm_logfile<<temp;
				icm_logfile<<endl;
			}
			break;
			case NEW_FILE_EVENT:
			printf("\r\n[INFO] create new ICM log file.\r\n");
			out.clear();
			if(icm_logfile.is_open())
				icm_logfile.close();
			icm_logfile.open(icm_filename.c_str());
			assert(!icm_logfile.fail());
			//write headers
			if(icm_logfile.is_open())
			{
				icm_logfile<<"seconds.milliseconds,";
				icm_logfile<<"gx(dps),gy,gz,";
				icm_logfile<<"ax(g),ay,az,";
				icm_logfile<<"mx(uT),my,mz,";
				icm_logfile<<"temperature(degree),";
				icm_logfile<<endl;
			}	
			break;
			case FILE_CLOSE_EVENT:
			if(icm_logfile.is_open())
			{
				//icm_logfile<<out.str();
				icm_logfile.close();
				out.clear();
			}
			printf("\r\n[INFO] close ICM log file.\r\n");
			break;
			case LOG_START_EVENT:
			printf("\r\n[INFO] ICM log start.\r\n");	
			break;
			case LOG_STOP_EVENT:
			printf("\r\n[INFO] ICM log stop.\r\n");	
			break;
			case APP_EXIT_EVENT:
			if(icm_logfile.is_open())
			{
				//icm_logfile<<out.str();
				icm_logfile.close();
				out.clear();
			}
			printf("\r\n[INFO] exit ICM thread.\r\n");
			pthread_exit(NULL);
			break;
			default:

			break;
			}
			icm_queue.pop();
		}
	}

}		


int main()
{
	char key = 0;
	pthread_t icm_thread;
	pthread_attr_t tattr;
	sched_param param;
	
	spi_init();

#if ENABLE_INTERRUPUT
	if(wiringPiISR(ICM_INTERRUPT_PIN,INT_EDGE_RISING,&icm_handler) < 0)
	{
		printf("[ERROR] Unable to setup ICM ISR.\r\n");
	}
#endif
	
	pthread_attr_init(&tattr);
	pthread_attr_getschedparam (&tattr, &param);
	param.sched_priority = 99;
	pthread_attr_setschedparam (&tattr, &param);
	pthread_create(&icm_thread,&tattr,icm_log_thread,(void *)NULL);

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
				icm_filename = "icm_";
				icm_filename.append(str);
				icm_filename.append(".txt");
				icm_queue.push(NEW_FILE_EVENT);
			}
			else if(key == '2')
			{
				icm_queue.push(FILE_CLOSE_EVENT);
			}
			else if(key == '3')
			{
				icm_queue.push(LOG_STOP_EVENT);
			}
			else if(key == '4')
			{
				icm_queue.push(LOG_START_EVENT);
			}
			else if(key == '5')
			{
				icm_queue.push(APP_EXIT_EVENT);
				pthread_join(icm_thread, NULL);
				exit(0);
			}
		}
	
	return 0;
}
