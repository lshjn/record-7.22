#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>

#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_bluetooth.h"
#include "task_monitor.h"
#include "task_gprs.h"
#include "task_adc.h"

/*****************************************************************************************************************************
 * Private Data
 ****************************************************************************************************************************/


struct  	TimeStruct  DisLocalTime;
struct		tm 			*tmp;
struct 		rtc_time	rtctime;
time_t		timelocal;

/****************************************************************************
 * getSystime
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
void  getSystime(void)
{
	time(&timelocal);	
	tmp = localtime(&timelocal);  //获取本地时间


	DisLocalTime.Year		=	1900+tmp->tm_year;
	DisLocalTime.Month		=	tmp->tm_mon;
	DisLocalTime.Day		=	tmp->tm_mday;
	DisLocalTime.Hour		=	tmp->tm_hour;
	DisLocalTime.Minute	=	tmp->tm_min;
	DisLocalTime.Second	=	tmp->tm_sec;
}
/****************************************************************************
 * setSystime
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int setSystime(struct rtc_time *rtc)
{
	//struct rtc_time rtc;  
	struct tm _tm;  
	struct timeval tv;  
	time_t timep;  

	_tm.tm_sec 	= rtc->tm_sec;  
	_tm.tm_min 	= rtc->tm_min;  
	_tm.tm_hour = rtc->tm_hour;  
	_tm.tm_mday = rtc->tm_mday;  
	_tm.tm_mon 	= rtc->tm_mon;  
	_tm.tm_year = rtc->tm_year - 1900;  

	timep = mktime(&_tm);  
	tv.tv_sec = timep;  
	tv.tv_usec = 0;  
	if(settimeofday (&tv, (struct timezone *) 0) < 0)  
	{  
		printf("Set system datatime error!/n");  
		return -1;  
	}  
	return 0;  

}
/****************************************************************************
 * timeint_upload_process
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
void timeint_upload_process(int fd,struct gprs_data	*gprsdata,struct adc_msg *adcdata)
{
	int ret;
	ret = gprs_rst(fd,gprsdata);
	if(ret == SUCCESS)
	{
		ret = gprs_timeint_upload(fd,gprsdata,adcdata);
		if(ret == SUCCESS)
		{
			printf("timeint upload ok\n");
			gprsdata->process_state = SUCCESS;
		}
		else
		{
			//ask close lock
			printf("timeint upload fail\n");
			gprsdata->process_state = FAIL;
		}
	}
	else
	{
		//ask close lock
		printf("gprs connect fail\n");
	}
}

/****************************************************************************
 * master_monitor
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
int master_monitor(int argc, char *argv[])
{
	int fd_gprs_copy;
	fd_gprs_copy = open(CONFIG_EXAMPLES_GPRS_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_gprs_copy < 0)
	{
		int errcode = errno;
		printf("gprs: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_GPRS_DEVPATH, errcode);
	}

    //power on
	boardctl(BOARDIOC_BLUEDEV_POWER_ENABLE, 0);			//power on
	usleep(500*1000L);
	//gprs
	boardctl(BOARDIOC_GPRS_PWRON, 0);
	usleep(500*1000);
	//gps power
	boardctl(BOARDIOC_GPS_PWRON, 0);
	usleep(500*1000);
	//433 power
	boardctl(BOARDIOC_433_PWRON, 0);
	usleep(500*1000);
	//433 power
	boardctl(BOARDIOC_4G_PWRON, 0);
	usleep(500*1000);
	sleep(5);                                     //sleep 100ms
	while(1)
	{
		//timeint_upload_process(fd_gprs_copy,&GprsData,&AdcData);
		sleep(5);                                     //sleep 100ms
	/*************************************************************************/
		//启动一次adc采集
		//EnAdcSampl(&g_AdcConVar,&g_AdcMutex);
		//printf("test float= %09.2f\n",24.3587);
		if(1==SensorDate.sampleisok)
		{
			SensorDate.sampleisok	= 0;
			/*************************************************************************/
			printf("%s: value: %d v\n", "sample_tempdata[0]", SensorDate.sample_tempdata[0].am_data);
			printf("%s: value: %d v\n", "sample_tempdata[1]", SensorDate.sample_tempdata[1].am_data);
			printf("%s: value: %d v\n", "sample_tempdata[2]", SensorDate.sample_tempdata[2].am_data);
			printf("%s: value: %d v\n", "sample_tempdata[3]", SensorDate.sample_tempdata[3].am_data);
			printf("%s: value: %d v\n", "sample_tempdata[4]", SensorDate.sample_tempdata[4].am_data);
			printf("%s: value: %d v\n", "sample_tempdata[5]", SensorDate.sample_tempdata[5].am_data);
			printf("%s: value: %d v\n", "sample_tempdata[6]", SensorDate.sample_tempdata[6].am_data);
			/*
			//VCC
			printf("%s: value: %d v\n", "SensorDate.adcmsg.VCC", SensorDate.adcmsg->VCC);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.O2", SensorDate.adcmsg->O2);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.NH3", SensorDate.adcmsg->NH3);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.H2S", SensorDate.adcmsg->H2S);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.CO", SensorDate.adcmsg->CO);
			printf("%s: value: %d m\n", "SensorDate.adcmsg.Water_high", SensorDate.adcmsg->Water_high);
			*/
			/*************************************************************************/
			//紧急上报
			//定时采集 8:00 | 18:00
		}
		
	}

 return EXIT_FAILURE;

}


