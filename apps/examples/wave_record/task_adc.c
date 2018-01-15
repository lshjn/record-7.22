#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_adc.h"
#include "task_monitor.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
 
pthread_mutex_t g_AdcMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_AdcConVar	= PTHREAD_COND_INITIALIZER;

struct sensor_msg	SensorDate;
struct adc_msg		adcdata;
struct i2c_msg		i2cdata;

static bool  g_adc_started;
static	int  fd_adc;

/****************************************************************************/
/******************************************************************************************************************************
函数名称：void	EnAdcSampl(pthread_cond_t *cond,pthread_mutex_t *mutex)
功    能：使能adc采样
输入参数：锁、条件变量
输入参数：采样状态
编写时间：2017.09.29
编 写 人：liushuhe
*******************************************************************************************************************************/
void	EnAdcSampl(pthread_cond_t *cond,pthread_mutex_t *mutex)
{

	//启动一次adc采集
	pthread_mutex_lock(mutex);
	SensorDate.startadc = true;
	pthread_cond_signal(cond);
	pthread_mutex_unlock(mutex);
}


/******************************************************************************************************************************
函数名称：int	StartADCsampl(int fd)
功    能：启动一次adc采样
输入参数：adc文件描述符
输入参数：采样状态
编写时间：2017.09.28
编 写 人：liushuhe
*******************************************************************************************************************************/

int	StartAdcSampl(int fd)
{
	int	ret;
#ifdef CONFIG_EXAMPLES_ADC_SWTRIG
	//启动adc转换
	ret = ioctl(fd, ANIOC_TRIGGER, 0);
	if (ret < 0)
	{
		int errcode = errno;
		printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
	}
#endif
	return ret;
}

/******************************************************************************************************************************
函数名称：int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
功    能：读取采样值
输入参数：T_adcsample结构指针
输入参数：采样状态
编写时间：2017.09.28
编 写 人：liushuhe
*******************************************************************************************************************************/
int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
{
	struct adc_msg_s sample[CONFIG_EXAMPLES_ADC_GROUPSIZE];

	size_t readsize;
	ssize_t nbytes;
	int errval = 0;
	int adc_num = 0;
	
	//读取采样数据 
	readsize = CONFIG_EXAMPLES_ADC_GROUPSIZE * sizeof(struct adc_msg_s);

	nbytes = read(fd, sample, readsize);
	//处理是否异常
	if (nbytes < 0)
	{
		errval = errno;
		if (errval != EINTR)
		{
			printf("adc_main: read %s failed: %d\n",CONFIG_EXAMPLES_ADC_DEVPATH, errval);
		}
		printf("adc_main: Interrupted read...\n");
	}
	else if (nbytes == 0)
	{
		printf("adc_main: No data read, Ignoring\n");
	}
	//读取到的采样数据
	else
	{
		int nsamples = nbytes / sizeof(struct adc_msg_s);
		if (nsamples * sizeof(struct adc_msg_s) != nbytes)
		{
			printf("adc_main: read size=%ld is not a multiple of sample size=%d, Ignoring\n",(long)nbytes, sizeof(struct adc_msg_s));
		}
		else
		{
			for (adc_num = 0; adc_num < nsamples; adc_num++)
			{
				Sensor_data->sample_tempdata[adc_num].am_data	=	sample[adc_num].am_data;
			}
		}
	}

	return	(int)nbytes;
	
}
/******************************************************************************************************************************
函数名称：int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
功    能：读取采样值
输入参数：T_adcsample结构指针
输入参数：采样状态
编写时间：2017.09.28
编 写 人：liushuhe
*******************************************************************************************************************************/
void CalcSampleData(struct sensor_msg *Sensor_data)
{
	//2534/2.042 = adc/vcc1, 12/2.042 = VCC/vcc1
	float vcc_temp = (Sensor_data->sample_tempdata[0].am_data*2.042)/2534;
	vcc_temp	  = (vcc_temp*12.18)/2.042;
	vcc_temp	  = vcc_temp*10;
 	Sensor_data->adcmsg->VCC		=	vcc_temp;	
 	Sensor_data->adcmsg->O2		=	Sensor_data->sample_tempdata[1].am_data;
 	Sensor_data->adcmsg->NH3		=	Sensor_data->sample_tempdata[2].am_data;
 	Sensor_data->adcmsg->H2S		=	Sensor_data->sample_tempdata[3].am_data;
 	Sensor_data->adcmsg->CO		=	Sensor_data->sample_tempdata[4].am_data;

	//0.496/611 = vcc/adc,0.496/4ma = vcc/y ma, 20 ma/5m = (y-4)/z m
	if(Sensor_data->sample_tempdata[5].am_data != 0)
	{
		float Water_temp = (Sensor_data->sample_tempdata[5].am_data*0.495)/611;
		Water_temp = (Water_temp*4)/0.496;
		Water_temp = ((Water_temp-4)*5)/20;
		Water_temp = Water_temp*10;
		Sensor_data->adcmsg->Water_high =	Water_temp;
	}
	else
	{
		Sensor_data->adcmsg->Water_high =	0;
	}

 	Sensor_data->adcmsg->tempretrue		=	Sensor_data->sample_tempdata[6].am_data;


	
	//ok
	Sensor_data->sampleisok = 1;

}



/****************************************************************************
 * slave_adc
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_adc(int argc, char *argv[])
{
	int ret;

	g_adc_started = true;

	fd_adc = open(CONFIG_EXAMPLES_ADC_DEVPATH, O_RDONLY);
	if (fd_adc < 0)
	{
		printf("slave_adc: open %s failed: %d\n", CONFIG_EXAMPLES_ADC_DEVPATH, errno);
		goto errout;
	}
	
	SensorDate.adcmsg	= &adcdata;
	SensorDate.i2cmsg	= &i2cdata;
	
	while(1)
	{
		pthread_mutex_lock(&g_AdcMutex);
		while(SensorDate.startadc != true)
		{
			pthread_cond_wait(&g_AdcConVar, &g_AdcMutex);
		}
		
		
		ret = StartAdcSampl(fd_adc);
		//successful
		if(ret == 0)
		{
			ret = ReadAdcData(fd_adc,&SensorDate);
			if(ret)
			{
				CalcSampleData(&SensorDate);
				//test
				SensorDate.i2cmsg->humidity    = 74;
				SensorDate.i2cmsg->tempretrue  = 156;
			}
		}
		
		
		SensorDate.startadc = false;
		pthread_mutex_unlock(&g_AdcMutex);

	}
/*
errout_with_dev:
  close(fd_adc);
 */
 errout:
  g_adc_started = false;

  printf("adc: Terminating\n");

  return EXIT_FAILURE;
}



