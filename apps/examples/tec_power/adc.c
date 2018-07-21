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

#include "adc.h"
#include "task_monitor.h"
#include "pid.h"
#include "task_modbus.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct sensor_msg	SensorDate;

/******************************************************************************************************************************
函数名称：int	StartADCsampl(int fd)
功    能：启动一次adc采样
输入参数：adc文件描述符
输入参数：采样状态
编写时间：2018.06.28
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
编写时间：2018.06.28
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
函数名称：float	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
功    能：读取采样值
输入参数：T_adcsample结构指针
输入参数：采样状态
编写时间：2018.06.28
编 写 人：liushuhe
*******************************************************************************************************************************/
float CalcSampleData(struct sensor_msg *Sensor_data)
{
	float    vout = 0;
	float    AO4485_A = 0;
	int32_t  adc_new = 0;  
	
#if 1
    //calc 需要用1.5v校准，我是用3.3--4096，比例计算得到1.5---1861
    //需要实际的校准
    #define  calc   1861
	
    //TP2:1.5v      calc                   
	//vout      	adc_new      
 
	adc_new = Sensor_data->sample_tempdata[0].am_data;
	printf("------------------------\n");
	printf("adc_new = %d\n",adc_new);
	
    vout = ((float)(1.5*adc_new))/calc;
	//printf("vout = %.2f\n",vout);
#endif	
	//同相偏置放大
	//传递函数Vout=25Vin+0.8
	
	float Vin  = (vout - 0.8)/25;
	printf("Vin = %.2f\n",Vin);

	//AO4485_A 当前电流
    //50mR = 0.05R
    
	AO4485_A = Vin/0.05;
	printf("------------------------\n");	
	printf("I_CUR = %.2f\n",AO4485_A);
	printf("I_MAX = %.2f\n",pid.I_MAX);
	printf("------------------------\n");	
	return AO4485_A;
}


float read_DC_I(void)
{
	int  fd_adc = -1;
	int ret;
	
	fd_adc = open(CONFIG_EXAMPLES_ADC_DEVPATH, O_RDONLY);
	if (fd_adc < 0)
	{
		printf("slave_adc: open %s failed: %d\n", CONFIG_EXAMPLES_ADC_DEVPATH, errno);
		return -1;
	}

	
	ret = StartAdcSampl(fd_adc);
	
	//successful
	if(ret == 0)
	{
		ret = ReadAdcData(fd_adc,&SensorDate);
		
		if(ret)
		{
			pid.I_CUR = CalcSampleData(&SensorDate);
			//设置modbus数据结构
			g_modbus.reginput[1] = pid.I_CUR;
		}
	}
	else
	{
		printf("StartAdcSampl error!!!!\n");
		return -1;
	}
	
	close(fd_adc);
	return pid.I_CUR;
}





