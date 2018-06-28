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
#include "pid.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct sensor_msg	SensorDate;

/******************************************************************************************************************************
�������ƣ�int	StartADCsampl(int fd)
��    �ܣ�����һ��adc����
���������adc�ļ�������
�������������״̬
��дʱ�䣺2018.06.28
�� д �ˣ�liushuhe
*******************************************************************************************************************************/

int	StartAdcSampl(int fd)
{
	int	ret;
#ifdef CONFIG_EXAMPLES_ADC_SWTRIG
	//����adcת��
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
�������ƣ�int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
��    �ܣ���ȡ����ֵ
���������T_adcsample�ṹָ��
�������������״̬
��дʱ�䣺2018.06.28
�� д �ˣ�liushuhe
*******************************************************************************************************************************/
int	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
{
	struct adc_msg_s sample[CONFIG_EXAMPLES_ADC_GROUPSIZE];

	size_t readsize;
	ssize_t nbytes;
	int errval = 0;
	int adc_num = 0;
	
	//��ȡ�������� 
	readsize = CONFIG_EXAMPLES_ADC_GROUPSIZE * sizeof(struct adc_msg_s);

	nbytes = read(fd, sample, readsize);
	//�����Ƿ��쳣
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
	//��ȡ���Ĳ�������
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
�������ƣ�float	ReadAdcData(int fd,struct sensor_msg *Sensor_data)
��    �ܣ���ȡ����ֵ
���������T_adcsample�ṹָ��
�������������״̬
��дʱ�䣺2018.06.28
�� д �ˣ�liushuhe
*******************************************************************************************************************************/
float CalcSampleData(struct sensor_msg *Sensor_data)
{
	//ͬ��ƫ�÷Ŵ�
	//���ݺ���Vout=25Vin+0.8
	int32_t Vout = Sensor_data->sample_tempdata[0].am_data;
	float Vin  = (Vout - 0.8)/25;
	return Vin;
}


float read_DC_I(void)
{
	static	int  fd_adc;
	static	int  adc_status = false;
	
	int ret;
if(!adc_status)
{
	fd_adc = open(CONFIG_EXAMPLES_ADC_DEVPATH, O_RDONLY);
	if (fd_adc < 0)
	{
		printf("slave_adc: open %s failed: %d\n", CONFIG_EXAMPLES_ADC_DEVPATH, errno);
		return -1;
	}
	adc_status = true;
}

	
	ret = StartAdcSampl(fd_adc);
	//successful
	if(ret == 0)
	{
		ret = ReadAdcData(fd_adc,&SensorDate);
		if(ret)
		{
			pid.DC_I_CUR_ADC = CalcSampleData(&SensorDate);
			//����modbus���ݽṹ
		}
	}
	else
	{
		return -1;
	}
	
	return pid.DC_I_CUR_ADC;
}

/****************************************************************************
 * master_adc
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_adc(int argc, char *argv[])
{
	while(1)
	{
		sleep(1);
	}

  return 0;;
}



