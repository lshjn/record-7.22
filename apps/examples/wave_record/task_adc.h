#ifndef _TASK_ADC_H
#define _TASK_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <nuttx/analog/adc.h>


/****************************************************************************
 * Private Data
 ****************************************************************************/
extern	pthread_mutex_t g_AdcMutex;
extern	pthread_cond_t  g_AdcConVar;

struct adc_msg
{
	int	 VCC;
	int	 O2;
	int	 NH3;
	int	 H2S;
	int	 CO;
	int	 Water_high;

	int tempretrue;
	int humidity;
	
};

struct i2c_msg
{
	int	 tempretrue;
	int	 humidity;
};


struct sensor_msg {
	bool	startadc;
	char	sampleisok;
	struct adc_msg_s	sample_tempdata[7];
	struct adc_msg		*adcmsg;
	struct i2c_msg		*i2cmsg;
};

/****************************************************************************
 * Private struct
 ****************************************************************************/

extern	struct sensor_msg		SensorDate;
extern	struct adc_msg		adcdata;
extern	struct i2c_msg		i2cdata;

/****************************************************************************
 * Private function
 ****************************************************************************/
 void	EnAdcSampl(pthread_cond_t *cond,pthread_mutex_t *mutex);
int	StartAdcSampl(int fd);
int	ReadAdcData(int fd,struct sensor_msg *Sensor_data);
void CalcSampleData(struct sensor_msg *Sensor_data);
int master_adc(int argc, char *argv[]);


#ifdef __cplusplus
}
#endif

#endif 
