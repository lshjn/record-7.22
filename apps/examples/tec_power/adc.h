#ifndef _ADC_H
#define _ADC_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include <nuttx/analog/adc.h>


/****************************************************************************
 * Private Data
 ****************************************************************************/
struct sensor_msg {
	struct adc_msg_s	sample_tempdata[1];
};

float CalcSampleData(struct sensor_msg *Sensor_data);
int	ReadAdcData(int fd,struct sensor_msg *Sensor_data);
int	StartAdcSampl(int fd);

float read_DC_I(void);



#ifdef __cplusplus
}
#endif

#endif 
