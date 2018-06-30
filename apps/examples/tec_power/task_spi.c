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

#include "task_spi.h"
#include "pid.h"
#include "task_modbus.h"
#include "max31865.h"
#include "task_monitor.h"


/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * master_adc
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_spi(int argc, char *argv[])
{
	while(1)
	{
		sleep(1);
	}

  return 0;
}


/****************************************************************************
 * read_temper
 * liushuhe
 * 2018.06.29
 ****************************************************************************/
void read_temper(int fd,int dev_num)
{
	float   AD_MAX81365 = 0;
	float   TEMPER_MAX81365 = 0;
	bool	DRDY_PIN_VALUE = DRDY_INVALID;
	
	Init_max31865(fd);
	memset(max31856_databuf,0,sizeof(max31856_databuf));

	start_conversion(fd);

	//wait max31865 drdy
	while(DRDY_PIN_VALUE == DRDY_INVALID)
	{
		if(dev_num == MAX31865_DEV1)
		{
			boardctl(BOARDIOC_GET_SPI1_DRDY, (uintptr_t)(&DRDY_PIN_VALUE));
		}
		else if(dev_num == MAX31865_DEV2)
		{
			boardctl(BOARDIOC_GET_SPI2_DRDY, (uintptr_t)(&DRDY_PIN_VALUE));
		}
		usleep(1000);
	}
	
	read_max31865(fd,max31856_databuf,sizeof(max31856_databuf));				//读取max31865当前的温度值

	if((max31856_databuf[ADDR_RTD_LSB]&0x01)==0x01)
	{
		Fault_Detect(fd);
	}
	else
	{
		AD_MAX81365 = ((max31856_databuf[ADDR_RTD_MSB]<<8)|max31856_databuf[ADDR_RTD_LSB])>>1;
		TEMPER_MAX81365 = ((AD_MAX81365/32)-256); 
		pid.Pv=TEMPER_MAX81365; 
		//设置modbus数据结构
		g_modbus.reginput[0] = pid.Pv;
	}
}




