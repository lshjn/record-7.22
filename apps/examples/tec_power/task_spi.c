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
 * master_adc
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
void Max31865_Init()
{
	return;
}

/****************************************************************************
 * read_max31865
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
uint16_t read_max31865(void)  
{
	uint16_t d;

	d = 1;

	return d;
}

/****************************************************************************
 * read_temper
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
void read_temper(void)
{
	uint16_t d;
	d=read_max31865();				//读取max31865当前的温度值
	pid.Pv=((d>>4)&0x0fff)*0.25;	//

	//设置modbus数据结构
	g_modbus.reginput[0] = pid.Pv;
}


