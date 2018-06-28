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

float read_DC_I(void)
{
	pid.DC_I_CUR_ADC = 1;
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



