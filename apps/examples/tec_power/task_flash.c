#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <nuttx/analog/adc.h>
//#include <nuttx/analog/ioctl.h>
#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <debug.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_flash.h"
#include "pid.h"


/****************************************************************************
 * slave_flash
 * liushuhe
 * 2017.10.23
 ****************************************************************************/

int master_flash(int argc, char *argv[])
{
	while(1)
	{
		sleep(1);
	}

  return 0;
}


