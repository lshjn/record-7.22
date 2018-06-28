#ifndef _PWM_H
#define _PWM_H

#ifdef __cplusplus
 extern "C" {
#endif 
/****************************************************************************
 * Private Data
 ****************************************************************************/
	
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
#include <nuttx/timers/timer.h>
#include <termios.h>


#include <nuttx/drivers/pwm.h>


struct pwm_state_s
{
  bool      initialized;
  uint8_t   duty;
  uint32_t  freq;
  uint32_t  count;
};


extern struct 	pwm_state_s g_pwmstate;



void set_pwm(float pwm_value);
void	pwm_stop(int fd);
void	pwm_start(int fd);
void	pwm_init(int fd,float pwm_value);



#ifdef __cplusplus
}
#endif

#endif 
