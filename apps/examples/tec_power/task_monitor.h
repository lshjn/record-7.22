#ifndef _TASK_MONITOR_H
#define _TASK_MONITOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include <nuttx/timers/rtc.h>


/****************************************************************************
 * Private function
 ****************************************************************************/
#define CONFIG_EXAMPLES_TIMER_DEVNAME 		"/dev/timer1_pid_samping"
#define CONFIG_EXAMPLES_TIMER_INTERVAL 		(1000000)
#define CONFIG_EXAMPLES_TIMER_SIGNO 17

struct pwm_state_s
{
  bool      initialized;
  uint8_t   duty;
  uint32_t  freq;
  uint32_t  count;
};
extern struct 	pwm_state_s g_pwmstate;

/****************************************************************************
 * Private function
 ****************************************************************************/
void  signal_timeInt(void);
void  signal_EXTER_CTR(void);
void set_pwm(float pwm_value);
void startup_pid_Sampling_timer(void);


int master_monitor(int argc, char *argv[]);




#ifdef __cplusplus
}
#endif

#endif 
