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
