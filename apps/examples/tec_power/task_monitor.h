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
#include "pid.h"

/****************************************************************************
 * Private function
 ****************************************************************************/
#define  CONFIG_EXAMPLES_TIMER_DEVNAME 		"/dev/timer1_pid_samping"
#define  CONFIG_EXAMPLES_TIMER_INTERVAL 		(pid.T*1000)     //unit us
#define  CONFIG_EXAMPLES_TIMER_SIGNO 17
#define  MAX31865_DEV1   1
#define  MAX31865_DEV2   2


extern int fd_EXTER_CTR;
extern int fd_max31865_1;
extern int fd_max31865_2;


/****************************************************************************
 * Private function
 ****************************************************************************/
void  signal_timeInt(void);
void  signal_EXTER_CTR(void);
void  startup_pid_Sampling_timer(void);


int master_monitor(int argc, char *argv[]);




#ifdef __cplusplus
}
#endif

#endif 
