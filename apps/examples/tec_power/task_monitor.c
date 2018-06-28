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

#include "task_modbus.h"
#include "task_monitor.h"
#include "task_adc.h"
#include "task_spi.h"
#include "pid.h"


//采样时间到
void  signal_timeInt(void)
{
	pidctl_tecT();
}


//接收到io控温信号
void  signal_EXTER_CTR(void)
{
	//初始化pid参数
	PID_Init();
	//启动定时器
	startup_pid_Sampling_timer();
}

void set_pwm(float pwm_value)
{
	return;
}

void startup_pid_Sampling_timer(void)
{
	return;
}

/****************************************************************************
 * master_monitor
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
int master_monitor(int argc, char *argv[])
{
	PID_Init();

	while(1)
	{
		sleep(1); 
		//没有启动信号是否要执行读取温度带确定
		read_temper();		//读取当前温度
		PID_Calc(); 		//pid计算 
	}

 return EXIT_FAILURE;

}


