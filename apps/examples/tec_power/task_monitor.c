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

/****************************************************************************
 * EXTER_CTR_Action 接收到io控温信号
 * liushuhe
 * 2018.06.28
 ****************************************************************************/
void EXTER_CTR_Action(int signo,siginfo_t *siginfo, void *arg)
{
	static int cnt; 
	if (signo == SIGUSR1)
	{
		printf("%2d SIGUSR1 received\n",cnt++);
		//初始化pid参数
		PID_Init();
		//启动定时器
		startup_pid_Sampling_timer();

	}
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
	enum gpio_pintype_e pintype;
	struct sigaction act;
	struct sigaction oldact;
	int ret;
	int status;
	int fd_EXTER_CTR;
	
	fd_EXTER_CTR = open(CONFIG_EXAMPLES_EXTER_CTR_DEVPATH, O_RDONLY);
	if (fd_EXTER_CTR < 0)
	{
		printf("fd_EXTER_CTR: open %s failed: %d\n", CONFIG_EXAMPLES_EXTER_CTR_DEVPATH, errno);
	}
	ret = ioctl(fd_EXTER_CTR, GPIOC_PINTYPE, (unsigned long)((uintptr_t)&pintype));
	if (ret < 0)
	{
		int errcode = errno;
		fprintf(stderr, "ERROR: Failed to read pintype from %s: %d\n", CONFIG_EXAMPLES_EXTER_CTR_DEVPATH, errcode);
		close(fd_EXTER_CTR);
	}
	
	//signal
	memset(&act, 0, sizeof(struct sigaction));
	act.sa_sigaction = EXTER_CTR_Action;
	act.sa_flags     = SA_SIGINFO;

	(void)sigemptyset(&act.sa_mask);

	status = sigaction(SIGUSR1, &act, &oldact);
	if (status != 0)
	{
		fprintf(stderr, "Failed to install SIGUSR1 handler, errno=%d\n",errno);
	}
	// Set up to receive signal 
	if(pintype == GPIO_INTERRUPT_PIN)
	{	
		ret = ioctl(fd_EXTER_CTR, GPIOC_REGISTER, (unsigned long)SIGUSR1);
		if (ret < 0)
		{
			int errcode = errno;
			fprintf(stderr, "ERROR: Failed to setup for signal from %s: %d\n", CONFIG_EXAMPLES_EXTER_CTR_DEVPATH, errcode);
			close(fd_EXTER_CTR);
		}
	}


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


