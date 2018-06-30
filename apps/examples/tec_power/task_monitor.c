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



#include "task_modbus.h"
#include "task_monitor.h"
#include "adc.h"
#include "task_spi.h"
#include "pid.h"
#include "pwm.h"


int fd_EXTER_CTR;
int fd_max31865_1;
int fd_max31865_2;



//采样时间到
void  timerInt_action(void)
{
	pidctl_tecT(fd_max31865_1,MAX31865_DEV1);
}

/****************************************************************************
 * EXTER_CTR_Action 接收到io控温信号
 * liushuhe
 * 2018.06.28
 ****************************************************************************/
void EXTER_CTR_Action(int signo,siginfo_t *siginfo, void *arg)
{
	static int timer_status = false; 

	if(!timer_status)
	{
		if (signo == SIGUSR1)
		{
			printf("start tec ctrl...\n");
			//启动定时器
			startup_pid_Sampling_timer();
			timer_status = true;
		}

	}
	else
	{
		printf("already start tec ctrl !\n");
		return;
	}

}

void startup_pid_Sampling_timer(void)
{
	int 	fd_timer;
	int  	iRet = 0;
	struct  timer_notify_s notify;
	struct  sigaction act;

	fd_timer = open(CONFIG_EXAMPLES_TIMER_DEVNAME, O_RDWR);
	if (fd_timer < 0)
	{
		printf("ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_TIMER_DEVNAME, errno);
	}

	iRet = ioctl(fd_timer, TCIOC_SETTIMEOUT, CONFIG_EXAMPLES_TIMER_INTERVAL);
	if (iRet < 0)
	{
		printf("ERROR: Failed to set the timer interval: %d\n", errno);
	}
	
	act.sa_sigaction = timerInt_action;
	act.sa_flags     = SA_SIGINFO;

	(void)sigfillset(&act.sa_mask);
	(void)sigdelset(&act.sa_mask, CONFIG_EXAMPLES_TIMER_SIGNO);

	iRet = sigaction(CONFIG_EXAMPLES_TIMER_SIGNO, &act, NULL);
	if (iRet != OK)
	{
		printf("ERROR: Fsigaction failed: %d\n", errno);
	}
	
	notify.arg   = NULL;
	notify.pid   = getpid();
	notify.signo = CONFIG_EXAMPLES_TIMER_SIGNO;
	
	iRet = ioctl(fd_timer, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
	if (iRet < 0)
	{
		printf("ERROR: Failed to set the timer handler: %d\n", errno);
	}
  
	iRet = ioctl(fd_timer, TCIOC_START, 0);
	if (iRet < 0)
	{
		printf("ERROR: Failed to start the timer: %d\n", errno);
	}

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

	//max31865_1
	fd_max31865_1 = open(CONFIG_EXAMPLES_MAX31865_1_DEVPATH, O_RDONLY);
	if (fd_max31865_1 < 0)
	{
		printf("fd_max31865_1: open %s failed: %d\n", CONFIG_EXAMPLES_MAX31865_1_DEVPATH, errno);
	}
	//max31865_2
	fd_max31865_2 = open(CONFIG_EXAMPLES_MAX31865_2_DEVPATH, O_RDONLY);
	if (fd_max31865_2 < 0)
	{
		printf("fd_max31865_2: open %s failed: %d\n", CONFIG_EXAMPLES_MAX31865_2_DEVPATH, errno);
	}

	while(1)
	{
		read_DC_I();		
		sleep(1); 
		//没有启动信号是否要执行读取温度带确定
		read_temper(fd_max31865_1,MAX31865_DEV1);		//读取当前温度
		PID_Calc(); 		//pid计算 
	}

 return EXIT_FAILURE;

}


