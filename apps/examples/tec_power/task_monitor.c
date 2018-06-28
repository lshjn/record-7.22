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


#include "task_modbus.h"
#include "task_monitor.h"
#include "task_adc.h"
#include "task_spi.h"
#include "pid.h"


struct 	pwm_state_s g_pwmstate;


//采样时间到
void  timerInt_action(void)
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
	static int timer_status = false; 

	if(!timer_status)
	{
		if (signo == SIGUSR1)
		{
			printf("start tec ctrl...\n");
			//初始化pid参数
			PID_Init();
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
/****************************************************************************
 * pwm_init
 * liushuhe
 * 2018.06.28
 ****************************************************************************/
void	pwm_init(int fd,float pwm_value)
{
	struct 	pwm_state_s pwmstate;
	struct pwm_info_s info;
	int ret;
	
	pwmstate.duty        = (uint8_t)((pwm_value/pid.pwmcycle)*100);
	pwmstate.freq        = pid.pwmcycle;
	
	info.frequency = pwmstate.freq;
	info.duty  = ((uint32_t)pwmstate.duty << 16) / 100;
	//set pwm info
	ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
	if (ret < 0)
	{
		printf("buzzalarm_pwm_init: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
	}
}
/****************************************************************************
 * pwm_start
 * liushuhe
 * 2017.10.12
 ****************************************************************************/
void	pwm_start(int fd)
{
	int ret;
	
	ret = ioctl(fd, PWMIOC_START, 0);
	if (ret < 0)
	{
		printf("pwm_start: ioctl(PWMIOC_START) failed: %d\n", errno);
	}
}
/****************************************************************************
 * pwm_stop
 * liushuhe
 * 2017.10.12
 ****************************************************************************/
void	pwm_stop(int fd)
{
	int ret;

	ret = ioctl(fd, PWMIOC_STOP, 0);
	if (ret < 0)
	{
		printf("pwm_stop: ioctl(PWMIOC_STOP) failed: %d\n", errno);
	}
}


/****************************************************************************
 * set_pwm
 * liushuhe
 * 2017.11.21
 ****************************************************************************/
void set_pwm(float pwm_value)
{
	static int fd_status = false;
	static int pwm_status = false;
	static int fd;

	if(!fd_status)
	{
		fd = open(CONFIG_EXAMPLES_PWM_DEVPATH, O_RDONLY);
		if (fd < 0)
		{
			printf("buzz_fd: open %s failed: %d\n", CONFIG_EXAMPLES_PWM_DEVPATH, errno);
		}
		else
		{
			fd_status = true;
		}
	}
	else
	{
		if(pwm_status)
		{
			pwm_stop(fd);
			pwm_status = false;
		}
	}

	pwm_init(fd,pwm_value);
	pwm_start(fd);
	pwm_status = true;
	return;
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
		read_DC_I();		
		sleep(1); 
		//没有启动信号是否要执行读取温度带确定
		read_temper();		//读取当前温度
		PID_Calc(); 		//pid计算 
	}

 return EXIT_FAILURE;

}


