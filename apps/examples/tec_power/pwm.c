#include "pwm.h"
#include "pid.h"

struct 	pwm_state_s g_pwmstate;



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
	
	pwmstate.duty        = (uint8_t)(pwm_value);
	pwmstate.freq        = pid.pwmcycle;
	
	info.frequency = pwmstate.freq;
	info.duty  = ((uint32_t)pwmstate.duty << 16)/pid.pwmcycle;
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

