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


//����ʱ�䵽
void  signal_timeInt(void)
{
	pidctl_tecT();
}


//���յ�io�����ź�
void  signal_EXTER_CTR(void)
{
	//��ʼ��pid����
	PID_Init();
	//������ʱ��
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
		//û�������ź��Ƿ�Ҫִ�ж�ȡ�¶ȴ�ȷ��
		read_temper();		//��ȡ��ǰ�¶�
		PID_Calc(); 		//pid���� 
	}

 return EXIT_FAILURE;

}


