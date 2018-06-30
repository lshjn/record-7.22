#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <nuttx/analog/adc.h>
//#include <nuttx/analog/ioctl.h>
#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <debug.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_flash.h"
#include "pid.h"

/****************************************************************************
 * readdata
 * liushuhe
 * 2018.06.30
 ****************************************************************************/
int readdata(const char *path,char  *buf,uint32_t len)
{
	int fd = -1;
	ssize_t nbytes = 0;

	fd = open(path, O_RDONLY|O_CREAT|O_EXCL ,0666);
	if (errno ==  EEXIST)		 //文件存在
	{
		printf("file exist already ,try open %s :O_RDONLY\n", path);
		fd = open(path, O_RDONLY,0666);
		if (fd < 0)
		{
			printf("try open %s :O_RDONLY failed:%d \n", path, errno);
			close(fd);
			return 0;
		}
		else
		{
			printf("try open %s :O_RDONLY successed \n", path);
		}
	}
	else if(errno == ENOENT)    //文件不存在
	{
		fd = open(path, O_WRONLY|O_CREAT|O_EXCL ,0666);
		if(fd > 0)
		{
			printf("creat %s :O_RDONLY successed \n",path);
		}
		close(fd);
		return 0;	
	}
	else if(fd > 0)
	{
		printf("fd_spiflash: open %s successed\n", path);
	}
	else
	{
		printf("error readdata:other %s %d\n", path,errno);
		close(fd);
		return 0;	
	}

	nbytes = read(fd, buf, len);
	if(nbytes <= 0)
	{
		printf("warning: read 0 bytes from %s\n",path);
	}
	
	close(fd);
	return nbytes;
}

/****************************************************************************
 * writedata
 * liushuhe
 * 2018.06.30
 ****************************************************************************/
int writedata(const char *path,char  *buf,uint32_t len)
{
	int fd = -1;
	ssize_t nbytes = 0;

	fd = open(path, O_WRONLY|O_CREAT|O_EXCL ,0666);
	if (errno ==  EEXIST)		 //文件存在
	{
		printf("file exist already ,try open %s :O_WRONLY|O_TRUNC\n", path);
		fd = open(path, O_WRONLY|O_CREAT|O_TRUNC,0666);
		if (fd < 0)
		{
			printf("try open %s :O_WRONLY|O_CREAT|O_TRUNCC failed:%d \n", path, errno);
			close(fd);
			return 0;
		}
		else
		{
			printf("try open %s :O_WRONLY|O_CREAT|O_TRUNC successed \n", path);
		}
	}
	else if(errno == ENOENT)    //文件不存在
	{
		fd = open(path, O_WRONLY|O_CREAT|O_EXCL ,0666);
		if(fd > 0)
		{
			printf("creat %s :O_WRONLY successed \n",path);
			
			nbytes = write(fd, buf, len);
			if(nbytes <= 0)
			{
				printf("warning: write 0 bytes from %s\n",path);
			}
		}
		close(fd);
		return 0;	
	}
	else if(fd > 0)
	{
		printf("fd_spiflash: open %s successed\n", path);
	}
	else
	{
		printf("error writedata:other %s %d\n", path,errno);
		close(fd);
		return 0;	
	}

	nbytes = write(fd, buf, len);
	if(nbytes <= 0)
	{
		printf("warning: write 0 bytes from %s\n",path);
	}
	
	close(fd);
	return nbytes;
}


/****************************************************************************
 * updata_pidarg_fromflash
 * liushuhe
 * 2018.06.30
 ****************************************************************************/
int updata_pidarg_fromflash(PID *pid)
{
	#define  DELAY  10*1000
	char  buf[128];
	
	usleep(DELAY);
	//pid.DC_I_MAX
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_DC_I_MAX,buf,sizeof(buf)))
	{
		printf("pid.DC_I_MAX=%.2f\n",atof(buf));
		pid->DC_I_MAX = atof(buf);
	}
	usleep(DELAY);
	//pid.Sv
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_SV,buf,sizeof(buf)))
	{
		printf("pid.Sv=%.2f\n",atof(buf));
		pid->Sv = atof(buf);
	}
	usleep(DELAY);
	//pid.T
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_T,buf,sizeof(buf)))
	{
		printf("pid.T=%d\n",atoi(buf));
		pid->T = atoi(buf);
	}
	usleep(DELAY);
	//pid.Kp
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_KP,buf,sizeof(buf)))
	{
		printf("pid.Kp=%.2f\n",atof(buf));
		pid->Kp = atof(buf);
	}
	usleep(DELAY);
	//pid.Ti
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_TI,buf,sizeof(buf)))
	{
		printf("pid.Ti=%d\n",atoi(buf));
		pid->Ti = atoi(buf);
	}
	usleep(DELAY);
	//pid.Td
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_TD,buf,sizeof(buf)))
	{
		printf("pid.Td=%d\n",atoi(buf));
		pid->Td = atoi(buf);
	}
	usleep(DELAY);
	//pid.pwmcycle
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_PWMCYC,buf,sizeof(buf)))
	{
		printf("pid.pwmcycle=%d\n",atoi(buf));
		pid->pwmcycle = atoi(buf);
	}
	usleep(DELAY);
	//pid.OUT0
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_OUT0,buf,sizeof(buf)))
	{
		printf("pid.OUT0=%.2f\n",atof(buf));
		pid->OUT0 = atof(buf);
	}
}
/****************************************************************************
 * updata_pidarg_modbusToflash
 * liushuhe
 * 2018.06.30
 ****************************************************************************/
int updata_pidarg_modbusToflash(PID *pid,int cmd_index)
{
	char  buf[128];
	switch(cmd_index)
	{
		case CMD_PID_DC_I_MAX:
			//pid.DC_I_MAX
			sprintf(buf,"%.2f",pid->DC_I_MAX);
			writedata(CONFIG_DEVPATH_PID_DC_I_MAX,buf,strlen(buf));
			break;
		case CMD_PID_SV:
			//pid.Sv
			sprintf(buf,"%.2f",pid->Sv);
			writedata(CONFIG_DEVPATH_PID_SV,buf,strlen(buf));
			break;
		case CMD_PID_T:
			//pid.T
			sprintf(buf,"%d",pid->T);
			writedata(CONFIG_DEVPATH_PID_T,buf,strlen(buf));
			break;
		case CMD_PID_KP:
			//pid.Kp
			sprintf(buf,"%.2f",pid->Kp);
			writedata(CONFIG_DEVPATH_PID_KP,buf,strlen(buf));
			break;
		case CMD_PID_TI:
			//pid.Ti
			sprintf(buf,"%d",pid->Ti);
			writedata(CONFIG_DEVPATH_PID_TI,buf,strlen(buf));
			break;
		case CMD_PID_TD:
			//pid.Td
			sprintf(buf,"%d",pid->Td);
			writedata(CONFIG_DEVPATH_PID_TD,buf,strlen(buf));
			break;
		case CMD_PID_PWMCYC:
			//pid.pwmcycle
			sprintf(buf,"%d",pid->pwmcycle);
			writedata(CONFIG_DEVPATH_PID_PWMCYC,buf,strlen(buf));
			break;
		case CMD_PID_OUT0:
			//pid.OUT0
			sprintf(buf,"%.2f",pid->OUT0);
			writedata(CONFIG_DEVPATH_PID_OUT0,buf,strlen(buf));
			break;
	}
}

/****************************************************************************
 * master_flash
 * liushuhe
 * 2018.06.30
 ****************************************************************************/

int master_flash(int argc, char *argv[])
{
	//pid给个初始值，随后从flash里读取设置值，进行更新
	PID_Init();
	
	updata_pidarg_modbusToflash(&pid,CMD_PID_DC_I_MAX);
	updata_pidarg_modbusToflash(&pid,CMD_PID_SV);
	updata_pidarg_modbusToflash(&pid,CMD_PID_T);
	updata_pidarg_modbusToflash(&pid,CMD_PID_KP);
	updata_pidarg_modbusToflash(&pid,CMD_PID_TI);
	updata_pidarg_modbusToflash(&pid,CMD_PID_TD);
	updata_pidarg_modbusToflash(&pid,CMD_PID_PWMCYC);
	updata_pidarg_modbusToflash(&pid,CMD_PID_OUT0);
	
	updata_pidarg_fromflash(&pid);

	while(1)
	{
		sleep(1);
	}

  return 0;
}


