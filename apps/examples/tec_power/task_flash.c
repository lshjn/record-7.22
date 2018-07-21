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
#include "task_modbus.h"

#include "pid.h"

pthread_mutex_t g_FlashMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_FlashConVar		= PTHREAD_COND_INITIALIZER;

bool	enFlashUpdata = false;

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
char updata_pidarg_fromflash(PID *pid)
{
	#define  DELAY  10*1000
	char  buf[128];
	char  updata_status = 0;
	
	usleep(DELAY);
	//pid.I_MAX
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_I_MAX,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.I_MAX=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->I_MAX = atof(buf);
		updata_status |= (1<<7);
	}
	usleep(DELAY);
	//pid.Sv
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_SV,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.Sv=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->Sv = atof(buf);
		updata_status |= (1<<6);
	}
	usleep(DELAY);
	//pid.T
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_T,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.T=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->T = atof(buf);
		updata_status |= (1<<5);
	}
	usleep(DELAY);
	//pid.Kp
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_KP,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.Kp=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->Kp = atof(buf);
		updata_status |= (1<<4);
	}
	usleep(DELAY);
	//pid.Ti
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_TI,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.Ti=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->Ti = atof(buf);
		updata_status |= (1<<3);
	}
	usleep(DELAY);
	//pid.Td
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_TD,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.Td=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->Td = atof(buf);
		updata_status |= (1<<2);
	}
	usleep(DELAY);
	//pid.pwmcycle
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_PWMCYC,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.pwmcycle=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->pwmcycle = atof(buf);
		updata_status |= (1<<1);
	}
	usleep(DELAY);
	//pid.OUT0
	memset(buf,0,sizeof(buf));
	if(readdata(CONFIG_DEVPATH_PID_OUT0,buf,sizeof(buf)))
	{
		printf("------------------------\n");
		printf("|pid.OUT0=%.2f|\n",atof(buf));
		printf("------------------------\n");
		pid->OUT0 = atof(buf);
		updata_status |= (1<<0);
	}
	return updata_status;
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
		case CMD_PID_I_MAX:
			//pid.I_MAX
			sprintf(buf,"%.2f",pid->I_MAX);
			writedata(CONFIG_DEVPATH_PID_I_MAX,buf,strlen(buf));
			break;
		case CMD_PID_SV:
			//pid.Sv
			sprintf(buf,"%.2f",pid->Sv);
			writedata(CONFIG_DEVPATH_PID_SV,buf,strlen(buf));
			break;
		case CMD_PID_T:
			//pid.T
			sprintf(buf,"%.2f",pid->T);
			writedata(CONFIG_DEVPATH_PID_T,buf,strlen(buf));
			break;
		case CMD_PID_KP:
			//pid.Kp
			sprintf(buf,"%.2f",pid->Kp);
			writedata(CONFIG_DEVPATH_PID_KP,buf,strlen(buf));
			break;
		case CMD_PID_TI:
			//pid.Ti
			sprintf(buf,"%.2f",pid->Ti);
			writedata(CONFIG_DEVPATH_PID_TI,buf,strlen(buf));
			break;
		case CMD_PID_TD:
			//pid.Td
			sprintf(buf,"%.2f",pid->Td);
			writedata(CONFIG_DEVPATH_PID_TD,buf,strlen(buf));
			break;
		case CMD_PID_PWMCYC:
			//pid.pwmcycle
			sprintf(buf,"%.2f",pid->pwmcycle);
			writedata(CONFIG_DEVPATH_PID_PWMCYC,buf,strlen(buf));
			break;
		case CMD_PID_OUT0:
			//pid.OUT0
			sprintf(buf,"%.2f",pid->OUT0);
			writedata(CONFIG_DEVPATH_PID_OUT0,buf,strlen(buf));
			break;
	}
	return 0;
}
/****************************************************************************
 * EnFlashUpdata
 * liushuhe
 * 2018.07.1
 ****************************************************************************/
void  EnFlashUpdata(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	if(!pthread_mutex_trylock(mutex))
	{
		enFlashUpdata = true;
		pthread_cond_signal(cond);
		pthread_mutex_unlock(mutex);
	}
}

/****************************************************************************
 * CheckFlashdata
 * liushuhe
 * 2018.07.1
 ****************************************************************************/
void  CheckFlashdata(PID *pid,PID *pid_modbus,char updata_status)
{
	if(!(updata_status & (1<<7)))
	{
		pid->I_MAX = DEF_I_MAX;
		pid_modbus->I_MAX  = pid->I_MAX;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_I_MAX);
	}
	else
	{
		pid_modbus->I_MAX  = pid->I_MAX;
	}


	if(!(updata_status & (1<<6)))
	{
		pid->Sv = DEF_SV;
		pid_modbus->Sv  = pid->Sv;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_SV);
	}
	else
	{
		pid_modbus->Sv  = pid->Sv;
	}

	
	if(!(updata_status & (1<<5)))
	{
		pid->T = DEF_T;
		pid_modbus->T  = pid->T;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_T);
	}
	else
	{
		pid_modbus->T  = pid->T;
	}
	
	if(!(updata_status & (1<<4)))
	{
		pid->Kp = DEF_KP;
		pid_modbus->Kp  = pid->Kp;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_KP);
	}
	else
	{
		pid_modbus->Kp  = pid->Kp;
	}


	if(!(updata_status & (1<<3)))
	{
		pid->Ti = DEF_TI;
		pid_modbus->Ti  = pid->Ti;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_TI);
	}
	else
	{
		pid_modbus->Ti  = pid->Ti;
	}
	
	if(!(updata_status & (1<<2)))
	{
		pid->Td = DEF_TD;
		pid_modbus->Td  = pid->Td;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_TD);
	}
	else
	{
		pid_modbus->Td  = pid->Td;
	}

	if(!(updata_status & (1<<1)))
	{
		pid->pwmcycle = DEF_PWMCYCLE;
		pid_modbus->pwmcycle  = pid->pwmcycle;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_PWMCYC);
	}
	else
	{
		pid_modbus->pwmcycle  = pid->pwmcycle;
	}

	if(!(updata_status & (1<<0)))
	{
		pid->OUT0 = DEF_OUT0;
		pid_modbus->OUT0  = pid->OUT0;
		updata_pidarg_modbusToflash(pid_modbus,CMD_PID_OUT0);
	}
	else
	{
		pid_modbus->OUT0  = pid->OUT0;
	}
	
}

/****************************************************************************
 * master_flash
 * liushuhe
 * 2018.06.30
 ****************************************************************************/

int master_flash(int argc, char *argv[])
{
	char updata_status = 0;

	updata_status = updata_pidarg_fromflash(&g_pid);
	CheckFlashdata(&g_pid,&g_pid_modbus,updata_status);

	g_modbus.regholding[0] = ((uint32_t)g_pid_modbus.I_MAX >> 16)&0xffff ;
	g_modbus.regholding[1] = (uint32_t)g_pid_modbus.I_MAX & 0xffff;
		
	g_modbus.regholding[2] = ((uint32_t)g_pid_modbus.Sv >> 16)&0xffff ;
	g_modbus.regholding[3] = (uint32_t)g_pid_modbus.Sv & 0xffff;
	
	g_modbus.regholding[4] = ((uint32_t)g_pid_modbus.T >> 16)&0xffff ;
	g_modbus.regholding[5] = (uint32_t)g_pid_modbus.T & 0xffff;
	
	g_modbus.regholding[6] = ((uint32_t)g_pid_modbus.Kp >> 16)&0xffff ;
	g_modbus.regholding[7] = (uint32_t)g_pid_modbus.Kp & 0xffff;
	
	g_modbus.regholding[8] = ((uint32_t)g_pid_modbus.Ti >> 16)&0xffff ;
	g_modbus.regholding[9] = (uint32_t)g_pid_modbus.Ti & 0xffff;
	
	g_modbus.regholding[10] = ((uint32_t)g_pid_modbus.Td >> 16)&0xffff ;
	g_modbus.regholding[11] = (uint32_t)g_pid_modbus.Td & 0xffff;
	
	g_modbus.regholding[12] = ((uint32_t)g_pid_modbus.pwmcycle >> 16)&0xffff ;
	g_modbus.regholding[13] = (uint32_t)g_pid_modbus.pwmcycle & 0xffff;
	
	g_modbus.regholding[14] = ((uint32_t)g_pid_modbus.OUT0 >> 16)&0xffff ;
	g_modbus.regholding[15] = (uint32_t)g_pid_modbus.OUT0 & 0xffff;
	
#if 1	
	printf("g_modbus.regholding[0]=%x\n",g_modbus.regholding[0]);
	printf("g_modbus.regholding[1]=%x\n",g_modbus.regholding[1]);
	printf("g_modbus.regholding[2]=%x\n",g_modbus.regholding[2]);
	printf("g_modbus.regholding[3]=%x\n",g_modbus.regholding[3]);
	printf("g_modbus.regholding[4]=%x\n",g_modbus.regholding[4]);
	printf("g_modbus.regholding[5]=%x\n",g_modbus.regholding[5]);
	printf("g_modbus.regholding[6]=%x\n",g_modbus.regholding[6]);
	printf("g_modbus.regholding[7]=%x\n",g_modbus.regholding[7]);
	printf("g_modbus.regholding[8]=%x\n",g_modbus.regholding[8]);
	printf("g_modbus.regholding[9]=%x\n",g_modbus.regholding[9]);
	printf("g_modbus.regholding[10]=%x\n",g_modbus.regholding[10]);
	printf("g_modbus.regholding[11]=%x\n",g_modbus.regholding[11]);
	printf("g_modbus.regholding[12]=%x\n",g_modbus.regholding[12]);
	printf("g_modbus.regholding[13]=%x\n",g_modbus.regholding[13]);
	printf("g_modbus.regholding[14]=%x\n",g_modbus.regholding[14]);
	printf("g_modbus.regholding[15]=%x\n",g_modbus.regholding[15]);
#endif	

	while(1)
	{
		if(!pthread_mutex_trylock(&g_FlashMutex))
		{
			while(enFlashUpdata!= true)
			{
				pthread_cond_wait(&g_FlashConVar, &g_FlashMutex);
			}
			enFlashUpdata = false;
			
			//更新pid数据结构

			g_pid_modbus.I_MAX		= (g_modbus.regholding[0]<<16)|g_modbus.regholding[1];
			g_pid_modbus.Sv			= (g_modbus.regholding[2]<<16)|g_modbus.regholding[3];
			g_pid_modbus.T			= (g_modbus.regholding[4]<<16)|g_modbus.regholding[5];
			g_pid_modbus.Kp			= (g_modbus.regholding[6]<<16)|g_modbus.regholding[7];
			g_pid_modbus.Ti			= (g_modbus.regholding[8]<<16)|g_modbus.regholding[9];
			g_pid_modbus.Td			= (g_modbus.regholding[10]<<16)|g_modbus.regholding[11];
			g_pid_modbus.pwmcycle		= (g_modbus.regholding[12]<<16)|g_modbus.regholding[13];
			g_pid_modbus.OUT0			= (g_modbus.regholding[14]<<16)|g_modbus.regholding[15];

			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_I_MAX);
			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_SV);
			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_T);
			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_KP);
			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_TI);
			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_TD);
			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_PWMCYC);
			updata_pidarg_modbusToflash(&g_pid_modbus,CMD_PID_OUT0);
			
			updata_pidarg_fromflash(&g_pid);	
			
			pthread_mutex_unlock(&g_FlashMutex);
		}
		else
		{
			sleep(1);
		}

	}

  return 0;
}


