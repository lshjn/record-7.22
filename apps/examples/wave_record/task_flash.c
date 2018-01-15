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


struct alarm_value  alarmdata;

/****************************************************************************
 * mid
 * liushuhe
 * 2017.10.23
 ****************************************************************************/
char * mid(char *dst,char *src, int n,int m) /*n为长度，m为位置*/  
{  
    char *p = src;  
    char *q = dst;  
    int len = strlen(src);  
    if(n>len) n = len-m;    /*从第m个到最后*/  
    if(m<0) m=0;    /*从第一个开始*/  
    if(m>len) return NULL;  
    p += m;  
    while(n--) *(q++) = *(p++);  
    *(q++)='\0'; /*有必要吗？很有必要*/  
    return dst;  
}  

/****************************************************************************
 * analyflashdata
 * liushuhe
 * 2017.10.23
 ****************************************************************************/
void analyflashdata(char **pcTempBuf)
{
    char   TempBuf[100];
	//get alarm data
	if(strstr(pcTempBuf[2],"mb=") != NULL)
	{
		mid(TempBuf,pcTempBuf[2],(strlen(pcTempBuf[2])-3),3);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.vcc_mb = VCC_MB_DEF;
		}
		else
		{
			alarmdata.vcc_mb = atoi(TempBuf);
		}
		printf("mb= %s,%d\n",TempBuf,alarmdata.vcc_mb);
	}
	if(strstr(pcTempBuf[3],"tempretrue=") != NULL)
	{
		mid(TempBuf,pcTempBuf[3],(strlen(pcTempBuf[3])-11),11);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.tempretrue = TEMPRETURE_DEF;
		}
		else
		{
			alarmdata.tempretrue = atoi(TempBuf);
		}
		printf("tempretrue= %s,%d\n",TempBuf,alarmdata.tempretrue);
	}
	if(strstr(pcTempBuf[4],"humidity=") != NULL)
	{
		mid(TempBuf,pcTempBuf[4],(strlen(pcTempBuf[4])-9),9);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.humidity = HUMIDITY_DEF;
		}
		else
		{
			alarmdata.humidity = atoi(TempBuf);
		}
		printf("humidity= %s,%d\n",TempBuf,alarmdata.humidity);
	}
	if(strstr(pcTempBuf[5],"co=") != NULL)
	{
		mid(TempBuf,pcTempBuf[5],(strlen(pcTempBuf[5])-3),3);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.co = CO_DEF;
		}
		else
		{
			alarmdata.co = atoi(TempBuf);
		}
		printf("co= %s,%d\n",TempBuf,alarmdata.co);
	}
	if(strstr(pcTempBuf[6],"h2s=") != NULL)
	{
		mid(TempBuf,pcTempBuf[6],(strlen(pcTempBuf[6])-4),4);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.h2s = H2S_DEF;
		}
		else
		{
			alarmdata.h2s = atoi(TempBuf);
		}
		printf("h2s= %s,%d\n",TempBuf,alarmdata.h2s);
	}
	if(strstr(pcTempBuf[7],"nh3=") != NULL)
	{
		mid(TempBuf,pcTempBuf[7],(strlen(pcTempBuf[7])-4),4);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.nh3 = NH3_DEF;
		}
		else
		{
			alarmdata.nh3 = atoi(TempBuf);
		}
		printf("nh3= %s,%d\n",TempBuf,alarmdata.nh3);
	}
	if(strstr(pcTempBuf[8],"o2=") != NULL)
	{
		mid(TempBuf,pcTempBuf[8],(strlen(pcTempBuf[8])-3),3);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.o2 = O2_DEF;
		}
		else
		{
			alarmdata.o2 = atoi(TempBuf);
		}
		printf("o2= %s,%d\n",TempBuf,alarmdata.o2);
	}
	if(strstr(pcTempBuf[9],"water=") != NULL)
	{
		mid(TempBuf,pcTempBuf[9],(strlen(pcTempBuf[9])-6),6);
		if(strstr(TempBuf,"NULL") != NULL)
		{
			alarmdata.water = WATER_DEF;
		}
		else
		{
			alarmdata.water = atoi(TempBuf);
		}
		printf("water= %s,%d\n",TempBuf,alarmdata.water);
	}
}


/****************************************************************************
 * slave_flash
 * liushuhe
 * 2017.10.23
 ****************************************************************************/

int master_flash(int argc, char *argv[])
{
    char   *pcTempBuf[255];
    char   *pChar = ";";
	int		cCharNum = 0;

	FAR uint32_t *buffer;
	ssize_t nbytes;
	int fd;

	buffer = (FAR uint32_t *)malloc(200);
	if (!buffer)
	{
		printf("ERROR: failed to allocate a sector buffer\n");
	}

	//check flash
	fd = open(CONFIG_EXAMPLES_FLASH_DEVPATH, O_RDONLY);
	if (fd < 0)
	{
		printf("slave_flash: open %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH, errno);
	}
	nbytes = read(fd, buffer, 200);
	if (nbytes < 0)
	{
		printf("ERROR: read from %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH,errno);
	}
	else
	{
		/*
		if((strstr((char *)buffer,"##time=") != NULL)&&(strlen((char *)buffer) > 50))
		{
			printf("read flash data:%s\n",buffer);
			cCharNum = 0;
			memset(pcTempBuf, 0, sizeof(pcTempBuf));
			pcTempBuf[0] = strtok((char*)buffer, pChar);
			while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
			{
			}
			//get alarm data
			analyflashdata((char **)&pcTempBuf);
		}
		else
		{
			close(fd);
			//init flash data
			fd = open(CONFIG_EXAMPLES_FLASH_DEVPATH, O_WRONLY);
			if (fd < 0)
			{
				printf("slave_flash: open %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH, errno);
			}
			//must memset
			//add by liushuhe 2017.11.10
			memset(buffer, 0, sizeof(buffer));
			sprintf((char*)buffer,"%s","##time=20151020170733;locker=off;mb=%d;tempretrue=%d;humidity=%d;co=%d;h2s=%d;nh3=%d;o2=%d;water=%d;sb=%d;id=123@@",
								VCC_MB_DEF,TEMPRETURE_DEF,HUMIDITY_DEF,CO_DEF,H2S_DEF,NH3_DEF,O2_DEF,WATER_DEF,VCC_SB_DEF);
			nbytes = write(fd, buffer, strlen((char *)buffer));
			if (nbytes < 0)
			{
				printf("ERROR: write to /dev/mtd0 failed: %d\n", errno);
			}

		  	close(fd);
			alarmdata.vcc_mb		= VCC_MB_DEF;
			alarmdata.tempretrue 	= TEMPRETURE_DEF;
			alarmdata.humidity 		= HUMIDITY_DEF;
			alarmdata.co 			= CO_DEF;
			alarmdata.h2s 			= H2S_DEF;
			alarmdata.nh3 			= NH3_DEF;
			alarmdata.o2 			= O2_DEF;
			alarmdata.water 		= WATER_DEF;
		}
		*/
        /*********************************************************************************/
		//test
		printf("test------read flash data:%s\n",buffer);
		if((strstr((char *)buffer,"##time=") != NULL)&&(strlen((char *)buffer) > 50))
		{
			printf("read flash data:%s\n",buffer);
			cCharNum = 0;
			memset(pcTempBuf, 0, sizeof(pcTempBuf));
			pcTempBuf[0] = strtok((char*)buffer, pChar);
			while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
			{
			}
			//get alarm data
			analyflashdata((char **)&pcTempBuf);

			close(fd);
			//init flash data
			fd = open(CONFIG_EXAMPLES_FLASH_DEVPATH, O_WRONLY);
			if (fd < 0)
			{
				printf("slave_flash: open %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH, errno);
			}
			memset(buffer, 0, sizeof(buffer));
			sprintf((char*)buffer,"%s","123456789=====================================");
			nbytes = write(fd, buffer, strlen((char *)buffer));
			if (nbytes < 0)
			{
				printf("ERROR: write to /dev/mtd0 failed: %d\n", errno);
			}

		  	close(fd);
		}
		else if((strstr((char *)buffer,"123456789") != NULL)&&(strlen((char *)buffer) > 50))
		{
			printf("read flash data:%s\n",buffer);
			cCharNum = 0;
			memset(pcTempBuf, 0, sizeof(pcTempBuf));
			pcTempBuf[0] = strtok((char*)buffer, pChar);
			while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
			{
			}
			//get alarm data
			analyflashdata((char **)&pcTempBuf);

			close(fd);
			//init flash data
			fd = open(CONFIG_EXAMPLES_FLASH_DEVPATH, O_WRONLY);
			if (fd < 0)
			{
				printf("slave_flash: open %s failed: %d\n", CONFIG_EXAMPLES_FLASH_DEVPATH, errno);
			}
			memset(buffer, 0, sizeof(buffer));
			sprintf((char*)buffer,"%s","##time=20151020170733;locker=off;mb=%d;tempretrue=%d;humidity=%d;co=%d;h2s=%d;nh3=%d;o2=%d;water=%d;sb=%d;id=123@@",
								VCC_MB_DEF,TEMPRETURE_DEF,HUMIDITY_DEF,CO_DEF,H2S_DEF,NH3_DEF,O2_DEF,WATER_DEF,VCC_SB_DEF);
			nbytes = write(fd, buffer, strlen((char *)buffer));
			if (nbytes < 0)
			{
				printf("ERROR: write to /dev/mtd0 failed: %d\n", errno);
			}

		  	close(fd);
		}
        /*********************************************************************************/
		
	}
	
	//close(fd);
	free(buffer);
	
	while(1)
	{
		sleep(1);
		//printf("slave_flash---\n");
		/*
		if(alarmdata.write_flag == 1)
		{
			alarmdata.write_flag = 0;
			if(strstr(alarmdata.msg,"##time=") != NULL)
			{
				printf(" refresh alarm data:%s\n",alarmdata.msg);
				cCharNum = 0;
				memset(pcTempBuf, 0, sizeof(pcTempBuf));
				pcTempBuf[0] = strtok((char*)alarmdata.msg, pChar);
				while((pcTempBuf[++cCharNum] = strtok(NULL, pChar))!= NULL)  																																											//分解字符串
				{
				}
				analyflashdata((char **)&pcTempBuf);
			}
		}
		*/
	}

  return 0;
}


