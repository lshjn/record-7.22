#include <nuttx/config.h>
#include <stdlib.h>
#include <stdbool.h>
#include <strings.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <signal.h>
#include <pthread.h>

#include <nuttx/i2c/i2c_master.h>

#include "tmp431.h"
#include "ds1338.h"

int DS1338_SECOND_REG		=0x00;
int DS1338_MINURES_REG		=0x01;
int DS1338_HOUR_REG		=0x02;
int DS1338_DAY_REG			=0x03;
int DS1338_DATE_REG		=0x04;
int DS1338_MONTH_REG		=0x05;
int DS1338_YEAR_REG		=0x06;
int DS1338_CTOL_REG		=0x07;


struct TimeStruct rtcds1338;

static bool  g_ds1338_started;
static	int  fd_ds1338;


/*****************************************************************************************************
函数名称：CalcWeek
功		能：计算出参数日期的星期
输入参数：时间结构体指针
输出参数：
编写时间：2017.11.18
作		者：lsh
******************************************************************************************************/
void CalcWeek(struct TimeStruct *Time)
{
    static int mdays[]= {0,31,28,31,30,31,30,31,31,30,31,30};
    int i,y = Time->Year-1,days = Time->Day;
    for(i = 0; i < Time->Month; ++i)
    {
        days+= mdays[i];
    }

    if(Time->Month > 2)
    {
        if(((Time->Year % 400) == 0) || ((Time->Year & 3) == 0 && (Time->Year % 100)))
            ++days;
    }

    Time->Week=(y+y/4-y/100+y/400+days)%7;

    if(Time->Week == 0)
    {
        Time->Week = 7;
    }
}

/******************************************************************************************************************************
函数名称：en_local_chanel
功    能：使能本地通道
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int init_ds1338_time  (int fd,struct  TimeStruct *rtc)
{
	struct i2c_msg_s msg[1];
	uint8_t setbuf[9];

	CalcWeek(rtc);
	setbuf[0] = (uint8_t)DS1338_SECOND_REG;
	setbuf[1] = (uint8_t)(((rtc->Second/10)<<4)|(rtc->Second%10)|(0<<7));
	setbuf[2] = (uint8_t)(((rtc->Minute/10)<<4)|(rtc->Minute%10));
	setbuf[3] = (uint8_t)(((rtc->Hour/10)<<4)  |(rtc->Hour%10)|(0<<6));
	setbuf[4] = (uint8_t)(rtc->Week);
	setbuf[5] = (uint8_t)(((rtc->Day/10)<<4)|(rtc->Day%10));
	setbuf[6] = (uint8_t)(((rtc->Month/10)<<4)|(rtc->Month%10));
	setbuf[7] = (uint8_t)((((rtc->Year-1900-100)/10)<<4)|((rtc->Year-1900-100)%10));
	setbuf[8] = (uint8_t)(0);
	
	msg[0].frequency = CONFIG_DS1338_FREQUENCY;
	msg[0].addr      = CONFIG_DS1338_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = setbuf;
	msg[0].length    = 9;

	printf("setbuf:%x ,%x ,%x ,%x ,%x ,%x ,%x ,%x\n",setbuf[0],setbuf[1],setbuf[2],setbuf[3],\
													 setbuf[4],setbuf[5],setbuf[6],setbuf[7]);

	i2c_transfer(fd, msg, 1);
	return OK;
}

/******************************************************************************************************************************
函数名称：read_temp_high
功    能：读取温度高字节
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int read_ds1338_time (int fd,struct  TimeStruct *rtc)
{
	struct i2c_msg_s msg[2];
	
	msg[0].frequency = CONFIG_DS1338_FREQUENCY;
	msg[0].addr      = CONFIG_DS1338_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = (uint8_t *)&DS1338_SECOND_REG;
	msg[0].length    = 1;
		
	msg[1].frequency = CONFIG_DS1338_FREQUENCY;
	msg[1].addr      = CONFIG_DS1338_ADDR;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer 	 = rtc->buff;
	msg[1].length 	 = 7;
	
	i2c_transfer(fd,msg,2);

	rtc->Year	= ((rtc->buff[6]>>4)*10 + (rtc->buff[6]&0x0f)) + 1900 +100;
	rtc->Month	= ((rtc->buff[5]>>4)*10 + (rtc->buff[5]&0x0f));
	rtc->Day	= ((rtc->buff[4]>>4)*10 + (rtc->buff[4]&0x0f));
	rtc->Week	= (rtc->buff[3]&0x0f);
	rtc->Hour	= ((rtc->buff[2]>>4)*10 + (rtc->buff[2]&0x0f));
	rtc->Minute	= ((rtc->buff[1]>>4)*10 + (rtc->buff[1]&0x0f));
	rtc->Second	= ((rtc->buff[0]>>4)*10 + (rtc->buff[0]&0x0f));
	return OK;
}

/****************************************************************************
 * master_ds1338
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_ds1338(int argc, char *argv[])
{
	int ret;

	g_ds1338_started = true;

	fd_ds1338 = open(CONFIG_EXAMPLES_DS1338_DEVPATH, O_RDWR);
	if (fd_ds1338 < 0)
	{
		printf("master_ds1338: open %s failed: %d\n", CONFIG_EXAMPLES_DS1338_DEVPATH, errno);
		goto errout;
	}

	rtcds1338.Year		= 2017;
	rtcds1338.Month	= 11;
	rtcds1338.Day		= 20;
	rtcds1338.Hour		= 10;
	rtcds1338.Minute	= 25;
	rtcds1338.Second	= 30;
	rtcds1338.Week		= 0;
	
	init_ds1338_time(fd_ds1338,&rtcds1338);
	
	while(1)
	{
		read_ds1338_time(fd_ds1338,&rtcds1338);
		sleep(1);
		/*
		printf("rtcTime:%d:%02d:%02d:%02d:%02d:%02d\n",rtcds1338.Year,rtcds1338.Month,rtcds1338.Day,\
													   rtcds1338.Hour,rtcds1338.Minute,rtcds1338.Second);
		*/
	}
	
 errout:
  g_ds1338_started = false;

  printf("master_ds1338: Terminating\n");

  return EXIT_FAILURE;
}




