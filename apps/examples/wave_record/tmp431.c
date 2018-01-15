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

int TMP431_CONVERSION_RATE_WRITE =0x0A;
int TMP431_CONVERSION_CHEL_WRITE =0x1A;
int TMP431_LOCALTEMP_HIGH	=0x00;
int TMP431_LOCALTEMP_LOW	=0x15;
int TMP431_CONFIG_WRITE	=0x09;
int TMP431_CONFIG_READ 	=0x03;
int I2C_M_WRIRE   			=0x00; /* write data, master to slave*/

struct tmp431_data 
{
	char temp_low[2];
	char temp_high[2];
	float  temp_show;
};

struct tmp431_data tmp431;

static bool  g_tmp431_started;
static	int  fd_tmp431;


/******************************************************************************************************************************
函数名称：set_rate
功    能：设置转换频率 2/s
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int set_rate (int fd)
{
	struct i2c_msg_s msg[1];
	uint8_t setbuf[2];
	setbuf[0] = (uint8_t)TMP431_CONVERSION_RATE_WRITE;
	setbuf[1] = (uint8_t)5;
	
	msg[0].frequency = CONFIG_TMP431_FREQUENCY;
	msg[0].addr      = CONFIG_TMP431_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = setbuf;
	msg[0].length    = 2;
	
	i2c_transfer(fd, msg, 1);
	return OK;
}

/******************************************************************************************************************************
函数名称：set_tempe_range
功    能：设置温度范围  -55°C to 150
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int set_tempe_range  (int fd)
{
	struct i2c_msg_s msg[1];

	uint8_t setbuf[2];
	setbuf[0] = (uint8_t)TMP431_CONFIG_WRITE;
	setbuf[1] = (uint8_t)((1<<2)|(0<<5) | (0<<6) |(0<<7));
	
	msg[0].frequency = CONFIG_TMP431_FREQUENCY;
	msg[0].addr      = CONFIG_TMP431_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = setbuf;
	msg[0].length    = 2;
		
	i2c_transfer(fd, msg, 1);
	return OK;
}

/******************************************************************************************************************************
函数名称：en_local_chanel
功    能：使能本地通道
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int en_local_chanel  (int fd)
{
	struct i2c_msg_s msg[1];
	uint8_t setbuf[2];
	setbuf[0] = (uint8_t)TMP431_CONVERSION_CHEL_WRITE;
	setbuf[1] = (uint8_t)((1<<2) | (1<<3) | (0<<4) |(0<<5));
	
	msg[0].frequency = CONFIG_TMP431_FREQUENCY;
	msg[0].addr      = CONFIG_TMP431_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = setbuf;
	msg[0].length    = 2;

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
int read_temp_high  (int fd)
{
	struct i2c_msg_s msg[2];
	union
	{
		uint16_t data16;
		uint8_t  data8;
	}set_val;
	
	msg[0].frequency = CONFIG_TMP431_FREQUENCY;
	msg[0].addr      = CONFIG_TMP431_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = (uint8_t *)&TMP431_LOCALTEMP_HIGH;
	msg[0].length    = 1;
		
	msg[1].frequency = CONFIG_TMP431_FREQUENCY;
	msg[1].addr      = CONFIG_TMP431_ADDR;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer 	 = &set_val.data8;
	msg[1].length 	 = 1;

	i2c_transfer(fd,msg,2);

	tmp431.temp_high[0] = set_val.data8;
	return OK;
}

/******************************************************************************************************************************
函数名称：read_temp_low
功    能：读取温度低字节
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int read_temp_low  (int fd)
{
	struct i2c_msg_s msg[2];
	union
	{
		uint16_t data16;
		uint8_t  data8;
	}set_val;
	
	msg[0].frequency = CONFIG_TMP431_FREQUENCY;
	msg[0].addr      = CONFIG_TMP431_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = (uint8_t *)&TMP431_LOCALTEMP_LOW;
	msg[0].length    = 1;
		
	msg[1].frequency = CONFIG_TMP431_FREQUENCY;
	msg[1].addr      = CONFIG_TMP431_ADDR;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer 	 = &set_val.data8;
	msg[1].length 	 = 1;

	i2c_transfer(fd,msg,2);

	tmp431.temp_low[0] = set_val.data8;
	return OK;
}

/******************************************************************************************************************************
函数名称：read_temp_range
功    能：读取温度范围
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int read_temp_range  (int fd)
{
	struct i2c_msg_s msg[2];
	union
	{
		uint16_t data16;
		uint8_t  data8;
	}set_val;
	
	msg[0].frequency = CONFIG_TMP431_FREQUENCY;
	msg[0].addr      = CONFIG_TMP431_ADDR;
	msg[0].flags     = I2C_M_WRIRE;
	msg[0].buffer    = (uint8_t *)&TMP431_CONFIG_READ;
	msg[0].length    = 1;
		
	msg[1].frequency = CONFIG_TMP431_FREQUENCY;
	msg[1].addr      = CONFIG_TMP431_ADDR;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer 	 = &set_val.data8;
	msg[1].length 	 = 1;
	i2c_transfer(fd,msg,2);
	return OK;
}


/******************************************************************************************************************************
函数名称：get_tempvalue
功    能：读取温度
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int get_tempvalue  (int fd)
{
	read_temp_high(fd);
	read_temp_low(fd);
	read_temp_range(fd);
	tmp431.temp_show = ((tmp431.temp_high[0]-64) + (tmp431.temp_low[0]>>4)*0.0625);
	return OK;
}

/******************************************************************************************************************************
函数名称：tmp431_init
功    能：读取温度
输入参数：
输入参数：
编写时间：2017.11.17
编 写 人：liushuhe
*******************************************************************************************************************************/
int tmp431_init(int fd)
{
	set_rate(fd);
	usleep(20*1000);
	set_tempe_range(fd);
	usleep(20*1000);
	en_local_chanel(fd);	
	usleep(20*1000);
	return OK;
}

/****************************************************************************
 * Name: i2cdev_transfer
 ****************************************************************************/
int i2c_transfer(int fd, FAR struct i2c_msg_s *msgv, int msgc)
{
  struct i2c_transfer_s xfer;

  /* Set up the IOCTL argument */

  xfer.msgv = msgv;
  xfer.msgc = msgc;
  /* Perform the IOCTL */

  return ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&xfer));
}


/****************************************************************************
 * slave_adc
 * liushuhe
 * 2017.09.26
 ****************************************************************************/
int master_tmp431(int argc, char *argv[])
{
	g_tmp431_started = true;

	fd_tmp431 = open(CONFIG_EXAMPLES_TMP431_DEVPATH, O_RDWR);
	if (fd_tmp431 < 0)
	{
		printf("master_tmp431: open %s failed: %d\n", CONFIG_EXAMPLES_TMP431_DEVPATH, errno);
		goto errout;
	}
	
	tmp431_init(fd_tmp431);
	
	while(1)
	{
		get_tempvalue(fd_tmp431);
		sleep(1);
		//printf("temp_show = %.3f\n",tmp431.temp_show);
	}
	
 errout:
  g_tmp431_started = false;

  printf("master_tmp431: Terminating\n");

  return EXIT_FAILURE;
}




