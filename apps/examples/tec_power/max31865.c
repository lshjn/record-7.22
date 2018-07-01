#include "pwm.h"
#include "max31865.h"
#include "task_monitor.h"
#include "task_modbus.h"

char max31856_databuf[14];
char g_fault_status = 0;

/****************************************************************************
 * read_max31865
 * 从0地址将数据全部读出来，不用指定寄存器地址，一共14字节
 * liushuhe
 * 2018.06.29
 ****************************************************************************/
uint16_t read_max31865(int fd,char *buf,uint16_t length)  
{
	uint16_t  wBytes = 0;		

	wBytes = read(fd, buf, length);

	return wBytes;

}
/****************************************************************************
 * write_max31865
 * buf的第一个字节是寄存器地址，后面为数据
 * liushuhe
 * 2018.06.29
 ****************************************************************************/
uint16_t write_max31865(int fd,char *buf,uint16_t length)  
{
	uint16_t  wBytes = 0;		

	wBytes = write(fd,buf, length);

	return wBytes;

}

/****************************************************************************
 * start_conversion
 * liushuhe
 * 2018.06.29
 ****************************************************************************/
void start_conversion(int fd)  
{
	//start_conversion,set 1-shot=1
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MODE_MENUNAL_4WIRE | D5_SHOT_SET;
	write_max31865(fd,max31856_databuf,2);	
}

/****************************************************************************
 * Init_max31865
 * liushuhe
 * 2018.06.29
 ****************************************************************************/
uint16_t Init_max31865(int fd)  
{
	//manual  mode
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MODE_MENUNAL_4WIRE;
	write_max31865(fd,max31856_databuf,2);

	//threshold 门限值
	max31856_databuf[0] = ADDR_HIGHT_FAULT_MSB;
	max31856_databuf[1] = THRESHOLD_HIGHT_FAULT_MSB;
	max31856_databuf[2] = THRESHOLD_HIGHT_FAULT_LSB;
	max31856_databuf[3] = THRESHOLD_LOW_FAULT_MSB;
	max31856_databuf[4] = THRESHOLD_LOW_FAULT_LSB;
	write_max31865(fd,max31856_databuf,5);

	return true;
}
/****************************************************************************
 * Fault_Detect
 * liushuhe
 * 2018.06.29
 ****************************************************************************/
void Fault_Detect(int fd)
{
	//短路检测
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MANUAL_FAULT_DETECT_CLOSE;
	write_max31865(fd,max31856_databuf,2);

	usleep(1000);
	
	//开路检测
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MANUAL_FAULT_DETECT_OPEN;
	write_max31865(fd,max31856_databuf,2);
	
	//等待完成故障检测
	do
	{
		read_max31865(fd,max31856_databuf,1);
	}
	while((max31856_databuf[ADDR_CONFIGURATION]&0x0C)!=0x00);                

	//读取故障值
	read_max31865(fd,max31856_databuf,sizeof(max31856_databuf));
	g_fault_status = max31856_databuf[ADDR_FAULT_STATUS];
	
	//清除故障
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MODE_AUTO_4WIRE | MANUAL_CLEAR_FAULT;
	write_max31865(fd,max31856_databuf,2);
}


/****************************************************************************
 * read_temper
 * liushuhe
 * 2018.06.29
 ****************************************************************************/
void read_temper(int fd,int dev_num)
{
	float   AD_MAX81365 = 0;
	float   TEMPER_MAX81365 = 0;
	bool	DRDY_PIN_VALUE = DRDY_INVALID;
	
	Init_max31865(fd);
	memset(max31856_databuf,0,sizeof(max31856_databuf));

	start_conversion(fd);

	//wait max31865 drdy
	while(DRDY_PIN_VALUE == DRDY_INVALID)
	{
		if(dev_num == MAX31865_DEV1)
		{
			boardctl(BOARDIOC_GET_SPI1_DRDY, (uintptr_t)(&DRDY_PIN_VALUE));
		}
		else if(dev_num == MAX31865_DEV2)
		{
			boardctl(BOARDIOC_GET_SPI2_DRDY, (uintptr_t)(&DRDY_PIN_VALUE));
		}
		usleep(1000);
	}
	
	read_max31865(fd,max31856_databuf,sizeof(max31856_databuf));				//读取max31865当前的温度值

	if((max31856_databuf[ADDR_RTD_LSB]&0x01)==0x01)
	{
		Fault_Detect(fd);
	}
	else
	{
		AD_MAX81365 = ((max31856_databuf[ADDR_RTD_MSB]<<8)|max31856_databuf[ADDR_RTD_LSB])>>1;
		TEMPER_MAX81365 = ((AD_MAX81365/32)-256); 
		pid.Pv=TEMPER_MAX81365; 
		//设置modbus数据结构
		g_modbus.reginput[0] = pid.Pv;
	}
}

