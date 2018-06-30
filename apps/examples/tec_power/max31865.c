#include "pwm.h"
#include "max31865.h"

char max31856_databuf[14];
char g_fault_status = 0;

/****************************************************************************
 * read_max31865
 * ��0��ַ������ȫ��������������ָ���Ĵ�����ַ��һ��14�ֽ�
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
 * buf�ĵ�һ���ֽ��ǼĴ�����ַ������Ϊ����
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

	//threshold ����ֵ
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
	//��·���
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MANUAL_FAULT_DETECT_CLOSE;
	write_max31865(fd,max31856_databuf,2);

	usleep(1000);
	
	//��·���
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MANUAL_FAULT_DETECT_OPEN;
	write_max31865(fd,max31856_databuf,2);
	
	//�ȴ���ɹ��ϼ��
	do
	{
		read_max31865(fd,max31856_databuf,1);
	}
	while((max31856_databuf[ADDR_CONFIGURATION]&0x0C)!=0x00);                

	//��ȡ����ֵ
	read_max31865(fd,max31856_databuf,sizeof(max31856_databuf));
	g_fault_status = max31856_databuf[ADDR_FAULT_STATUS];
	
	//�������
	max31856_databuf[0] = ADDR_CONFIGURATION;
	max31856_databuf[1] = MODE_AUTO_4WIRE | MANUAL_CLEAR_FAULT;
	write_max31865(fd,max31856_databuf,2);
}
