#ifndef _TASK_CC1101_H
#define _TASK_CC1101_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define ALIGN __attribute__((packed))

struct cc110x_msg1{
	uint8_t cmd;
	uint8_t online_A;
	uint8_t online_B;
	uint8_t online_C;


	//u8_t CheckSum;
	uint8_t Ack;
	uint32_t  TimerCnt;
	uint32_t  SysTitk;
	
   	uint8_t cur_balladdr;										//��ǰ�����Ĳɼ���ĵ�ַ���
 
	uint8_t src;
	uint8_t dist;
	
	uint32_t  GetTime;
	uint32_t  SetTime;
	
};


typedef struct ALIGN timemsg{
	uint8_t  start_flag;
	uint8_t  msglen;
	uint8_t  type;
	uint8_t  dist;
	uint8_t  src;
	uint32_t second; 		//��ʱ���ж��ۼ�ֵ
	uint32_t us;            //��ʱ����cnt
	uint8_t  endflag;        
}cc110x_timemsg;


int master_cc1101(int argc, char *argv[]);



#ifdef __cplusplus
}
#endif

#endif 