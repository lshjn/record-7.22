#ifndef _PID_H
#define _PID_H

#ifdef __cplusplus
 extern "C" {
#endif 
/****************************************************************************
 * Private Data
 ****************************************************************************/
	
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

typedef struct pid_t{
	float DC_I_MAX;		//�û��趨ֵ,pid������
	float DC_I_CUR_ADC;	//��鵽��ǰ��adc����ֵ

	
	float Sv;			//�û��趨ֵ
	float Pv;			//���������ֵ	

	float Kp;       	//������ϵ��
	uint32_t Ti;		//����ʱ�䳣��
	uint32_t Td; 		//΢��ʱ�䳣��
	
	uint32_t T;  		//PID��������--��������  ms

	float Ek;  			//����ƫ��
	float Ek_1;			//�ϴ�ƫ��
	float SEk; 			//��ʷƫ��֮��

	float Pout;			//���������
	float Iout;			//���������	
	float Dout;			//΢�������

	float OUT0;     	//��С��ƫ��

	float OUT;      	//���ε����ֵ

	uint16_t pwmcycle;	//pwm����
	
}PID;

extern PID pid; //���PID�㷨����Ҫ������

void PID_Calc(void); //pid����
void PID_Init(void);
void pidctl_tecT(int fd,int dev_num);
void pid_exec(void);
void PID_out(float pwm_value);



#ifdef __cplusplus
}
#endif

#endif 
