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
			

#define		DEF_I_MAX		9.5			//�û��趨������
#define		DEF_SV			17			//�û��趨�¶�
#define		DEF_T			1000		//PID�������ڣ��������� 
#define		DEF_KP			30			//��������
#define		DEF_TI			5000000		//����ʱ�䳣��
#define		DEF_TD			500			//΢��ʱ�䳣��
#define		DEF_PWMCYCLE	1000		//pwm����1000
#define		DEF_OUT0		1			//ƫ����

typedef struct pid_t{
	float I_MAX;		//�û��趨ֵ,pid������
	float I_CUR;		//��鵽��ǰ��adc����ֵ

	
	float Sv;			//�û��趨ֵ
	float Pv;			//���������ֵ	

	float Kp;       	//������ϵ��
	float Ti;			//����ʱ�䳣��
	float Td; 			//΢��ʱ�䳣��
	
	float T;  			//PID��������--��������  ms

	float Ek;  			//����ƫ��
	float Ek_1;			//�ϴ�ƫ��
	float SEk; 			//��ʷƫ��֮��

	float Pout;			//���������
	float Iout;			//���������	
	float Dout;			//΢�������

	float OUT0;     	//��С��ƫ��

	float OUT;      	//���ε����ֵ

	float pwmcycle;	//pwm����
	
}PID;

extern PID g_pid; 		//���PID�㷨����Ҫ������
extern PID g_pid_modbus;  //���modbus���õ�����

void PID_Calc(PID *pid); //pid����
void PID_Init(PID *pid);
void pidctl_tecT(int fd,int dev_num,int *start);
void pid_exec(PID *pid);
void PID_out(float pwm_value);



#ifdef __cplusplus
}
#endif

#endif 
