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
			

#define		DEF_I_MAX		9.5			//用户设定最大电流
#define		DEF_SV			17			//用户设定温度
#define		DEF_T			1000		//PID计算周期，采样周期 
#define		DEF_KP			30			//比例常数
#define		DEF_TI			5000000		//积分时间常数
#define		DEF_TD			500			//微分时间常数
#define		DEF_PWMCYCLE	1000		//pwm周期1000
#define		DEF_OUT0		1			//偏差项

typedef struct pid_t{
	float I_MAX;		//用户设定值,pid最大电流
	float I_CUR;		//检查到当前的adc电流值

	
	float Sv;			//用户设定值
	float Pv;			//传感器检测值	

	float Kp;       	//比例项系数
	float Ti;			//积分时间常数
	float Td; 			//微分时间常数
	
	float T;  			//PID计算周期--采样周期  ms

	float Ek;  			//本次偏差
	float Ek_1;			//上次偏差
	float SEk; 			//历史偏差之和

	float Pout;			//比例项输出
	float Iout;			//积分项输出	
	float Dout;			//微分项输出

	float OUT0;     	//很小的偏差

	float OUT;      	//本次的输出值

	float pwmcycle;	//pwm周期
	
}PID;

extern PID g_pid; 		//存放PID算法所需要的数据
extern PID g_pid_modbus;  //存放modbus设置的数据

void PID_Calc(PID *pid); //pid计算
void PID_Init(PID *pid);
void pidctl_tecT(int fd,int dev_num,int *start);
void pid_exec(PID *pid);
void PID_out(float pwm_value);



#ifdef __cplusplus
}
#endif

#endif 
