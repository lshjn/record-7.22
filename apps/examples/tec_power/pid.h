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
	float DC_I_MAX;		//用户设定值,pid最大电流
	float DC_I_CUR_ADC;	//检查到当前的adc电流值

	
	float Sv;		//用户设定值
	float Pv;		//传感器检测值	

	float Kp;       //比例项系数
	float Ti;		//积分时间常数
	float Td; 		//微分时间常数
	
	float T;  		//PID计算周期--采样周期

	float Ek;  		//本次偏差
	float Ek_1;		//上次偏差
	float SEk; 		//历史偏差之和

	float Pout;		//比例项输出
	float Iout;		//积分项输出	
	float Dout;		//微分项输出

	float OUT0;     //很小的偏差

	float OUT;      //本次的输出值

	uint16_t pwmcycle;	//pwm周期
	
}PID;

extern PID pid; //存放PID算法所需要的数据

void PID_Calc(void); //pid计算
void PID_Init(void);
void pidctl_tecT(int fd,int dev_num);
void pid_exec(void);
void PID_out(float pwm_value);



#ifdef __cplusplus
}
#endif

#endif 
