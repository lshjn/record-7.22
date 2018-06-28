#include "pid.h"

PID pid; //存放PID算法所需要的数据


void PID_Init(void)
{
	pid.DC_I_MAX = 9;		//用户设定最大电流
	pid.Sv	= 120;			//用户设定温度
	pid.T	= 500;			//PID计算周期，采样周期	
	pid.Kp	= 30;			//比例常数
	pid.Ti	= 5000000;		//积分时间常数
	pid.Td	= 1000;			//微分时间常数
	pid.pwmcycle=1000;		//pwm周期1000
	pid.OUT0	=1;			//偏差项
}


void PID_Calc(void)  //pid计算
{
	float DelEk;
	float ti,ki;
	float td;
	float kd;
	float out;
	

	pid.Ek=pid.Sv-pid.Pv;   	//得到当前的偏差值
	pid.Pout=pid.Kp*pid.Ek;     //比例输出

	pid.SEk+=pid.Ek;        	//历史偏差总和

	DelEk=pid.Ek-pid.Ek_1;  	//最近两次偏差之差

	ti=pid.T/pid.Ti;
	ki=ti*pid.Kp;

	pid.Iout=ki*pid.SEk*pid.Kp;  //积分输出

	td=pid.Td/pid.T;

	kd=pid.Kp*td;

	pid.Dout=kd*DelEk;    		//微分输出

	out= pid.Pout+ pid.Iout+ pid.Dout;

	//////////////////////////////////////////////////////////

	if(out>pid.pwmcycle)
	{
		pid.OUT=pid.pwmcycle;   //最大周期
	}
	else if(out<0)
	{
		pid.OUT=pid.OUT0;       //最小量
	}
	else 
	{
		pid.OUT=out;			//本次pid计算量
	}
	
	pid.Ek_1=pid.Ek;  			//更新偏差
}

//pid输出
void PID_out(float pwm_value)  
{
	set_pwm(pwm_value);
}

//pid执行供电，并作限制处理
void pid_exec(void)
{
	//当前的电流超过设定最大值，pwm/2
	if(pid.DC_I_CUR_ADC <= pid.DC_I_MAX)
	{
		PID_out(pid.OUT);	
	}
	else
	{
		PID_out(pid.OUT/2);	
	}
}

//pid控制温度
void pidctl_tecT(void)
{
	//读取当前温度
	read_temper();
	//pid计算
	PID_Calc();
	//pid执行供电
	pid_exec();
}

