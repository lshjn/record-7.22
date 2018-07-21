#include "pid.h"
#include "pwm.h"

PID pid; 		//存放PID算法所需要的数据
PID pid_modbus; //存放modbus设置的数据


void PID_Init(void)
{
	pid.I_MAX		= 9.5;			//用户设定最大电流
	pid.Sv			= 120;			//用户设定温度
	pid.T			= 500;			//PID计算周期，采样周期	
	pid.Kp			= 30;			//比例常数
	pid.Ti			= 5000000;		//积分时间常数
	pid.Td			= 1000;			//微分时间常数
	pid.pwmcycle	= 1000;			//pwm周期1000
	pid.OUT0		= 1;			//偏差项
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
	//当前的电流超过设定最大值，pwm/(T/2)
	//本次的输出值,需要取pwm周期-PID输出值,因为pwm加电时温度下降
	float PID_Out_PWM = 0;  

	PID_Out_PWM = pid.pwmcycle - pid.OUT;

	//因为电路没有使用H桥所以导致没法加热,pwm环需要做成单边闭环,低于控制温度直接掐掉
	if(pid.Pv < pid.Sv)
	{
		PID_Out_PWM = 0;	
	}
	
	if(pid.I_CUR <= pid.I_MAX)
	{
		//pid全功率输出电流太大，这里将输出值限制一下
		if(PID_Out_PWM > (pid.pwmcycle-200))
		{
			PID_Out_PWM = (pid.pwmcycle-200);
		}
		PID_out(PID_Out_PWM);
		printf("pid.OUT=%.2f\n",pid.OUT);
		printf("PID_Out_PWM=%.2f\n",PID_Out_PWM);
	}
	else
	{
		//电流超过了最大值，输出一个很小的pwm
		PID_out(PID_Out_PWM/(pid.pwmcycle/2));	
		printf("pid.OUT=%.2f\n",pid.OUT);
		printf("PID_Out_PWM=%.2f\n",PID_Out_PWM/(pid.pwmcycle/2));
	}

}

//pid控制温度
void pidctl_tecT(int fd,int dev_num,int *start)
{
	//读取当前温度
	read_temper(fd,dev_num);
	//pid计算
	PID_Calc();
	//pid执行供电
	pid_exec();
	
	*start = false;
	
}

