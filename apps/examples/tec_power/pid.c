#include "pid.h"
#include "pwm.h"

PID pid; 		//���PID�㷨����Ҫ������
PID pid_modbus; //���modbus���õ�����


void PID_Init(void)
{
	pid.I_MAX		= 9.5;			//�û��趨������
	pid.Sv			= 120;			//�û��趨�¶�
	pid.T			= 500;			//PID�������ڣ���������	
	pid.Kp			= 30;			//��������
	pid.Ti			= 5000000;		//����ʱ�䳣��
	pid.Td			= 1000;			//΢��ʱ�䳣��
	pid.pwmcycle	= 1000;			//pwm����1000
	pid.OUT0		= 1;			//ƫ����
}


void PID_Calc(void)  //pid����
{
	float DelEk;
	float ti,ki;
	float td;
	float kd;
	float out;
	

	pid.Ek=pid.Sv-pid.Pv;   	//�õ���ǰ��ƫ��ֵ
	pid.Pout=pid.Kp*pid.Ek;     //�������

	pid.SEk+=pid.Ek;        	//��ʷƫ���ܺ�

	DelEk=pid.Ek-pid.Ek_1;  	//�������ƫ��֮��

	ti=pid.T/pid.Ti;
	ki=ti*pid.Kp;

	pid.Iout=ki*pid.SEk*pid.Kp;  //�������

	td=pid.Td/pid.T;

	kd=pid.Kp*td;

	pid.Dout=kd*DelEk;    		//΢�����

	out= pid.Pout+ pid.Iout+ pid.Dout;

	//////////////////////////////////////////////////////////

	if(out>pid.pwmcycle)
	{
		pid.OUT=pid.pwmcycle;   //�������
	}
	else if(out<0)
	{
		pid.OUT=pid.OUT0;       //��С��
	}
	else 
	{
		pid.OUT=out;			//����pid������
	}
	
	pid.Ek_1=pid.Ek;  			//����ƫ��
}

//pid���
void PID_out(float pwm_value)  
{
	set_pwm(pwm_value);
}

//pidִ�й��磬�������ƴ���
void pid_exec(void)
{
	//��ǰ�ĵ��������趨���ֵ��pwm/(T/2)
	//���ε����ֵ,��Ҫȡpwm����-PID���ֵ,��Ϊpwm�ӵ�ʱ�¶��½�
	float PID_Out_PWM = 0;  

	PID_Out_PWM = pid.pwmcycle - pid.OUT;

	//��Ϊ��·û��ʹ��H�����Ե���û������,pwm����Ҫ���ɵ��߱ջ�,���ڿ����¶�ֱ������
	if(pid.Pv < pid.Sv)
	{
		PID_Out_PWM = 0;	
	}
	
	if(pid.I_CUR <= pid.I_MAX)
	{
		//pidȫ�����������̫�����ｫ���ֵ����һ��
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
		//�������������ֵ�����һ����С��pwm
		PID_out(PID_Out_PWM/(pid.pwmcycle/2));	
		printf("pid.OUT=%.2f\n",pid.OUT);
		printf("PID_Out_PWM=%.2f\n",PID_Out_PWM/(pid.pwmcycle/2));
	}

}

//pid�����¶�
void pidctl_tecT(int fd,int dev_num,int *start)
{
	//��ȡ��ǰ�¶�
	read_temper(fd,dev_num);
	//pid����
	PID_Calc();
	//pidִ�й���
	pid_exec();
	
	*start = false;
	
}

