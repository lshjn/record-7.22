#include "pid.h"
#include "pwm.h"
#include "max31865.h"
#include "task_monitor.h"
#include "task_modbus.h"


PID g_pid; 		//���PID�㷨����Ҫ������
PID g_pid_modbus; //���modbus���õ�����


void PID_Init(PID *pid)
{
	pid->I_MAX		= DEF_I_MAX;		//�û��趨������
	pid->Sv			= DEF_SV;			//�û��趨�¶�
	pid->T			= DEF_T;			//PID�������ڣ���������	
	pid->Kp			= DEF_KP;			//��������
	pid->Ti			= DEF_TI;			//����ʱ�䳣��
	pid->Td			= DEF_TD;			//΢��ʱ�䳣��
	pid->pwmcycle	= DEF_PWMCYCLE;		//pwm����1000
	pid->OUT0		= DEF_OUT0;			//ƫ����
}


void PID_Calc(PID *pid)  //pid����
{
	float DelEk;
	float ti,ki;
	float td;
	float kd;
	float out;
	

	pid->Ek=pid->Sv-pid->Pv;   	//�õ���ǰ��ƫ��ֵ
	pid->Pout=pid->Kp*pid->Ek;     //�������

	pid->SEk+=pid->Ek;        	//��ʷƫ���ܺ�

	DelEk=pid->Ek-pid->Ek_1;  	//�������ƫ��֮��

	ti=pid->T/pid->Ti;
	ki=ti*pid->Kp;

	pid->Iout=ki*pid->SEk*pid->Kp;  //�������

	td=pid->Td/pid->T;

	kd=pid->Kp*td;

	pid->Dout=kd*DelEk;    		//΢�����

	out= pid->Pout+ pid->Iout+ pid->Dout;

	//////////////////////////////////////////////////////////

	if(out>pid->pwmcycle)
	{
		pid->OUT=pid->pwmcycle;   //�������
	}
	else if(out<0)
	{
		pid->OUT=pid->OUT0;       //��С��
	}
	else 
	{
		pid->OUT=out;			//����pid������
	}
	
	pid->Ek_1=pid->Ek;  			//����ƫ��
}

//pid���
void PID_out(float pwm_value)  
{
	set_pwm(pwm_value);
}

//pidִ�й��磬�������ƴ���
void pid_exec(PID *pid)
{
	//��ǰ�ĵ��������趨���ֵ��pwm/(T/2)
	//���ε����ֵ,��Ҫȡpwm����-PID���ֵ,��Ϊpwm�ӵ�ʱ�¶��½�
	float PID_Out_PWM = 0;  

	PID_Out_PWM = pid->pwmcycle - pid->OUT;

	//��Ϊ��·û��ʹ��H�����Ե���û������,pwm����Ҫ���ɵ��߱ջ�,���ڿ����¶�ֱ������
	if(pid->Pv < pid->Sv)
	{
		PID_Out_PWM = 0;	
	}
	
	if(pid->I_CUR <= pid->I_MAX)
	{
	#if 0
		//pidȫ�����������̫�����ｫ���ֵ����һ��
		if(PID_Out_PWM > (pid->pwmcycle-200))
		{
			PID_Out_PWM = (pid->pwmcycle-200);
		}
	#endif
		PID_out(PID_Out_PWM);
		printf("pid->OUT=%.2f\n",pid->OUT);
		printf("PID_Out_PWM=%.2f\n",PID_Out_PWM);
	}
	else
	{
		//�������������ֵ�����һ����С��pwm
		PID_out(PID_Out_PWM/(pid->pwmcycle/2));	
		printf("pid->OUT=%.2f\n",pid->OUT);
		printf("PID_Out_PWM=%.2f\n",PID_Out_PWM/(pid->pwmcycle/2));
	}

}

//pid�����¶�
void pidctl_tecT(int fd,int dev_num,int *start)
{
	//��ȡ��ǰ�¶ȳɹ���ִ��pid�㷨����
	if(read_temper(fd,dev_num) == true)
	{
		//pid����
		PID_Calc(&g_pid);
		//pidִ�й���
		pid_exec(&g_pid);
		
		*start = false;
	}
}

