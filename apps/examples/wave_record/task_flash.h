#ifndef _TASK_FLASH_H
#define _TASK_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif 


#define		VCC_MB_DEF			55	
#define		VCC_SB_DEF			55	
#define		TEMPRETURE_DEF		300	
#define		HUMIDITY_DEF		90	
#define		CO_DEF				70	
#define		H2S_DEF				70	
#define		NH3_DEF				70	
#define		O2_DEF				70	
#define		WATER_DEF			150	

struct alarm_value
{
	char msg[200];
	char write_flag;
	int vcc_mb;
	int vcc_sb;
	int tempretrue;
	int humidity;
	int co;
	int h2s;
	int nh3;
	int o2;
	int water;
	int user;
	int password;
	int id;
};


extern struct alarm_value  alarmdata;
int master_flash(int argc, char *argv[]);


#ifdef __cplusplus
}
#endif

#endif 
