#ifndef _TASK_GPRS_H
#define _TASK_GPRS_H

#ifdef __cplusplus
 extern "C" {
#endif 

//msgack
#define		WAIT		0
#define		INIT		1
#define		REGOK		2
#define		CONNECT		3
#define		START		4
#define		RCV_OK		5


//set state
#define		FAIL		0
#define		SUCCESS		1

//lock state
#define		ENABLE		1
#define		DISABLE		0
//ack
#define		NOACK				0
#define		ACK					1

struct	ms_data
{
	char time[200];
	char locker;
	char vcc;
    char tempretrue;
	char humidity;
	char co;
	char h2s;
	char nh3;
	char o2;
	char water;
	char user[20];
	char password[20];
	char id;
};


struct  gprs_data
{
	char  set_slavetime_flag;
	char  InitOK;
	char  process_state;
	char  msgack;
	struct	ms_data download_data;
	struct	ms_data upload_data;
	char  msgbuf[255];
	char  msglen;
};

extern struct 	adc_msg		AdcData;
extern struct	gprs_data	GprsData;



int   master_gprs(int argc, char *argv[]);
int   gprs_warn_upload(int fd,struct gprs_data *gprs,struct adc_msg *adcdada);
int   gprs_timeint_upload(int fd,struct gprs_data *gprs,struct adc_msg *adcdada);
int   gprs_openlock(int fd,struct gprs_data *gprs,struct adc_msg *adcdada);
int   set_sconType(int fd,struct	gprs_data *gprs);
int   set_apn(int fd,struct	gprs_data *gprs);
int   set_srvType(int fd,struct	gprs_data *gprs);
int   set_conId(int fd,struct	gprs_data *gprs);
int   set_address(int fd,struct	gprs_data *gprs);
int   into_establish(int fd,struct	gprs_data *gprs);
int   into_transparent(int fd,struct	gprs_data *gprs);
int   gprs_tcpinit(int fd,struct gprs_data *initgprs);
int   gprs_register(int fd,struct	gprs_data *gprs);
int   gprs_rst(int fd,struct	gprs_data *gprs);



#ifdef __cplusplus
}
#endif

#endif 
