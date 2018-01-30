#ifndef _TASK_DS1338_H
#define _TASK_DS1338_H

#ifdef __cplusplus
 extern "C" {
#endif 
#include <nuttx/i2c/i2c_master.h>

struct  TimeStruct
{
    uint8_t  buff[7];         

    unsigned int   Year;          //��
    unsigned char  Month;         //��
    unsigned char  Day;           //��
    unsigned char  Hour;          //ʱ
    unsigned char  Minute;        //��
    unsigned char  Second;        //��
    unsigned char  Week;          //����
	
    unsigned long  NTPSecond;
};
extern struct TimeStruct rtcds1338;


void CalcWeek(struct TimeStruct *Time);
int init_ds1338_time  (int fd,struct  TimeStruct *rtc);
int read_ds1338_time (int fd,struct  TimeStruct *rtc);
int master_ds1338(int argc, char *argv[]);




#ifdef __cplusplus
}
#endif

#endif 
