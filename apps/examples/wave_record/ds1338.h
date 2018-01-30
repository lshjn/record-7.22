#ifndef _TASK_DS1338_H
#define _TASK_DS1338_H

#ifdef __cplusplus
 extern "C" {
#endif 
#include <nuttx/i2c/i2c_master.h>

struct  TimeStruct
{
    uint8_t  buff[7];         

    unsigned int   Year;          //年
    unsigned char  Month;         //月
    unsigned char  Day;           //日
    unsigned char  Hour;          //时
    unsigned char  Minute;        //分
    unsigned char  Second;        //秒
    unsigned char  Week;          //星期
	
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
