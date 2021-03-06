#ifndef _TASK_FLASH_H
#define _TASK_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include "pid.h"

#define  CMD_PID_I_MAX			0
#define  CMD_PID_SV				1
#define  CMD_PID_T				2
#define  CMD_PID_KP				3
#define  CMD_PID_TI				4
#define  CMD_PID_TD				5
#define  CMD_PID_PWMCYC			6
#define  CMD_PID_OUT0			7

extern pthread_mutex_t g_FlashMutex	;
extern pthread_cond_t  g_FlashConVar;

extern bool	enFlashUpdata;

int master_flash(int argc, char *argv[]);
int readdata(const char *path,char  *buf,uint32_t len);
int writedata(const char *path,char  *buf,uint32_t len);
char updata_pidarg_fromflash(PID *pid);
int updata_pidarg_modbusToflash(PID *pid,int cmd_index);
void  EnFlashUpdata(pthread_cond_t *cond,pthread_mutex_t *mutex);


#ifdef __cplusplus
}
#endif

#endif 
