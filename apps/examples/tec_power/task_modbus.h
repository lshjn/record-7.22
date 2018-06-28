#ifndef _TASK_MODBUS_H
#define _TASK_MODBUS_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

enum modbus_threadstate_e
{
  STOPPED = 0,
  RUNNING,
  SHUTDOWN
};

struct modbus_state_s
{
  enum modbus_threadstate_e threadstate;
  uint16_t reginput[CONFIG_EXAMPLES_MODBUS_REG_INPUT_NREGS];
  uint16_t regholding[CONFIG_EXAMPLES_MODBUS_REG_HOLDING_NREGS];
  pthread_t threadid;
  pthread_mutex_t lock;
  volatile bool quit;
};


extern struct modbus_state_s g_modbus;


int master_modbus(int argc, char *argv[]);


#ifdef __cplusplus
}
#endif

#endif 
