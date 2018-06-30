#ifndef _TASK_SPI_H
#define _TASK_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif 


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>


/****************************************************************************
 * Private Data
 ****************************************************************************/

void     read_temper(int fd,int dev_num);


int master_spi(int argc, char *argv[]);
 


#ifdef __cplusplus
}
#endif

#endif 
