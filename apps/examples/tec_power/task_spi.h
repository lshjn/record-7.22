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
void read_temper(void);
uint16_t read_max31865(void);


int master_spi(int argc, char *argv[]);
 


#ifdef __cplusplus
}
#endif

#endif 
