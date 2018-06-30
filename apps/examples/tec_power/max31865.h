#ifndef _MAX31865_H
#define _MAX31865_H

#ifdef __cplusplus
 extern "C" {
#endif 
/****************************************************************************
 * Private Data
 ****************************************************************************/
	
#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>

#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/timers/timer.h>
#include <termios.h>


#include <nuttx/drivers/pwm.h>

//REG  ADDR
#define  ADDR_CONFIGURATION		0
#define  ADDR_RTD_MSB			1
#define  ADDR_RTD_LSB			2
#define  ADDR_HIGHT_FAULT_MSB	3
#define  ADDR_HIGHT_FAULT_LSB	4
#define  ADDR_LOW_FAULT_MSB		5
#define  ADDR_LOW_FAULT_LSB		6
#define  ADDR_FAULT_STATUS		7
//CMD
#define  MODE_AUTO_4WIRE				0xC1
#define  MODE_MENUNAL_4WIRE				0x81
#define  MANUAL_FAULT_DETECT_CLOSE		0x88
#define  MANUAL_FAULT_DETECT_OPEN		0x8c
#define  MANUAL_CLEAR_FAULT				(1<<2)
#define  D5_SHOT_SET					(1<<5)

//THRESHOLD
#define  THRESHOLD_HIGHT_FAULT_MSB		0xFF
#define  THRESHOLD_HIGHT_FAULT_LSB		0xFF
#define  THRESHOLD_LOW_FAULT_MSB		0x00
#define  THRESHOLD_LOW_FAULT_LSB		0x00
//
#define  DRDY_VALID						0
#define  DRDY_INVALID					1

extern char max31856_databuf[14];
extern char g_fault_status;

uint16_t Init_max31865(int fd) ;
uint16_t write_max31865(int fd,char *buf,uint16_t length);  
uint16_t read_max31865(int fd,char *buf,uint16_t length)  ;
void start_conversion(int fd) ;
void Fault_Detect(int fd);

#ifdef __cplusplus
}
#endif

#endif 
