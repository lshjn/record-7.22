/****************************************************************************
 * configs/nucleo-l476rg/src/stm32l4_appinit.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>

#include <arch/board/board.h>
//add by liushuhe 2017.09.13
#include "nucleo-l476rg.h"
//add by liushuhe 2017.09.13
#include <nuttx/mtd/mtd.h>
#include "stm32l4_i2c.h"
//add by liushuhe 2017.09.13
#include "stm32l476rg_gpioint.h"

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif



/****************************************************************************
 * Public Functions
 ****************************************************************************/
//add by liushuhe 2017.09.01
#define LED_DRIVER_PATH  "/dev/userleds"


/****************************************************************************
 * Private Data
 ***************************************************************************/
//add by liushuhe 2017.09.15
#if defined(CONFIG_STM32L4_I2C1)
struct i2c_master_s* i2c1;
#endif
#if defined(CONFIG_STM32L4_I2C2)
struct i2c_master_s* i2c2;
#endif
#if defined(CONFIG_STM32L4_I2C3)
struct i2c_master_s* i2c3;
#endif


//add by liushuhe 2017.09.16
void stm32l4_gpioinit(void);
void AM2320_Start(void);
void AM2320_Stop(void);
void Ack(void);
void NoAck(void);
char Test_Ack(void);

void AM2320_SendByte(uint8_t dat);
uint8_t AM2320_RecvByte(void);
int AM2320_Multiple_read(struct  i2c_msg_s * msg);
int AM2320_Multiple_Write(struct  i2c_msg_s * msg);
void AM2320_WakeUp(char SlaveAddress);
/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Dummy function expected to start-up logic.
 *
 ****************************************************************************/

#ifdef CONFIG_WL_CC3000
void up_netinitialize(void)
{
}
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *rtclower;
#endif
#ifdef CONFIG_SENSORS_QENCODER
  int index;
  char buf[9];
#endif
  int ret;

  (void)ret;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

//add by liushuhe 2017.10.09
//注册gpio设备，外部中断设备
	stm32l4_gpiodev_initialize();

  //add by liushuhe 2017.08.31
  /*
  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    } 
  */

//add by liushuhe 2017.09.15

#if defined(CONFIG_I2C)
  /* Configure I2C */

  /* REVISIT: this is ugly! */

#if defined(CONFIG_STM32L4_I2C1)
  i2c1 = stm32l4_i2cbus_initialize(1);
#endif

//现在用的是adc2
#if defined(CONFIG_STM32L4_I2C2)
  i2c2 = stm32l4_i2cbus_initialize(2);
#endif

#if defined(CONFIG_STM32L4_I2C3)
  i2c3 = stm32l4_i2cbus_initialize(3);
#endif

#if defined(CONFIG_STM32L4_I2C1)
  i2c_register(i2c1, 1);
#endif
#if defined(CONFIG_STM32L4_I2C2)
  i2c_register(i2c2, 2);
#endif
#if defined(CONFIG_STM32L4_I2C3)
  i2c_register(i2c3, 3);
#endif
#endif /* CONFIG_I2C */


#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32L4 lower-half RTC driver */

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */
       
	  //add by liushue  2017.09.04
      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }

	}

#endif



//watch
//add by liushuhe 2017.10.25
#ifdef CONFIG_STM32L4_IWDG
  /* Initialize the watchdog timer */

  stm32l4_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif

#ifdef CONFIG_STM32L476_WDG
  /* Start WDG kicker thread */

  ret = STM32L476_watchdog_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to start watchdog thread: %d\n", ret);
      return ret;
    }
#endif





#ifdef HAVE_MMCSD
  /* First, get an instance of the SDIO interface */

  g_sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!g_sdio)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_sdio);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  /* Then let's guess and say that there is a card in the slot. There is no
   * card detect GPIO.
   */

  sdio_mediachange(g_sdio, true);

  syslog(LOG_INFO, "[boot] Initialized SDIO\n");
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32l4_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_pwm_setup() failed: %d\n", ret);
    }
#endif

  /*mtd  at24c512 add by liushuhe 2017.09.13*/

	//ret = progmem_initialize();
	if (ret < 0)
	{
		syslog(LOG_ERR, "ERROR: my_at24config() failed: %d\n", ret);
	}

	//ret = my_at24config();
	if (ret < 0)
	{
		syslog(LOG_ERR, "ERROR: my_at24config() failed: %d\n", ret);
	}



#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */
  /*
  //add by liushuhe 2017.09.08
  ret = stm32l4_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_adc_setup failed: %d\n", ret);
    }
  */
  
  //add by liushuhe 2017.09.08
  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }  
  
#endif


#ifdef CONFIG_AJOYSTICK
  /* Initialize and register the joystick driver */

  ret = board_ajoy_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the joystick driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

  ret = board_timer_driver_initialize("/dev/timer0", 2);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER

  /* Initialize and register the qencoder driver */

  index = 0;

#ifdef CONFIG_STM32L4_TIM1_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 1);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM2_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 2);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM3_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 3);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM4_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 4);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM5_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 5);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM8_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 8);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#endif



  UNUSED(ret);
  return OK;
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
	//add by liushuhe  2017.09.04
	switch(cmd)
	{
		/*********************************************************/
		//master
        //bluetooth   		
		case RECORD_BLUEDEV_GPIOINIT:
				stm32l4_configgpio(RESET_BLUEDEV_BT_PWR_CTL);
				stm32l4_configgpio(RESET_BLUEEV_BT_RDY);
				stm32l4_configgpio(RESET_BLUEDEV_BT_WAKEUP);
				break;
		case RECORD_BLUEDEV_POWER_DISABLE:
				stm32l4_gpiowrite(RESET_BLUEDEV_BT_PWR_CTL,false);
				break;
		case RECORD_BLUEDEV_POWER_ENABLE:
				stm32l4_gpiowrite(RESET_BLUEDEV_BT_PWR_CTL,true);
				break;
		case RECORD_BLUEDEV_WAKEUP_ENABLE:
				stm32l4_gpiowrite(RESET_BLUEDEV_BT_WAKEUP,false);
				break;
		case RECORD_BLUEDEV_WAKEUP_DISABLE:
				stm32l4_gpiowrite(RESET_BLUEDEV_BT_WAKEUP,true);
				break;
		//init gpio 
		case RECORD_LED_GPIOINIT:
				stm32l4_configgpio(LED_WAKEUP);
				stm32l4_configgpio(LED_DEBUG);
				stm32l4_configgpio(RESET_GPIO_LED_GPS);
				stm32l4_configgpio(RESET_GPIO_LED_433M);
				break;
		//hall sensor
		case HALL_SENSOR_GPIOINIT:
				stm32l4_configgpio(HALL_SENSOR1);
				stm32l4_configgpio(HALL_SENSOR2);
				break;
	    //pwm gpio
		case PWM_GPIOINIT:
				stm32l4_configgpio(MOTOR_EN);
				stm32l4_configgpio(MOTOR_DIR);
				//pwm disable
				stm32l4_gpiowrite(MOTOR_EN,true);
				//lock close
				stm32l4_gpiowrite(MOTOR_DIR,false);
				break;
		//en EINTR
		case MOTOR_EN_ENABLE:
				stm32l4_gpiowrite(MOTOR_EN,false);
				break;
		case MOTOR_EN_DISABLE:
				stm32l4_gpiowrite(MOTOR_EN,true);
				break;
		//pwm dir		
		case MOTOR_DIR_OPEN:
				stm32l4_gpiowrite(MOTOR_DIR,true);
				break;
		case MOTOR_DIR_CLOSE:
				stm32l4_gpiowrite(MOTOR_DIR,false);
				break;

		
		//led
		case LED_WAKEUP_ENABLE:
				stm32l4_gpiowrite(LED_WAKEUP,false);
				break;
		case LED_WAKEUP_DISABLE:
				stm32l4_gpiowrite(LED_WAKEUP,true);
				break;
		case LED_DEBUG_ENABLE:
				stm32l4_gpiowrite(LED_DEBUG,false);
				break;
		case LED_DEBUG_DISABLE:
				stm32l4_gpiowrite(LED_DEBUG,true);
				break;
		case RECORD_GPS_RESET_ENABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED_GPS,false);
				break;
		case RECORD_GPS_RESET_DISABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED_GPS,true);
				break;
		case RECORD_433M_RESET_ENABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED_433M,false);
				break;
		case RECORD_433M_RESET_DISABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED_433M,true);
				break;
		//relay
		case RECORD_RELAY_GPIOINIT:
				stm32l4_configgpio(RESET_GPIO_RELAY);
				break;
		case RECORD_RELAY_RESET_ENABLE:
				stm32l4_gpiowrite(RESET_GPIO_RELAY,false);
				break;
		case RECORD_RELAY_RESET_DISABLE:
				stm32l4_gpiowrite(RESET_GPIO_RELAY,true);
				break;
		/*********************************************************/
		//slave
		case SLAVE_LED_GPIOINIT:
				stm32l4_configgpio(RESET_GPIO_LED2);
				stm32l4_configgpio(RESET_GPIO_LED3);
				stm32l4_configgpio(RESET_GPIO_LED4);
				break;
		case SLAVE_RELAY_GPIOINIT:
				stm32l4_configgpio(RESET_RELAY_LSENSOR);
				stm32l4_configgpio(RESET_RELAY_PUMP);
				stm32l4_configgpio(RESET_RELAY_GASIN);
				stm32l4_configgpio(RESET_RELAY_GASOUT);
				stm32l4_configgpio(RESET_RELAY_MAINBD);
				stm32l4_configgpio(RESET_SYS_WAKEUP);
				break;
		//led		
		case SLAVE_DEBUG_LED2_RESET_ENABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED2,false);
				break;
		case SLAVE_DEBUG_LED2_RESET_DISABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED2,true);
				break;
		case SLAVE_DEBUG_LED3_RESET_ENABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED3,false);
				break;
		case SLAVE_DEBUG_LED3_RESET_DISABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED3,true);
				break;
		case SLAVE_DEBUG_LED4_RESET_ENABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED4,false);
				break;
		case SLAVE_DEBUG_LED4_RESET_DISABLE:
				stm32l4_gpiowrite(RESET_GPIO_LED4,true);
				break;
        //relay
		case SLAVE_RELAY_LSENSOR_ENABLE:
				stm32l4_gpiowrite(RESET_RELAY_LSENSOR,false);
				break;
		case SLAVE_RELAY_LSENSOR_DISABLE:
				stm32l4_gpiowrite(RESET_RELAY_LSENSOR,true);
				break;
		case SLAVE_RELAY_PUMP_ENABLE:
				stm32l4_gpiowrite(RESET_RELAY_PUMP,false);
				break;
		case SLAVE_RELAY_PUMP_DISABLE:
				stm32l4_gpiowrite(RESET_RELAY_PUMP,true);
				break;
		case SLAVE_RELAY_GASIN_ENABLE:
				stm32l4_gpiowrite(RESET_RELAY_GASIN,false);
				break;
		case SLAVE_RELAY_GASIN_DISABLE:
				stm32l4_gpiowrite(RESET_RELAY_GASIN,true);
				break;
		case SLAVE_RELAY_GASOUT_ENABLE:
				stm32l4_gpiowrite(RESET_RELAY_GASOUT,false);
				break;
		case SLAVE_RELAY_GASOUT_DISABLE:
				stm32l4_gpiowrite(RESET_RELAY_GASOUT,true);
				break;
		case SLAVE_RELAY_MAINBD_ENABLE:
				stm32l4_gpiowrite(RESET_RELAY_MAINBD,false);
				break;
		case SLAVE_RELAY_MAINBD_DISABLE:
				stm32l4_gpiowrite(RESET_RELAY_MAINBD,true);
				break;
		/*
		case SLAVE_RELAY_SYS_WAKEUP_ENABLE:
				stm32l4_gpiowrite(RESET_SYS_WAKEUP,false);
				break;
		case SLAVE_RELAY_SYS_WAKEUP_DISABLE:
				stm32l4_gpiowrite(RESET_SYS_WAKEUP,true);
				break;
		*/
		/*********************************************************/
		//wakeup gprs
		//add by liushue 2017.09.17
		
		case GPRS_PWRON:
		  	{
				 stm32l4_configgpio(GPRS_PWR_ONOFF);
				 stm32l4_gpiowrite(GPRS_PWR_ONOFF,true);
			}
		    break;
		case GPRS_PWROFF:
		  	{
				 stm32l4_configgpio(GPRS_PWR_ONOFF);
				 stm32l4_gpiowrite(GPRS_PWR_ONOFF,false);
			}
		    break;
		/************************************************/
		//正
		case GPRS_WAKEUP:
		  	{
				 stm32l4_configgpio(GPRS_MCU_ONOFF);
				 stm32l4_configgpio(GPRS_MCU_RST);
 			     usleep(200*1000);
				 //init
				 stm32l4_gpiowrite(GPRS_MCU_ONOFF,false);				//IGT	high
				 stm32l4_gpiowrite(GPRS_MCU_RST,false);				//RST	high
 			     usleep(200*1000);
                 //start up
				 stm32l4_gpiowrite(GPRS_MCU_ONOFF,true);				//IGT	lower
 			     usleep(200*1000);
				 stm32l4_gpiowrite(GPRS_MCU_ONOFF,false);				//IGT	high
			}
		    break;
		/************************************************/
		//正
		case GPRS_RST:
		  	{
				 stm32l4_configgpio(GPRS_MCU_RST);
				 //init
				 stm32l4_gpiowrite(GPRS_MCU_RST,false);
 			     usleep(200*1000);
				 //rst
				 stm32l4_gpiowrite(GPRS_MCU_RST,true);
 			     usleep(200*1000);
				 stm32l4_gpiowrite(GPRS_MCU_RST,false);

			}
		    break;
		/************************************************/
		//add by liushue 2017.09.15
		case AM2320_WAKEUP:
		  	{
				 stm32l4_configgpio(GPIO_I2C2_SCL_AM2320);
				 stm32l4_configgpio(GPIO_I2C2_SDA_AM2320);
				 AM2320_WakeUp(0xb8);
				 printf("111\n");
			}
		    break;
		
		case RDWR_AM2320_DATA:
		  	{ 						
				 struct i2c_msg_s *msg = (struct i2c_msg_s *)((uintptr_t)arg);

				 if(msg->flags == I2C_M_READ)
				 {
					 	printf("222-r\n");
						AM2320_Multiple_read(msg);
				 }
				 else
				 {
						 printf("333-w\n");
						AM2320_Multiple_Write(msg);
				 }
			}
		    break;
		/*********************************************************/
		default:
				return -ENOTTY;
				break;
	}
	return OK;
  
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32l4_get_uniqueid(uniqueid);
  return OK;
}
#endif




/***************************************************************************************************************************
函数名称：void AM2320_Start(void)
功    能：产生i2c起始条件
输入参数：
输出参数：
编写时间：2017.09.15
编 写 人：liushuhe
注    意：
***************************************************************************************************************************/
void AM2320_Start(void)
{
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
	
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,false);
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
}

/***************************************************************************************************************************
函数名称：void AM2320_Stop(void)
功    能：产生i2c停止条件
输入参数：
输出参数：
编写时间：2017.09.15
编 写 人：liushuhe
注    意：
***************************************************************************************************************************/
void AM2320_Stop(void)
{
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,false);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
    usleep(15);
}


/***************************************************************************************************************************
函数名称：void Ack(void)
功    能：ack
输入参数：
输出参数：
编写时间：2017.09.15
编 写 人：liushuhe
注    意：
***************************************************************************************************************************/
void Ack(void)
{  
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,false);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
}
/***************************************************************************************************************************
函数名称：void NoAck(void)
功    能：noack
输入参数：
输出参数：
编写时间：2017.09.15
编 写 人：liushuhe
注    意：
***************************************************************************************************************************/
void NoAck(void)
{  
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
}


/***************************************************************************************************************************
函数名称：void Test_Ack(void)
功    能：Test_Ack
输入参数：
输出参数：
编写时间：2017.09.15
编 写 人：liushuhe
注    意：
***************************************************************************************************************************/
char Test_Ack(void)
{  
   char ACK_Flag=0;
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
    usleep(15);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
    usleep(15);
	
   if(stm32l4_gpioread(GPIO_I2C2_SDA_AM2320)==0)
     	ACK_Flag = 1;
   else 
     	ACK_Flag = 0;
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
   return ACK_Flag;
}
/***************************************************************************************************************************
函数名称：void AM2320_SendByte(uint8_t dat)
功    能：i2c发送一个字节
输入参数：
输出参数：
编写时间：2016.04.06
编 写 人：lsh
注    意：
***************************************************************************************************************************/
void AM2320_SendByte(uint8_t dat)
{
   unsigned char BitCnt=8;//一字节8位
   //设置SDA 口为输出
   do
   {
		stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
	    usleep(15);
      	if((dat&0x80)==0) //判断最高位是0还是1
			stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,false);
     	else
			stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
		stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
	    usleep(15);
      	dat=dat<<1;//将buffer中的数据左移一位
      	BitCnt--;
   }
   while(BitCnt);
   stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
	
}
/***************************************************************************************************************************
函数名称：uint8_t AM2320_RecvByte(void)
功    能：i2c接收一个字节
输入参数：
输出参数：
编写时间：2017.09.16
编 写 人：lsh
注    意：
***************************************************************************************************************************/
uint8_t AM2320_RecvByte(void)
{
  unsigned char BitCnt=8,IIC_RX_Data=0;
  stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);

  do
  {
		stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
	    usleep(15);
	 	IIC_RX_Data <<=1;   //数据左移一位
	 	BitCnt--;	  
		stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
	    usleep(15);
     if(stm32l4_gpioread(GPIO_I2C2_SDA_AM2320)==1)
       IIC_RX_Data = IIC_RX_Data|0x01;  //低位置1
     else
       IIC_RX_Data = IIC_RX_Data&0x0fe; //低位清0	
   }
   while(BitCnt);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,false);
	
   return IIC_RX_Data;
	
}

/***************************************************************************************************************************
函数名称：void AM2320_Multiple_Write(struct  i2c_msg_s * msg)
功    能：发送读取温湿度指令
输入参数：
输出参数：
编写时间：2017.09.16
编 写 人：lsh
注    意：
***************************************************************************************************************************/
int AM2320_Multiple_Write(struct  i2c_msg_s * msg)
{
	uint8_t wNum = 0;

    AM2320_Start();                  								//起始信号
    AM2320_SendByte(msg->addr|0x00);   								//发送设备地址+写信号
   if(!Test_Ack())
   {	
	  //return(0);
   }
	for(wNum=0;wNum<msg->length;wNum++)
	{
		AM2320_SendByte(msg->buffer[wNum]);
	   if(!Test_Ack())
	   {	
		  //return(0);
	   }
	}
    AM2320_Stop();                   								//发送停止信号
    return 0;
}

/***************************************************************************************************************************
函数名称：void AM2320_Multiple_read(struct  i2c_msg_s * msg)
功    能：发送读取温湿度时序
输入参数：
输出参数：
编写时间：2017.09.16
编 写 人：lsh
注    意：
***************************************************************************************************************************/
int AM2320_Multiple_read(struct  i2c_msg_s * msg)
{
	uint8_t rNum = 0;
		
    AM2320_Start();                          				//起始信号
    AM2320_SendByte(msg->addr|0x01);         				//发送设备地址+读信号
   if(!Test_Ack())
   {	
	  //return(0);
   }
    usleep(40);
	for(rNum=0;rNum<(msg->length-1);rNum++)
	{
	    msg->buffer[rNum] = AM2320_RecvByte();
	    Ack();                       			//回应ACK
	}
    msg->buffer[rNum] = AM2320_RecvByte();
    NoAck();                       				//最后一个数据需要回NOACK

    AM2320_Stop();                           				//停止信号
    return 0;
}


/***************************************************************************************************************************
函数名称：void AM2320_WakeUp(char dat)
功    能：唤醒am2320传感器
输入参数：
输出参数：
编写时间：2017.09.15
编 写 人：lsh
注    意：
***************************************************************************************************************************/
void AM2320_WakeUp(char SlaveAddress)
{
	stm32l4_gpiowrite(GPIO_I2C2_SDA_AM2320,true);
	stm32l4_gpiowrite(GPIO_I2C2_SCL_AM2320,true);
	
	AM2320_Start();
	AM2320_SendByte(SlaveAddress);
	Test_Ack();
    usleep(1200);
	AM2320_Stop();
}



