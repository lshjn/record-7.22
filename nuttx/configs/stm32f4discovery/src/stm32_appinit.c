/****************************************************************************
 * config/stm32f4discovery/src/stm32_appinit.c
 *
 *   Copyright (C) 2012, 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/board.h>
//add by liushuhe 2017.11.13
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>


#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/binfmt/elf.h>
#include <nuttx/i2c/i2c_master.h>


#include "stm32f4discovery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef OK
#  define OK 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
#ifdef CONFIG_SENSORS_BMP180
  stm32_bmp180initialize("/dev/press0");
#endif

#ifdef CONFIG_BOARD_INITIALIZE
  /* Board initialization already performed by board_initialize() */

  return OK;
#else
  /* Perform board-specific initialization */

  return stm32_bringup();
#endif
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
	_uint32_t * ptr_pinValue = (_uint32_t *)arg;
	//add by liushuhe  2017.09.04
	switch(cmd)
	{
		/*********************************************************/
		//led0
		case BOARDIOC_LED0_ON:
				stm32_configgpio(GPIO_LED0);
				stm32_gpiowrite(GPIO_LED0,false);
				break;
		case BOARDIOC_LED0_OFF:
				stm32_configgpio(GPIO_LED0);
				stm32_gpiowrite(GPIO_LED0,true);
				break;
		//led1
		case BOARDIOC_LED1_ON:
				stm32_configgpio(GPIO_LED1);
				stm32_gpiowrite(GPIO_LED1,false);
				break;
		case BOARDIOC_LED1_OFF:
				stm32_configgpio(GPIO_LED1);
				stm32_gpiowrite(GPIO_LED1,true);
				break;
		//led2
		case BOARDIOC_LED2_ON:
				stm32_configgpio(GPIO_LED2);
				stm32_gpiowrite(GPIO_LED2,false);
				break;
		case BOARDIOC_LED2_OFF:
				stm32_configgpio(GPIO_LED2);
				stm32_gpiowrite(GPIO_LED2,true);
				break;
		//led3
		case BOARDIOC_LED3_ON:
				stm32_configgpio(GPIO_LED3);
				stm32_gpiowrite(GPIO_LED3,false);
				break;
		case BOARDIOC_LED3_OFF:
				stm32_configgpio(GPIO_LED3);
				stm32_gpiowrite(GPIO_LED3,true);
				break;
		//get max31865 DRDY		
		case BOARDIOC_GET_SPI1_DRDY:
				stm32_configgpio(GPIO_MAX31865_1_DRDY);
				*ptr_pinValue = stm32_gpioread(GPIO_MAX31865_1_DRDY);
				//printf("ptr_pinValue1=%d,%d\n",*ptr_pinValue,stm32_gpioread(GPIO_MAX31865_1_DRDY));
				break;
		case BOARDIOC_GET_SPI2_DRDY:
				stm32_configgpio(GPIO_MAX31865_1_DRDY);
				*ptr_pinValue = stm32_gpioread(GPIO_MAX31865_2_DRDY);
				//printf("ptr_pinValue2=%d,%d\n",*ptr_pinValue,stm32_gpioread(GPIO_MAX31865_2_DRDY));
				break;
				

		/*********************************************************/
		default:
				return -ENOTTY;
				break;
	}
	return OK;
  
}
#endif

//stm32f407vg_at24c08_automount
//add by liushuhe 2017.11.10
int stm32f407vg_at24c08_automount(struct i2c_master_s *i2c)
{
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  char blockdev[18];
  char chardev[12];
  char devname[16];
  
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      mtd = at24c_initialize(i2c);
      if (!mtd)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind i2c%d to the AT24 EEPROM driver\n",AT24_BUS);
          return -ENODEV;
        }
	  
//add by liushuhe 2017.11.19
#ifndef CONFIG_FS_NXFFS
      /* And finally, use the FTL layer to wrap the MTD driver as a block driver */
      ret = ftl_initialize(AT24_MINOR, mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }
	  
      // Use the minor number to create device paths//

      snprintf(blockdev, 18, "/dev/mtdblock%d", AT24_MINOR);
      snprintf(chardev, 12, "/dev/mtd%d", AT24_MINOR);

      // Now create a character device on the block device //

      ret = bchdev_register(blockdev, chardev, false);
      if (ret < 0)
        {
          ferr("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
          return ret;
        }

  
#else
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      printf("ERROR: NXFFS initialization failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system at /mnt/w25 */

  snprintf(devname, 12, "/mnt/at24%c", 'a' + AT24_MINOR);
  ret = mount(NULL, devname, "nxffs", 0, NULL);
  if (ret < 0)
    {
      printf("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }
#endif

	 
      /* Now we are initialized */
      initialized = true;
    }

  return OK;
}


