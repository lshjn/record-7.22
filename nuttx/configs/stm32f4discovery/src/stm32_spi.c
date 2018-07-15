/************************************************************************************
 * configs/stm32f4discovery/src/stm32_spi.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/
	 
#include <sys/types.h>
#include <sys/mount.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/config.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
//add by liushuhe 2018.06.30
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"

#include "stm32f4discovery.h"
//add by liushuhe 2017.12.20
#include <nuttx/wireless/cc1101.h>
//add by liushuhe 2018.06.29
#include <nuttx/sensors/max31865.h>

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI1
struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
struct spi_dev_s *g_spi2;
#endif

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery board.
 *
 ************************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32_SPI1
  (void)stm32_configgpio(GPIO_CS_MEMS);    /* MEMS chip select */
#endif
#if defined(CONFIG_STM32_SPI2) && defined(CONFIG_SENSORS_MAX31855)
  (void)stm32_configgpio(GPIO_MAX31855_CS); /* MAX31855 chip select */
#endif
#if defined(CONFIG_STM32_SPI2) && defined(CONFIG_SENSORS_MAX6675)
  (void)stm32_configgpio(GPIO_MAX6675_CS); /* MAX6675 chip select */
#endif
#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
  (void)stm32_configgpio(GPIO_OLED_CS);    /* OLED chip select */
# if defined(CONFIG_LCD_UG2864AMBAG01)
  (void)stm32_configgpio(GPIO_OLED_A0);    /* OLED Command/Data */
# endif
# if defined(CONFIG_LCD_UG2864HSWEG01) || defined(CONFIG_LCD_SSD1351)
  (void)stm32_configgpio(GPIO_OLED_DC);    /* OLED Command/Data */
# endif
#endif
}




/************************************************************************************
 * Name: stm32l4_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-F401RE and
 *   Nucleo-F411RE boards.
 *
 ************************************************************************************/
//add by liushehe 2017.12.19
void  stm32f407_cc1100_spiinitialize(void)
{

#ifdef CONFIG_STM32_SPI1
  /* Configure SPI-based devices */
  g_spi1 = stm32_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: FAILED to initialize SPI port 1\n");

    }

/* Setup CS, EN & IRQ line IOs */

//add by liushuhe 2017.11.30
#ifdef CONFIG_WL_CC1101
    stm32_configgpio(GPIO_CC1100_CS);
	cc1101_auto_register(g_spi1,GPIO_GDO2_C1100,GPIO_SPI1_MISO,GPIO_SPI1_MOSI);
#endif
#endif

}


//add by liushuhe 2018.06.30
int stm32_mx25L_initialize(void)
{
	int minor = 0;
//add by liushuhe 2018.06.29
#ifdef CONFIG_STM32_SPI3

  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  char devname[12];
#ifndef CONFIG_FS_NXFFS
  char blockdev[18];
  char chardev[12];
#endif


  int ret;

  /* Get the SPI port */
  //add by liushuhe 2018.06.29
  spi = stm32_spibus_initialize(3);
  if (!spi)
	{
	  ferr("ERROR: Failed to initialize SPI port 3\n");
	  return -ENODEV;
	}

  /* Now bind the SPI interface to the W25 SPI FLASH driver */
  stm32_configgpio(GPIO_MA25L_CS);
  
  mtd = mx25l_initialize_spi(spi);
  if (!mtd)
	{
	  ferr("ERROR: Failed to bind SPI port 3 to the mx25L FLASH driver\n");
	  return -ENODEV;
	}

  /* And finally, use the FTL layer to wrap the MTD driver as a block driver */
#if 1
  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
	{
	  ferr("ERROR: Initialize the FTL layer\n");
	  return ret;
	}

  // Use the minor number to create device paths//
  snprintf(blockdev, 18, "/dev/mtdblock%d", minor);
  snprintf(chardev, 12, "/dev/mtd%d", minor);
  
  //add by liushuhe 2018.06.30
  //在块设备上创建一个字符设备
  ret = bchdev_register(blockdev, chardev, false);
  if (ret < 0)
	{
	  ferr("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
	  return ret;
	}	  
#endif
#endif
#if 0
	ret = nxffs_initialize(mtd);

  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system at /mnt/mx25L */

  snprintf(devname, 12, "/mnt/MX25L%c", 'a' + minor);
  ret = mount(NULL, devname, "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }
#endif	
  return OK;
}


//add by liushuhe_test 2018.06.30
int stm32_w25_initialize(void)
{
	int minor = 0;
//add by liushuhe 2018.06.29
#ifdef CONFIG_STM32_SPI3

  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  char devname[12];
#ifndef CONFIG_FS_NXFFS
  char blockdev[18];
  char chardev[12];
#endif


  int ret;

  /* Get the SPI port */
  //add by liushuhe 2018.06.29
  spi = stm32_spibus_initialize(3);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the W25 SPI FLASH driver */
  //stm32_configgpio(GPIO_W25_CS);
    stm32_configgpio(GPIO_MA25L_CS);

  mtd = w25_initialize(spi);
  if (!mtd)
    {
      ferr("ERROR: Failed to bind SPI port 3 to the mx25L FLASH driver\n");
      return -ENODEV;
    }

#ifndef CONFIG_FS_NXFFS
  /* And finally, use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initialize the FTL layer\n");
      return ret;
    }

  // Use the minor number to create device paths//
  snprintf(blockdev, 18, "/dev/mtdblock%d", minor);
  snprintf(chardev, 12, "/dev/mtd%d", minor);
  
  //add by liushuhe 2018.06.30
  #ifndef CONFIG_FS_FAT
  
	  //在块设备上创建一个字符设备
	  ret = bchdev_register(blockdev, chardev, false);
	  if (ret < 0)
		{
		  ferr("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
		  return ret;
		}	  
  #else
  
	  /*挂载fat32 文件系统  /mnt/spiflash_fat32  */
	  //fat32 没有成功，问题比较多，改用nxffs文件系统
	  ret = mount(blockdev, "/tmp/MX25L_fat32", "vfat", 0, NULL);
	  if (ret < 0)
		{
		  ferr("ERROR: Failed to mount the FAT volume: %d\n", errno);
		  return ret;
		}	  
  #endif

  
#else
  /* Initialize to provide NXFFS on the MTD interface */
  //改用nxffs文件系统
	ret = nxffs_initialize(mtd);

  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system at /mnt/mx25L */

  snprintf(devname, 12, "/mnt/MX25L%c", 'a' + minor);
  ret = mount(NULL, devname, "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }
#endif
#endif
  return OK;
}


//add by liushuhe 2018.06.28
void  stm32f407_max31865_spiinitialize(void)
{
	int ret = -1;
//spi1
#ifdef CONFIG_STM32_SPI1
  /* Configure SPI-based devices */
  g_spi1 = stm32_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: FAILED to initialize SPI port 1\n");

    }
	
#ifdef CONFIG_SENSORS_MAX31865
	    stm32_configgpio(GPIO_MAX31865_CS1);
		
		ret = max31865_register("/dev/max31865_1", g_spi1);
		if (ret < 0)
		{
			snerr("ERROR: Error registering MAX31865\n");
		}
		
#endif
#endif
//add by liushuhe_test 2018.06.30
#if 1
//spi2
#ifdef CONFIG_STM32_SPI2
  /* Configure SPI-based devices */
  g_spi2 = stm32_spibus_initialize(2);
  if (!g_spi2)
    {
      spierr("ERROR: FAILED to initialize SPI port 2\n");

    }
#ifdef CONFIG_SENSORS_MAX31865
	    stm32_configgpio(GPIO_MAX31865_CS2);

		ret = max31865_register("/dev/max31865_2", g_spi2);
		if (ret < 0)
		{
			snerr("ERROR: Error registering MAX31865\n");
		}

#endif
#endif
#endif
}


/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including stm32_spibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  
//add by liushuhe 2017.12.22

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_OLED_CS, !selected);
    }
  else
#endif
    {
      stm32_gpiowrite(GPIO_CS_MEMS, !selected);
    }

//add by liushuhe 2018.06.28
#ifdef CONFIG_SENSORS_MAX31865
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX31865_CS1, !selected);
    }
#endif


}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_SENSORS_MAX31855)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX31855_CS, !selected);
    }
#endif
#if defined(CONFIG_SENSORS_MAX6675)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX6675_CS, !selected);
    }
#endif

//add by liushuhe_test 2018.06.30
#if 0
//add by liushuhe 2018.06.28
#ifdef CONFIG_SENSORS_MAX31865
  if (devid == SPIDEV_TEMPERATURE(1))
    {
      stm32_gpiowrite(GPIO_MAX31865_CS2, !selected);
    }
#endif
#endif

	//add by liushuhe_test 2018.06.30
	stm32_gpiowrite(GPIO_W25_CS, !selected);



}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  //add by liushuhe 2018.06.29
  stm32_gpiowrite(GPIO_MA25L_CS, !selected);
  	
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32_SPI1
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* "This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       *
       *  A0 = "H": the inputs at D0 to D7 are treated as display data.
       *  A0 = "L": the inputs at D0 to D7 are transferred to the command
       *       registers."
       */

# if defined(CONFIG_LCD_UG2864AMBAG01)
      (void)stm32_gpiowrite(GPIO_OLED_A0, !cmd);
# endif
# if defined(CONFIG_LCD_UG2864HSWEG01) || defined(CONFIG_LCD_SSD1351)
      (void)stm32_gpiowrite(GPIO_OLED_DC, !cmd);
# endif
      return OK;
    }
#endif

  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI2
int stm32_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI3
int stm32_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 */
