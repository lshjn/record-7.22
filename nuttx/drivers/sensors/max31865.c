/****************************************************************************
 * drivers/sensors/max31865.c
 * add by liushuhe 2018.06.28
****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/max31865.h>
#include <nuttx/random.h>
#include <string.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MAX31865)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MAX31865_WRITE      (1<<7)

/****************************************************************************
 * Private
 ****************************************************************************/

struct max31865_dev_s
{
  FAR struct spi_dev_s *spi;           /* Saved SPI driver instance */
  int16_t temp;
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void max31865_lock(FAR struct spi_dev_s *spi);
static void max31865_unlock(FAR struct spi_dev_s *spi);

/* Character driver methods */

static int     max31865_open(FAR struct file *filep);
static int     max31865_close(FAR struct file *filep);
static ssize_t max31865_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t max31865_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max31865fops =
{
  max31865_open,
  max31865_close,
  max31865_read,
  max31865_write,
  NULL,
  NULL
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max31865_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void max31865_lock(FAR struct spi_dev_s *spi)
{
  (void)SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE1);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 400000);
}

/****************************************************************************
 * Name: max31865_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void max31865_unlock(FAR struct spi_dev_s *spi)
{
  (void)SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: max31865_open
 *
 * Description:
 *   This function is called whenever the MAX31865 device is opened.
 *
 ****************************************************************************/

static int max31865_open(FAR struct file *filep)
{
	printf("max31865_open\n");
  return OK;
}

/****************************************************************************
 * Name: max31865_close
 *
 * Description:
 *   This routine is called when the MAX31865 device is closed.
 *
 ****************************************************************************/

static int max31865_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: max31865_read
 ****************************************************************************/

static ssize_t max31865_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct max31865_dev_s *priv  = inode->i_private;
  uint8_t 					addr;

  /* Check for issues */
  

  if (!buffer)
    {
      printf("ERROR: Buffer is null\n");
      return -EINVAL;
    }
#if 0
  if (buflen != 2)
    {
      printf("ERROR: You can't read something other than 16 bits (2 bytes)\n");
      return -EINVAL;
    }
#endif
  /* Enable MAX31865's chip select */

  max31865_lock(priv->spi);
  if(strcmp(inode->i_name,"max31865_1") == 0)
  {
	  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(0), true);
  }
  else if(strcmp(inode->i_name,"max31865_2") == 0)
  {
	  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(1), true);
  }

  //add by liushuhe 2018.06.29
  addr = 0x00;
  SPI_SEND(priv->spi, addr);
  SPI_RECVBLOCK(priv->spi, buffer, buflen);
  
  /* Disable MAX31865's chip select */
  if(strcmp(inode->i_name,"max31865_1") == 0)
  {
	  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(0), false);
  }
  else if(strcmp(inode->i_name,"max31865_2") == 0)
  {
	  SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(1), false);
  }
  
  max31865_unlock(priv->spi);

  /* Return two bytes, the temperature is fixed point Q12.2, then divide by 4
   * in your application in other to get real temperature in Celsius degrees.
   */

  return buflen;
}

/****************************************************************************
 * Name: max31865_write
 ****************************************************************************/

static ssize_t max31865_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
	FAR struct inode		  *inode = filep->f_inode;
	FAR struct max31865_dev_s *priv  = inode->i_private;
	uint8_t 				  addr;
    char temp_buf[255] = {0};
	/* Check for issues */
	
	if (!buffer)
	  {
		printf("ERROR: Buffer is null\n");
		return -EINVAL;
	  }
#if 0	
	if (buflen != 2)
	  {
		printf("ERROR: You can't read something other than 16 bits (2 bytes)\n");
		return -EINVAL;
	  }
#endif	
	/* Enable MAX31865's chip select */


	max31865_lock(priv->spi);
	if(strcmp(inode->i_name,"max31865_1") == 0)
	{
		SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(0), true);
	}
	else if(strcmp(inode->i_name,"max31865_2") == 0)
	{
		SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(1), true);
	}
	
	//add by liushuhe 2018.06.29
	//addr = buffer[0] | MAX31865_WRITE;
	//SPI_SEND(priv->spi, addr);
	//SPI_SNDBLOCK(priv->spi, &buffer[1], buflen-1);
	
	memcpy(temp_buf,buffer,buflen);
	
	temp_buf[0] = temp_buf[0] | MAX31865_WRITE;
	

	
	SPI_SNDBLOCK(priv->spi, &temp_buf[0], buflen);
	
	/* Disable MAX31865's chip select */
	if(strcmp(inode->i_name,"max31865_1") == 0)
	{
		SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(0), false);
	}
	else if(strcmp(inode->i_name,"max31865_2") == 0)
	{
		SPI_SELECT(priv->spi, SPIDEV_TEMPERATURE(1), false);
	}
	
	max31865_unlock(priv->spi);
	
	/* Return two bytes, the temperature is fixed point Q12.2, then divide by 4
	 * in your application in other to get real temperature in Celsius degrees.
	 */
	
	return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max31865_register
 *
 * Description:
 *   Register the MAX31865 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate wit
 *   MAX31865 addr - The I2C address of the MAX31865.  The base I2C address
 *   of the MAX31865 is 0x48.  Bits 0-3 can be controlled to get 8 unique
 *   addresses from 0x48 through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max31865_register(FAR const char *devpath, FAR struct spi_dev_s *spi)
{
  FAR struct max31865_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the MAX31865 device structure */

  priv = (FAR struct max31865_dev_s *)kmm_malloc(sizeof(struct max31865_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi        = spi;
  priv->temp       = 0;

  /* Register the character driver */

  ret = register_driver(devpath, &g_max31865fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_SENSORS_MAX31865 */
