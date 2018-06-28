/****************************************************************************
 * include/nuttx/input/max31865.h
 * add by liushuhe 2018.06.28
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MAX31865_H
#define __INCLUDE_NUTTX_SENSORS_MAX31865_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#if defined(CONFIG_SENSORS_MAX31865)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#define MAX31865_SPI_MAXFREQ 4000000

struct spi_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: max31865_register
 *
 * Description:
 *  This function will register the max31855 driver as /dev/tempN
 *  where N is the minor device number
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MAX31855
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int max31865_register(FAR const char *devpath, FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MAX31865 */
#endif /* __INCLUDE_NUTTX_SENSORS_MAX31865_H */
