#include <arch/board/board.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
//add by liushuhe 207.09.08

#include <nuttx/ioexpander/gpio.h>
#include "stm32f4discovery.h"
/* GPIO pins used by the GPIO Subsystem */

//add by liushuhe 2018.06.28
#define BOARD_NGPIOIN     0 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    0 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define	EXTER_CTR_INT			(GPIO_INPUT | GPIO_PULLUP  |GPIO_EXTI | GPIO_PORTB | GPIO_PIN12)

//×¢²ágpioÇý¶¯Éè±¸
int stm32f407_gpiodev_initialize(void);




