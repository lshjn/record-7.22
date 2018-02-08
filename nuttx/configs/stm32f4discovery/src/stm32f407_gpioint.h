#include <arch/board/board.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
//add by liushuhe 207.09.08

#include <nuttx/ioexpander/gpio.h>
#include "stm32f4discovery.h"
/* GPIO pins used by the GPIO Subsystem */

//add by liushuhe 2017.10.12
//changed by liubofei 2017-12-28
#define BOARD_NGPIOIN     0 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    0 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

//add by liushuhe 2017.10.09
#define HALL_SENSORA          (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN8)
#define HALL_SENSORB          (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTC | GPIO_PIN9)
#define LIGHT_315Mhz          (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTB | GPIO_PIN15)
//new add by liubofei 2017-12-28
#define LOCK_CLOSE			  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTB | GPIO_PIN12)
#define LOCK_OPEN			  (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTB | GPIO_PIN14)

//Photoresistor
#define	ZD801S_SENSOR		   (GPIO_PORTB | GPIO_PIN5 | GPIO_INPUT | GPIO_PULLDOWN)

#define GPIO_OUT1         	   (GPIO_PORTB | GPIO_PIN7 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz)                          

//#define GPIO_INT2         		(GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTB | GPIO_PIN5)
#define	GPS_PPS_INT			(GPIO_INPUT | GPIO_PULLUP  |GPIO_EXTI | GPIO_PORTA | GPIO_PIN8)

//×¢²ágpioÇý¶¯Éè±¸
int stm32f407_gpiodev_initialize(void);




