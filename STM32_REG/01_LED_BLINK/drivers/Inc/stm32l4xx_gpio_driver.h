/*
 * stm32l4xx_gpio_driver.h
 *
 *  Created on: Aug 20, 2025
 *      Author: USER
 */

#ifndef INC_STM32L4XX_GPIO_DRIVER_H_
#define INC_STM32L4XX_GPIO_DRIVER_H_

#include "stm32l4xx.h"



/*
 *  This is a configuration Structure for a GPIO pin
 */

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;



/*
 *  This is a Handle Structure for GPIO Pin
 */

typedef struct{
	GPIO_Regdef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;



/*
 *  GPIO Pin possible modes
 */


#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6



/*
 *  GPIO Pin possible speeds
 */


#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3


/*
 *  GPIO Pin possible output types
 */

#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1


/*
 *  GPIO Pin Pull Up and Pull Down configuration macros
 */

#define GPIO_NO_PUPD   0
#define GPIO_PIN_PU    1
#define GPIO_PIN_PD    2



/*
 *  GPIO Pin numbers
 */

#define GPIO_PIN_NO_0     0
#define GPIO_PIN_NO_1     1
#define GPIO_PIN_NO_2     2
#define GPIO_PIN_NO_3     3
#define GPIO_PIN_NO_4     4
#define GPIO_PIN_NO_5     5
#define GPIO_PIN_NO_6     6
#define GPIO_PIN_NO_7     7
#define GPIO_PIN_NO_8     8
#define GPIO_PIN_NO_9     9
#define GPIO_PIN_NO_10    10
#define GPIO_PIN_NO_11    11
#define GPIO_PIN_NO_12    12
#define GPIO_PIN_NO_13    13
#define GPIO_PIN_NO_14    14
#define GPIO_PIN_NO_15    15





/*
 *  APIs supported by this driver
 *
 */

/* Peripheral clock setup */
void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx , uint8_t EnorDi);


/* Initialization and de-initialization */
void GPIO_Init(GPIO_Handle_t *pGPIOHANDLE);
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx);



/* Read and write operations */
uint8_t  GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx , uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Regdef_t *pGPIOx , uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_Regdef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t PinNumber);



/* IRQ and ISR configuration */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void GPIO_PRIORITY_CONFIG(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);












#endif /* INC_STM32L4XX_GPIO_DRIVER_H_ */
