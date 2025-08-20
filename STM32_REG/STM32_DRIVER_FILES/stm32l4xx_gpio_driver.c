/*
 * stm32l4xx_gpio_driver.c
 *
 *  Created on: Aug 20, 2025
 *      Author: USER
 */

#include "stm32l4xx_gpio_driver.h"


/* Peripheral clock setup */

// Enable or disable clock for given GPIO port
void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)      { GPIOA_PCLK_EN(); }
		else if(pGPIOx==GPIOB)   { GPIOB_PCLK_EN(); }
		else if(pGPIOx==GPIOC)   { GPIOC_PCLK_EN(); }
		else if(pGPIOx==GPIOD)   { GPIOD_PCLK_EN(); }
		else if(pGPIOx==GPIOE)   { GPIOE_PCLK_EN(); }
		else if(pGPIOx==GPIOF)   { GPIOF_PCLK_EN(); }
		else if(pGPIOx==GPIOG)   { GPIOG_PCLK_EN(); }
		else if(pGPIOx==GPIOH)   { GPIOH_PCLK_EN(); }
	}
	else
	{
		if(pGPIOx == GPIOA)      { GPIOA_PCLK_DI(); }
		else if(pGPIOx==GPIOB)   { GPIOB_PCLK_DI(); }
		else if(pGPIOx==GPIOC)   { GPIOC_PCLK_DI(); }
		else if(pGPIOx==GPIOD)   { GPIOD_PCLK_DI(); }
		else if(pGPIOx==GPIOE)   { GPIOE_PCLK_DI(); }
		else if(pGPIOx==GPIOF)   { GPIOF_PCLK_DI(); }
		else if(pGPIOx==GPIOG)   { GPIOG_PCLK_DI(); }
		else if(pGPIOx==GPIOH)   { GPIOH_PCLK_DI(); }
	}
}


/* Initialization and de-initialization */

// Initialize GPIO pin (mode, speed, type, pull-up/down, alt function)
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// 1. Configure mode (input, output, analog, alt function)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		// interrupt mode (not implemented yet)
	}

	// 2. Configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Configure pull-up / pull-down
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configure output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure alternate function if mode is alt function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;  // AFR[0] or AFR[1]
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

// Reset all registers of given GPIO port
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)      { GPIOA_REG_RESET(); }
	else if(pGPIOx==GPIOB)   { GPIOB_REG_RESET(); }
	else if(pGPIOx==GPIOC)   { GPIOC_REG_RESET(); }
	else if(pGPIOx==GPIOD)   { GPIOD_REG_RESET(); }
	else if(pGPIOx==GPIOE)   { GPIOE_REG_RESET(); }
	else if(pGPIOx==GPIOF)   { GPIOF_REG_RESET(); }
	else if(pGPIOx==GPIOG)   { GPIOG_REG_RESET(); }
	else if(pGPIOx==GPIOH)   { GPIOH_REG_RESET(); }
}


/* Read and write operations */

// Read value (0/1) from a single input pin
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx , uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
	return value;
}

// Read values from entire input port
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

// Write value (0/1) to a single output pin
void GPIO_WriteToOutputPin(GPIO_Regdef_t *pGPIOx , uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);   // set bit
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);  // clear bit
	}
}

// Write value to entire output port
void GPIO_WriteToOutputPort(GPIO_Regdef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR = value;
}

// Toggle (flip) output pin state
void GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/* IRQ and ISR configuration */

// Enable or disable given interrupt number
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi)
{
	// to be implemented
}

// Set priority for given interrupt number
void GPIO_PRIORITY_CONFIG(uint8_t IRQNumber,uint8_t IRQPriority)
{
	// to be implemented
}

// Handle interrupt for given pin
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// to be implemented
}
