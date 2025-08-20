#include <stdint.h>

#include "stm32l4xx.h"
#include "stm32l4xx_gpio_driver.h"


// Simple delay function (for debouncing and LED blink timing)
void delay()
{
	for(uint32_t i = 0; i < 200000; i++);
}

int main(void)
{
	/* Create GPIO handles for LED and Button */
	GPIO_Handle_t GpioLed, GpioBtn;

	// --- LED Setup ---
	GpioLed.pGPIOx = GPIOA;                          // LED is on Port A
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // Use Pin 5
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;   // Set as Output
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; // Fast speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-pull type
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No pull-up/down
	GPIO_PeriClockControl(GPIOA, ENABLE);           // Turn on clock for Port A
	GPIO_Init(&GpioLed);                            // Apply LED settings

	// --- Button Setup ---
	GpioBtn.pGPIOx = GPIOC;                          // Button is on Port C
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13; // Use Pin 13
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;     // Set as Input
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; // Fast speed
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No pull-up/down
	GPIO_PeriClockControl(GPIOC, ENABLE);           // Turn on clock for Port C
	GPIO_Init(&GpioBtn);                            // Apply Button settings

	while(1)
	{
		// Button pressed? (PC13 = 0 when pressed)
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
		{
			delay(); // Small wait to avoid bouncing effect
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5); // Change LED state
		}
	}
}
