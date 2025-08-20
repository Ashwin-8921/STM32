#include<stdint.h>
#include<string.h>
#include<stdio.h>
#include"stm32l4xx.h"
#include"stm32l4xx_gpio_driver.h"

// Simple delay function (used for debouncing)
void delay()
{
	for(uint32_t i=0; i<250000; i++);
}

int main(void)
{
    /* Declare handles for LED and Button */
	GPIO_Handle_t GpioLed, GpioBtn;

    // Clear structures (avoid garbage values)
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

    // --- LED Setup ---
	GpioLed.pGPIOx = GPIOA;                          // LED is on Port A
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;  // Pin A5
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;    // Set as output
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; // Fast speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-pull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up
	GPIO_PeriClockControl(GPIOA, ENABLE);            // Enable Port A clock
	GPIO_Init(&GpioLed);                             // Initialize LED pin

    // --- Button Setup ---
	GpioBtn.pGPIOx = GPIOC;                          // Button is on Port C
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;  // Pin C13
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;   // Interrupt on falling edge
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Fast speed
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up
	GPIO_PeriClockControl(GPIOC, ENABLE);            // Enable Port C clock
	GPIO_Init(&GpioBtn);                             // Initialize button pin

    // --- NVIC Configuration ---
	GPIO_PRIORITY_CONFIG(IRQ_NO_EXTI15_10, 15);     // Set EXTI15_10 priority
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE); // Enable EXTI15_10 interrupt

	while(1); // Wait here, actions happen in ISR
}

// --- Interrupt Service Routine for PC13 ---
void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_13);               // Clear EXTI flag
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);    // Toggle LED
	delay();                                        // Debounce
}
