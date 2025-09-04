#include "stm32l4xx.h"
#include "stm32l4xx_timer_driver.h"
#include "stm32l4xx_usart_driver.h"
#include "stm32l4xx_gpio_driver.h"
#include <stdio.h>
#include <string.h>

#define TIM2EN   (1U<<0)   // TIM2 clock enable bit
#define TIM3EN   (1U<<1)   // TIM3 clock enable bit
#define CR1_CEN  (1U<<0)   // Counter enable bit
#define GPIOA_EN (1U<<0)   // GPIOA clock enable bit
#define SR_UIF   (1U<<0)   // Update interrupt flag

static void TIM2_Callback(void);

USART_Handle_t usart2_handle;   // USART2 handle structure
GPIO_Handle_t GpioLed;          // LED GPIO handle structure

// Initialize USART2 peripheral
void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;                                 // Use USART2
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;  // Baud rate = 115200
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // No flow control
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;        // Enable TX and RX
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // One stop bit
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8 data bits
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // No parity
    USART_Init(&usart2_handle);                                     // Apply config
}

// Configure PA2 (TX) and PA3 (RX) for USART2
void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart2_gpio;
    memset(&usart2_gpio,0,sizeof(usart2_gpio));                      // Reset struct

    usart2_gpio.pGPIOx = GPIOA;                                      // Use Port A
    usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;       // Alternate function
    usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;     // Push-pull
    usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;    // Pull-up
    usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;      // Fast speed
    usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;               // AF7 = USART2

    GPIOA_PCLK_EN();                                                 // Enable GPIOA clock

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;       // PA2 = TX
    GPIO_Init(&usart2_gpio);                                         // Init TX pin

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;       // PA3 = RX
    GPIO_Init(&usart2_gpio);                                         // Init RX pin
}

// Simple software delay
void delay(void)
{
    for(uint32_t i=0;i<500000;i++);                                  // Busy wait
}

// Redirect printf output to USART2
int __io_putchar(int ch)
{
    USART_SendData(&usart2_handle, (uint8_t*)&ch, 1);                // Send one char
    return ch;                                                       // Return sent char
}

// Configure PA5 as output (LED)
void Led_Init(void)
{
    memset(&GpioLed,0,sizeof(GpioLed));                              // Reset struct

    GpioLed.pGPIOx = GPIOA;                                          // Use Port A
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;           // PA5
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;             // Output mode
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;          // Fast speed
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;         // Push-pull
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;        // Pull-up

    GPIO_PeriClockControl(GPIOA, ENABLE);                            // Enable GPIOA clock
    GPIO_Init(&GpioLed);                                             // Init PA5
}



int main(void)
{
    USART2_GPIOInit();                                               // Init USART2 pins
    USART2_Init();                                                   // Init USART2
    USART_PeripheralControl(USART2, ENABLE);                         // Enable USART2
    Led_Init();                                                      // Init LED pin
    timer2_1hz_interrupt_init();      // Initialize Timer2 for 1Hz with interrupt

    while(1)
    {
        // Main loop does nothing, work is handled in ISR
    }
}

// Timer2 user callback function
static void TIM2_Callback(void)
{
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);   // Toggle LED on PA5
     printf("Interrupt Generated\r\n");            // Print debug message
}

// Timer2 interrupt handler
void TIM2_IRQHandler(void)
{
   TIM2->SR &= ~(SR_UIF);        // Clear update interrupt flag
   TIM2_Callback();              // Call user callback
}
