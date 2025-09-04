#include "stm32l4xx.h"
#include "stm32l4xx_timer_driver.h"
#include "stm32l4xx_usart_driver.h"
#include "stm32l4xx_gpio_driver.h"
#include <stdio.h>
#include <string.h>

USART_Handle_t usart2_handle;   // USART2 handle

// Configure USART2 with 115200 baud, 8 data bits, no parity, 1 stop bit
void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;  // Baud = 115200
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // No flow control
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;        // Enable TX and RX
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // 1 stop bit
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8-bit data
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // No parity
    USART_Init(&usart2_handle); // Apply config
}

// Configure GPIOA pins PA2=TX, PA3=RX for USART2
void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart2_gpio;
    memset(&usart2_gpio,0,sizeof(usart2_gpio)); // Reset config struct

    usart2_gpio.pGPIOx = GPIOA;
    usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;     // Alt function mode
    usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   // Push-pull
    usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  // Pull-up
    usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;    // Fast speed
    usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;             // AF7 = USART2

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2; // PA2 as TX
    GPIOA_PCLK_EN();                // Enable GPIOA clock
    GPIO_Init(&usart2_gpio);        // Init PA2

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3; // PA3 as RX
    GPIO_Init(&usart2_gpio);        // Init PA3
}

// Small delay loop (software delay)
void delay()
{
    for(uint32_t i=0; i<500000; i++);
}

// Redirect printf() to USART2 by sending characters one by one
int __io_putchar(int ch)
{
    USART_SendData(&usart2_handle, (uint8_t*)&ch, 1); // Send one character
    return ch;                                        // Return same char
}

int val;   // Variable to hold captured value

int main()
{


    USART2_GPIOInit();                       // Init PA2/PA3 for USART2
    USART2_Init();                           // Configure USART2
    USART_PeripheralControl(USART2, ENABLE); // Enable USART2 peripheral

    timer2_pa5_output_compare();             // Setup TIM2 to toggle PA5 (test signal)
    timer3_pa6_input_capture();              // Setup TIM3 to capture signal on PA6

    while(1)
    {
        while(!(TIM2->SR & SR_CC1IF));       // Wait for TIM2 compare event (toggle on PA5)

        val = TIM3->CCR1;                    // Read captured counter value from TIM3 CH1

        printf("Timestamp: %d\r\n", val); // Print captured value to serial terminal

        delay();                             // Add delay to avoid flooding USART output
    }
}
