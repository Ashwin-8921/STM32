#include "stm32l4xx.h"
#include "stm32l4xx_systick_driver.h"
#include <string.h>
#include <stdio.h>

static void systick_callback(void);

USART_Handle_t usart2_handle;   // USART2 handle

// Setup USART2: 115200 baud, 8 data bits, no parity, 1 stop bit
void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;                                // Use USART2
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200; // Baud rate = 115200
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // Disable HW flow control
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;       // Enable TX and RX
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // 1 stop bit
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8-bit data
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // No parity
    USART_Init(&usart2_handle);                                    // Initialize USART2 with config
}

// Configure PA2 (TX) and PA3 (RX) for USART2
void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart2_gpio;
    memset(&usart2_gpio,0,sizeof(usart2_gpio));                    // Reset struct to zero

    usart2_gpio.pGPIOx = GPIOA;                                    // Use GPIOA
    usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;     // Set to alternate function
    usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   // Push-pull output
    usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  // Enable pull-up
    usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;    // Fast switching
    usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;             // AF7 = USART2

    GPIOA_PCLK_EN();                                               // Enable GPIOA clock

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;     // PA2 as TX
    GPIO_Init(&usart2_gpio);                                       // Init PA2

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;     // PA3 as RX
    GPIO_Init(&usart2_gpio);                                       // Init PA3
}

// Simple delay loop
void delay(void)
{
    for(uint32_t i=0;i<500000;i++);                                // Busy wait
}

// Redirect printf output to USART2
int __io_putchar(int ch)
{
    USART_SendData(&usart2_handle, (uint8_t*)&ch, 1);              // Send one char
    return ch;                                                     // Return sent char
}

int main()
{
    GPIO_Handle_t GpioLed;
    memset(&GpioLed, 0, sizeof(GpioLed));                          // Clear struct

    GpioLed.pGPIOx = GPIOA;                                        // Use GPIOA
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;         // PA5 pin
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;           // Set as output
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;        // Fast speed
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;       // Push-pull output
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;      // Enable pull-up

    GPIO_PeriClockControl(GPIOA, ENABLE);                          // Enable GPIOA clock
    GPIO_Init(&GpioLed);                                           // Init PA5 as output

    USART2_GPIOInit();                                             // Init PA2/PA3 for USART2
    USART2_Init();                                                 // Init USART2
    USART_PeripheralControl(USART2, ENABLE);                       // Enable USART2

    systick_1hz_Interrupt();                                       // Setup SysTick to trigger every 1s

    while(1)
    {
        // Main loop is empty, work is done in SysTick interrupt
    }
}

// Called every SysTick interrupt (1 second)
static void systick_callback(void)
{
     GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);                   // Toggle LED on PA5
     printf("one second passed\r\n");                              // Print message
}

// SysTick ISR
void SysTick_Handler(void)
{
    systick_callback();                                            // Call user callback
}
