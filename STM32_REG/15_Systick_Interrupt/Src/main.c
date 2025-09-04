#include "stm32l4xx.h"
#include "stm32l4xx_systick_driver.h"
#include <string.h>
#include <stdio.h>

static void systick_callback(void);

USART_Handle_t usart2_handle;   // USART2 handle structure

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


int main()
{
    GPIO_Handle_t GpioLed;

    // 1. Reset the GPIO handle structure to avoid garbage values
    memset(&GpioLed, 0, sizeof(GpioLed));

    // 2. Configure GPIO settings for LED
    GpioLed.pGPIOx = GPIOA;                              // Select GPIO Port A
    GpioLed.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_5;   // Use Pin A5 (on many boards, this is the user LED)
    GpioLed.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_OUT;   // Set pin mode as Output
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST; // Set output speed (fast switching capability)
    GpioLed.GPIO_PinConfig.GPIO_PinOPType   = GPIO_OP_TYPE_PP; // Output type: Push-Pull (standard output)
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  // Enable Pull-Up resistor (not strictly needed for LED, but safe)

    // 3. Enable the peripheral clock for GPIOA before using it
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // 4. Initialize GPIOA Pin 5 with the above configuration
    GPIO_Init(&GpioLed);

    USART2_GPIOInit();                                               // Init USART2 pins
    USART2_Init();                                                   // Init USART2
    USART_PeripheralControl(USART2, ENABLE);                         // Enable USART2

   systick_1hz_Interrupt();
    while(1)
    {

    }
}

static void systick_callback(void)
{
	 GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);  // Toggle LED on PA5
	 printf("one second passed\r\n");             // Print message
}

void SysTick_Handler(void)
{
	systick_callback();                           // Call SysTick callback
}
