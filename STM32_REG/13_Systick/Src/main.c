#include "stm32l4xx.h"                  
#include "stm32l4xx_systick_driver.h"   
#include <string.h>                     

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

    // 5. Infinite loop: Toggle LED with delay
    while(1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);  // Toggle LED state (ON → OFF or OFF → ON)
        systickDelayMs(500);                         // Delay for 500ms using SysTick timer
    }
}
