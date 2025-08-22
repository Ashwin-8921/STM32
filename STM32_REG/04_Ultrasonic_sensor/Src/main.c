#include<stdint.h>
#include<string.h>
#include<stdio.h>
#include "stm32l4xx.h"
#include "stm32l4xx_gpio_driver.h"

// Trigger: PB6 (output)
// Echo:    PB7 (input)
// Debug:   PA5 (LED)


void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

int main(void)
{
    // === Configure Trigger (PB6) ===
    GPIO_Handle_t GpioTrig;
    GpioTrig.pGPIOx = GPIOB;
    GpioTrig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GpioTrig.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioTrig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioTrig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioTrig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_Init(&GpioTrig);

    // === Configure Echo (PB7) ===
    GPIO_Handle_t GpioEcho;
    GpioEcho.pGPIOx = GPIOB;
    GpioEcho.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GpioEcho.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioEcho.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioEcho.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioEcho);

    // === Debug LED (PA5) ===
    GPIO_Handle_t GpioLED;
    GpioLED.pGPIOx = GPIOA;
    GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLED);

    uint32_t pulse_width = 0;
    float distance = 0;

    while(1)
    {
        // Send 10µs trigger pulse
        GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, GPIO_PIN_SET);
        delay_us(10);
        GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, GPIO_PIN_RESET);

        // Wait for Echo HIGH
        while(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == 0);

        // Measure pulse width
        pulse_width = 0;
        while(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == 1) {
            pulse_width++;
            delay_us(1);
        }

        // Calculate distance (cm)
        distance = (pulse_width * 0.0343f) / 2;

        // Debug: Toggle LED if object is closer than 10cm
        if(distance < 10.0f) {
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);
        } else {
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_RESET);
        }

        delay_ms(50);
    }
}
