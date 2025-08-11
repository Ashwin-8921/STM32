#include <stdio.h>

#include "stm32l4xx_hal.h"


#define BTN_PORT    GPIOC
#define BTN_PIN     GPIO_PIN_13
#define LED_PORT    GPIOA
#define LED_PIN     GPIO_PIN_5

void gpio_pc13_interrupt_init(void);

int main()
{
    HAL_Init();
    gpio_pc13_interrupt_init();

    while(1)
    {

    }
}

void gpio_pc13_interrupt_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure PC13 as input with interrupt on rising edge
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);

    // Configure PA5 as output for LED
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // Configure EXTI interrupt
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Toggle LED when button interrupt occurs
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}



