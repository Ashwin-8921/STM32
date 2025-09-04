#include "stm32l4xx.h"
#include "stm32l4xx_timer_driver.h"

#define TIM2EN   (1U<<0)   // Enable TIM2 clock
#define TIM3EN   (1U<<1)   // Enable TIM3 clock
#define CR1_CEN  (1U<<0)   // Counter enable
#define GPIOA_EN (1U<<0)   // Enable GPIOA clock
#define DIER_UIE (1U<<0)

void timer2_1hz_init(void)
{
    RCC->APB1ENR1 |= TIM2EN;     // Turn on TIM2 clock
    TIM2->PSC = 4000 - 1;       // Divide 4 MHz to 1 kHz
    TIM2->ARR = 1000 - 1;        // Count 1000 ticks = 1 second
    TIM2->CNT = 0;               // Reset counter
    TIM2->CR1 = CR1_CEN;         // Start timer
}

void timer2_1hz_interrupt_init(void)
{
    RCC->APB1ENR1 |= TIM2EN;     // Turn on TIM2 clock
    TIM2->PSC = 4000 - 1;       // Divide 4 MHz to 1 kHz
    TIM2->ARR = 1000 - 1;        // Count 1000 ticks = 1 second
    TIM2->CNT = 0;               // Reset counter
    TIM2->CR1 = CR1_CEN;         // Start timer
    TIM2->DIER |= DIER_UIE;
    GPIO_IRQInterruptConfig(28, ENABLE);

}


