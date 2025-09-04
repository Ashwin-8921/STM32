#include "stm32l4xx.h"
#include "stm32l4xx_timer_driver.h"

#define TIM2EN   (1U<<0)   // Enable TIM2 clock
#define TIM3EN   (1U<<1)   // Enable TIM3 clock


void timer2_1hz_init(void)
{
    RCC->APB1ENR1 |= TIM2EN;     // Turn on TIM2 clock
    TIM2->PSC = 16000 - 1;       // Divide 16 MHz to 1 kHz
    TIM2->ARR = 1000 - 1;        // Count 1000 ticks = 1 second
    TIM2->CNT = 0;               // Reset counter
    TIM2->CR1 = CR1_CEN;         // Start timer
}


