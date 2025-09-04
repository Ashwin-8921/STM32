#include "stm32l4xx.h"
#include "stm32l4xx_timer_driver.h"

#define TIM2EN   (1U<<0)   // Enable TIM2 clock
#define TIM3EN   (1U<<1)   // Enable TIM3 clock
#define CR1_CEN  (1U<<0)   // Counter enable
#define GPIOA_EN (1U<<0)   // Enable GPIOA clock

void timer2_1hz_init(void)
{
    RCC->APB1ENR1 |= TIM2EN;     // Turn on TIM2 clock
    TIM2->PSC = 16000 - 1;       // Divide 16 MHz to 1 kHz
    TIM2->ARR = 1000 - 1;        // Count 1000 ticks = 1 second
    TIM2->CNT = 0;               // Reset counter
    TIM2->CR1 = CR1_CEN;         // Start timer
}


void timer2_pa5_output_compare(void)
{
    RCC->AHB2ENR |= GPIOA_EN;            // Enable GPIOA clock

    GPIOA->MODER &= ~(3U << 10);         // Clear PA5 mode bits
    GPIOA->MODER |=  (2U << 10);         // Set PA5 to alternate function

    GPIOA->AFR[0] &= ~(0xF << 20);       // Clear AFR for PA5
    GPIOA->AFR[0] |=  (1U << 20);        // AF1 = TIM2_CH1

    RCC->APB1ENR1 |= TIM2EN;             // Enable TIM2 clock

    TIM2->PSC = 4000 - 1;               // Prescaler: 16 MHz → 1 kHz
    TIM2->ARR = 1000 - 1;                // Auto-reload: 1000 ticks = 1 Hz

    TIM2->CCMR1 &= ~(7U << 4);           // Clear channel 1 mode
    TIM2->CCMR1 |=  (3U << 4);           // Set toggle mode on CH1

    TIM2->CCER |= (1U << 0);             // Enable channel 1 output

    TIM2->CNT = 0;                       // Reset counter
    TIM2->CR1 = CR1_CEN;                 // Enable TIM2
}



void timer3_pa6_input_capture(void)
{
    RCC->AHB2ENR |= GPIOA_EN;             // Enable GPIOA clock

    GPIOA->MODER &= ~(3U << 12);          // Clear mode bits for PA6
    GPIOA->MODER |=  (2U << 12);          // Set PA6 to alternate function mode

    GPIOA->AFR[0] &= ~(0xF << 24);        // Clear AF bits for PA6
    GPIOA->AFR[0] |=  (2U << 24);         // Select AF2 (TIM3_CH1) for PA6

    RCC->APB1ENR1 |= TIM3EN;              // Enable TIM3 clock

    TIM3->PSC = 4 - 1;                   // Set prescaler: 16 MHz / 16 = 1 MHz timer clock

    TIM3->CCMR1 &= ~(3U << 0);            // Clear CC1S bits
    TIM3->CCMR1 |=  (1U << 0);            // Map TI1 input to channel 1 (input capture mode)

    TIM3->CCER &= ~(1U << 1);             // Capture on rising edge (CC1P = 0)
    TIM3->CCER |=  (1U << 0);             // Enable capture on channel 1

    TIM3->CR1 = CR1_CEN;                  // Enable TIM3 counter
}



