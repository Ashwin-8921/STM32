#include "stm32l4xx.h"


void delay1(void)
{
    // Short delay
	for (volatile int i = 0; i < 1000; i++);
}

void adc_init(void)
{
    // --- Step 1: Enable peripheral clocks ---
    GPIOA_PCLK_EN();                       // GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;     // ADC1 clock

    // --- Step 2: Configure PA0 as ADC input (channel 5) ---
    GPIOA->MODER |= (3U << (0 * 2));       // Analog mode
    GPIOA->PUPDR &= ~(3U << (0 * 2));      // No pull-up/pull-down
    GPIOA->ASCR  |= (1U << 0);             // Enable analog switch

    // --- Step 3: Set ADC common clock ---
    ADCOM->CCR &= ~ADC_CCR_CKMODE;
    ADCOM->CCR |= ADC_CCR_CKMODE_DIV1;     // Clock = HCLK / 1

    // --- Step 4: Power up ADC ---
    ADC1->CR &= ~ADC_CR_DEEPPWD;           // Disable deep power-down
    ADC1->CR |= ADC_CR_ADVREGEN;           // Enable voltage regulator
    delay1();                              // Wait for regulator

    // --- Step 5: Calibrate ADC ---
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);       // Wait until calibration ends

    // --- Step 6: Configure sampling time for channel 5 ---
    ADC1->SMPR1 &= ~(0x7U << (5 * 3));
    ADC1->SMPR1 |=  (0x2U << (5 * 3));     // 12.5 ADC cycles

    // --- Step 7: Configure sequence (1 conversion, channel 5) ---
    ADC1->SQR1 &= ~ADC_SQR1_L;             // Sequence length = 1
    ADC1->SQR1 &= ~(0x1FU << 6U);          // Clear SQ1
    ADC1->SQR1 |=  (5U << 6U);             // SQ1 = channel 5

    // --- Step 8: Enable ADC ---
    ADC1->ISR |= ADC_ISR_ADRDY;            // Clear ready flag
    ADC1->CR  |= ADC_CR_ADEN;              // Enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY));  // Wait until ADC is ready
}

void start_conversion(void)
{
    ADC1->CFGR |= (1 << 13);   // Continuous conversion mode
    ADC1->CR   |= ADC_CR_ADSTART; // Start conversion
}

uint32_t adc_read(void)
{
    while (!(ADC1->ISR & ADC_ISR_EOC)); // Wait for conversion to complete
    return ADC1->DR;                    // Return data register value
}



