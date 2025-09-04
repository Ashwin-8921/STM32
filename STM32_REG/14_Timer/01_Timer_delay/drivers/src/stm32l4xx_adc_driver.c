#include "stm32l4xx.h"
#include"stm32l4xx_usart_driver.h"
#include<stdio.h>
#include<string.h>
#define RCC_AHB2ENR_ADCEN       (1U << 13)  // Bit 13: ADC clock enable
#define ADC_CR_ADEN             (1UL << 0U)      // ADC enable

#define ADC_CR_ADDIS            (1UL << 1U)     // ADC disable
#define ADC_CR_ADSTART          (1UL << 2U)   // Start conversion
#define ADC_CR_ADCAL            (1UL << 31U)     // Calibration
#define ADC_CR_ADVREGEN         (1UL << 28U)  // Regulator enable
#define ADC_CR_DEEPPWD          (1UL << 29U)   // Deep-power-down
#define ADC_CCR_CKMODE_Msk      (0x3UL << 16U)
#define ADC_CCR_CKMODE          ADC_CCR_CKMODE_Msk

#define ADC_CCR_CKMODE_DIV1     (0x1UL << 16U) // HCLK / 1
#define ADC_CCR_CKMODE_DIV2     (0x2UL << 16U) // HCLK / 2
#define ADC_ISR_ADRDY           (1UL << 0U)    // ADC ready
#define ADC_ISR_EOC             (1UL << 2U)      // End of conversion
#define ADC_SQR1_L_Msk          (0xFUL << 0U)
#define ADC_SQR1_L              ADC_SQR1_L_Msk                // Sequence length




void delay(void)
{
	for (volatile int i = 0; i < 1000; i++); // Delay ~20us
}
void adc_init(void)
{

    GPIOA_PCLK_EN();
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;     // ADC clock


    // ADC1_IN5 = PA0
    GPIOA->MODER |= (3U << (0 * 2));       // PA0 analog
    GPIOA->PUPDR &= ~(3U << (0 * 2));      // No pull
    GPIOA->ASCR  |= (1U << 0);             // Connect analog switch


    ADCOM->CCR &= ~ADC_CCR_CKMODE;
    ADCOM->CCR |= ADC_CCR_CKMODE_DIV1; // HCLK / 1


    ADC1->CR &= ~ADC_CR_DEEPPWD;       // Exit deep power-down
    ADC1->CR |= ADC_CR_ADVREGEN;       // Enable regulator
    delay();


    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    ADC1->SMPR1 &= ~(0x7U << (5 * 3));
    ADC1->SMPR1 |=  (0x2U << (5 * 3)); // 12.5 cycles


    ADC1->SQR1 &= ~ADC_SQR1_L;                   // 1 conversion
    ADC1->SQR1 &= ~(0x1FU << 6U);  // Clear SQ1
    ADC1->SQR1 |=  (5U << 6U);     // SQ1 = channel 5


    ADC1->ISR |= ADC_ISR_ADRDY;        // Clear ADRDY
    ADC1->CR  |= ADC_CR_ADEN;          // Enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

void start_conversion(void)
{
    /* Start ADC conversion */
    ADC1->CR |= (1<<2);
}

uint16_t adc_read(void)
{
    while (!(ADC1->ISR & ADC_ISR_EOC));    // Wait end of conversion
    return (uint16_t)(ADC1->DR);           // Return 12-bit result
}
