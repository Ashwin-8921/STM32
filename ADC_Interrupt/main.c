#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "uart.h"

ADC_HandleTypeDef hadc1;
uint32_t sensor_value;

int main()
{
    HAL_Init();
    uart2_init();
    adc_pa0_interrupt_init();

    HAL_ADC_Start_IT(&hadc1);

    while(1)
    {
        printf("The sensor value : %d \n\r", (int)sensor_value);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

    sensor_value = pa0_adc_read();
}
