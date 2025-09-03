#include "stm32l4xx.h"
#include"stm32l4xx_usart_driver.h"
#include<stdio.h>
#include<string.h>




USART_Handle_t usart2_handle;

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart2_gpio;
	memset(&usart2_gpio,0,sizeof(usart2_gpio));

	usart2_gpio.pGPIOx = GPIOA;
	usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;  // TX
	GPIOA_PCLK_EN();
	GPIO_Init(&usart2_gpio);

	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;  // RX
	GPIO_Init(&usart2_gpio);
}

void delay2(void)
{
	for(uint32_t i = 0 ; i < 250000 ; i ++);
}

uint16_t adc_val;  // Variable to hold ADC result

int main(void)
{
	char msg[30];


    USART2_GPIOInit();
    USART2_Init();
    USART_PeripheralControl(USART2, ENABLE);
    adc_init();
    start_conversion();

    while (1)
    {
        adc_val = adc_read(); // Read ADC value (0 – 4095 for 12-bit ADC)

        // Format the ADC value as a string
        sprintf(msg,"ADC-Value: %u\r\n", adc_val);


        // Transmit the string over USART2
        USART_SendData(&usart2_handle, (uint8_t*) msg, strlen(msg));

        delay2(); // Small delay between transmissions
    }
}


