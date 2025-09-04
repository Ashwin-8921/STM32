#include"stm32l4xx.h"
#include"stm32l4xx_usart_driver.h"
#include"stm32l4xx_gpio_driver.h"
#include<stdio.h>
#include<string.h>

#define TX_BUFFER_SIZE 50
uint8_t tx_buffer[TX_BUFFER_SIZE] = "Hello DMA UART TX!\r\n";  // DMA transmit buffer

USART_Handle_t usart2_handle;   // USART2 handle structure

// USART2 peripheral setup
void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;                                 // Select USART2
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;  // Baud = 115200
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // No HW flow ctrl
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;        // Enable TX + RX
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // 1 stop bit
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8-bit data
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // No parity
	USART_Init(&usart2_handle);                                     // Apply config
}

// GPIO config for USART2: PA2 (TX) & PA3 (RX)
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart2_gpio;
	memset(&usart2_gpio,0,sizeof(usart2_gpio));                     // Clear struct

	usart2_gpio.pGPIOx = GPIOA;                                      // Use Port A
	usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;       // Alternate function
	usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;     // Push-pull
	usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;    // Pull-up
	usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;      // Fast speed
	usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;               // AF7 = USART2

	GPIOA_PCLK_EN();                                                 // Enable Port A clock

	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;       // PA2 as TX
	GPIO_Init(&usart2_gpio);

	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;       // PA3 as RX
	GPIO_Init(&usart2_gpio);
}

// Simple delay loop
void delay()
{
	for(uint32_t i=0;i<500000;i++);
}

// Redirect printf() to USART2
int __io_putchar(int ch)
{
	USART_SendData(&usart2_handle, (uint8_t*)&ch, 1);  // Send one char
	return(ch);
}

// DMA setup for USART2 TX
void uart2_dma_tx_init(void)
{
    RCC->AHB1ENR |= (1<<0);                         // Enable DMA1 clock

    DMA1_CSELR->CSELR &= ~(0xF << (4*(7-1)));       // Reset CH7 mapping
    DMA1_CSELR->CSELR |=  (0x02000000);             // Map CH7 → USART2_TX

    DMA1_Channel7->CCR &= ~(1<<0);                  // Disable CH7 before config
    DMA1_Channel7->CPAR  = (uint32_t)&USART2->TDR;  // Peripheral = USART2->TDR
    DMA1_Channel7->CMAR  = (uint32_t)tx_buffer;     // Memory source = tx_buffer
    DMA1_Channel7->CNDTR = strlen((char*)tx_buffer); // Transfer length
    DMA1_Channel7->CCR   = (1<<4)|(1<<7)|(1<<13);   // DIR=mem→periph, MINC=1, high priority

    USART2->CR3 |= (1<<7);                          // Enable USART2 DMA TX

    DMA1_Channel7->CCR |= (1<<0);                   // Enable CH7
}

int main()
{
	USART2_GPIOInit();                              // Setup USART2 pins
	USART2_Init();                                  // Init USART2 peripheral
	USART_PeripheralControl(USART2, ENABLE);        // Enable USART2
	printf("Hello World\r\n");                      // Send msg via USART2
	uart2_dma_tx_init();                            // Start DMA transfer

	while(1)
	{
	    while((DMA1_Channel7->CNDTR) != 0);         // Wait until DMA transfer done

	    DMA1_Channel7->CCR &= ~(1<<0);              // Disable CH7
	    strcpy((char*)tx_buffer, "DMA TX AGAIN\r\n"); // Update buffer
	    DMA1_Channel7->CNDTR = strlen((char*)tx_buffer); // Reload transfer count
	    DMA1_Channel7->CCR |= (1<<0);               // Restart DMA
	    delay();                                    // Small wait
	}
}
