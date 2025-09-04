#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32l4xx.h"
#include "stm32l4xx_gpio_driver.h"
#include "stm32l4xx_spi_driver.h"
#include "stm32l4xx_usart_driver.h"

SPI_Handle_t SPI2Handle;
uint8_t data = 10;

USART_Handle_t usart2_handle;

// Simple blocking delay
void delay(void)
{
    for(volatile int i = 0; i < 25000; i++);   // Waste time in loop
}

// Init USART2 GPIO (PA2 TX, PA3 RX)
void USART2_GPIOInits(void)
{
    GPIO_Handle_t usart2_gpio;
    memset(&usart2_gpio,0,sizeof(usart2_gpio)); // Reset struct

    usart2_gpio.pGPIOx = GPIOA;
    usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;   // Alternate function
    usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-pull
    usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;// Pull-up
    usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Fast I/O
    usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;           // AF7 = USART2

    GPIOA_PCLK_EN(); // Enable port A clock

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;   // TX pin
    GPIO_Init(&usart2_gpio);

    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;   // RX pin
    GPIO_Init(&usart2_gpio);
}

// Init USART2 peripheral
void USART2_Inits(void)
{
    usart2_handle.pUSARTx = USART2;                          // Use USART2
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200; // Baud rate 115200
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // No flow ctrl
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX; // Enable TX and RX
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // 1 stop bit
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8 bits
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // No parity
    USART_Init(&usart2_handle); // Apply config
}

// Init SPI2 GPIO (PB12..PB15)
void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;   // Alternate function
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;           // AF5 = SPI2
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-pull
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;// No pull
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;  // High speed
    GPIO_PeriClockControl(GPIOB, ENABLE);                    // Enable port B clock

    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;  // NSS
    GPIO_Init(&SPIPins);

    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;  // SCK
    GPIO_Init(&SPIPins);

    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;  // MISO
    GPIO_Init(&SPIPins);

    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;  // MOSI
    GPIO_Init(&SPIPins);
}

// Init SPI2 peripheral
void SPI2_Inits(void)
{
    SPI2Handle.pSPIx = SPI2;                        // Use SPI2
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER; // Master mode
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;       // Full duplex
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;    // PCLK/32
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;                 // 8-bit data
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;                 // CPOL = 0
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;                 // CPHA = 0
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;                    // Hardware NSS
    SPI_Init(&SPI2Handle);                                        // Apply config
}

// SPI event callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    if(AppEv == SPI_EVENT_TX_CMPLT)                 // TX complete event
    {
        char msg[] = "SPI TX Complete!\r\n";
        USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg)); // Print via UART
    }
}

// Main function
int main(void)
{
    USART2_GPIOInits();                         // Init USART2 pins
    USART2_Inits();                             // Init USART2 peripheral
    USART_PeripheralControl(USART2, ENABLE);    // Enable USART2

    SPI2_GPIOInits();                           // Init SPI2 pins
    SPI2_Inits();                               // Init SPI2 peripheral

    SPI_SSOEConfig(SPI2, ENABLE);               // Enable NSS output
    SPI_PeripheralControl(SPI2, ENABLE);        // Enable SPI2

    SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);// Enable SPI2 interrupt

    char start_msg[] = "SPI Interrupt Example Started\r\n";
    USART_SendData(&usart2_handle, (uint8_t*)start_msg, strlen(start_msg)); // Print start msg

    SPI_SendDataIT(&SPI2Handle, &data, 1);      // Send 1 byte via SPI interrupt

    while(1)
    {
        // Wait here, work done in interrupt
    }
}

// SPI2 interrupt handler
void SPI2_IRQHandler(void)
{
    const char msg[] = "SPI2 Interrupt generated\r\n";
    USART_SendData(&usart2_handle, (uint8_t*)msg, sizeof(msg)-1); // Debug print
    SPI_IRQHandling(&SPI2Handle); // Handle SPI2 events
}
