#include <string.h>
#include "stm32l4xx.h"
#include "stm32l4xx_usart_driver.h"

USART_Handle_t usart2_handle; // USART2 handle
uint8_t rx_data;              // variable to store received byte

// Configure USART2 peripheral
void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;                           // use USART2
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200; // baud = 115200
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // no RTS/CTS
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;  // enable TX + RX
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // 1 stop bit
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8-bit data
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // no parity
    USART_Init(&usart2_handle);  // apply config to hardware
}

// Configure PA2 (TX) and PA3 (RX) pins
void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;
    usart_gpios.pGPIOx = GPIOA;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;   // alternate function
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // push-pull output
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // enable pull-up
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // fast speed
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;           // AF7 for USART2

    // PA2 -> USART2_TX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_PeriClockControl(GPIOA, ENABLE); // enable GPIOA clock
    GPIO_Init(&usart_gpios);

    // PA3 -> USART2_RX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);
}

// Configure PA5 as LED output
void GPIO_LedInit(void)
{
    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // PA5
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;   // output mode
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);
}

int main(void)
{
    USART2_GPIOInit();    // setup TX/RX pins
    USART2_Init();        // setup USART2
    GPIO_LedInit();       // setup LED

    USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE); // enable USART2 interrupt in NVIC
    USART_PeripheralControl(USART2, ENABLE);         // enable USART2 module

    // Send startup message
    char init_msg[] = "UART Initialized\r\n";
    USART_SendData(&usart2_handle, (uint8_t*)init_msg, strlen(init_msg));

    // enable RX interrupt for 1 byte
    USART_ReceiveDataIT(&usart2_handle, &rx_data, 1);

    while (1)
    {

    }
}

// ISR for USART2
void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart2_handle); // delegate handling to driver
}

// Called when RX complete interrupt occurs
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
    if (ApEv == USART_EVENT_RX_CMPLT) // check if RX is complete
    {
        // notify via UART
        char msg[] = "Interrupt generated\r\n";
        USART_SendData(pUSARTHandle, (uint8_t*)msg, strlen(msg));

        // toggle LED if 'a' was received
        if (rx_data == 'a')
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

        // re-enable RX interrupt for next byte
        USART_ReceiveDataIT(pUSARTHandle, &rx_data, 1);
    }
}
