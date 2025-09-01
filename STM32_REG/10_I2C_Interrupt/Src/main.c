#include "stm32l4xx.h"
#include "stm32l4xx_i2c_driver.h"
#include "stm32l4xx_usart_driver.h"
#include <string.h>

#define SLAVE_ADDR  0x68   // I2C slave device address
#define MY_ADDR     0x61   // Own I2C address

I2C_Handle_t I2C1Handle;       // I2C handle structure
USART_Handle_t usart2_handle;  // USART handle structure

uint8_t some_data[] = "hello from STM";   // Data to send via I2C

// ------------------- USART2 GPIO Initialization -------------------
void USART2_GPIOInits(void)
{
    GPIO_Handle_t usart2_gpio;
    memset(&usart2_gpio,0,sizeof(usart2_gpio)); // Clear structure

    usart2_gpio.pGPIOx = GPIOA;                           // Use GPIOA pins
    usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;   // Alternate function mode
    usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-pull
    usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;// Pull-up
    usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Fast speed
    usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;           // USART2 AF7

    // PA2 -> USART2 TX
    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIOA_PCLK_EN();
    GPIO_Init(&usart2_gpio);

    // PA3 -> USART2 RX
    usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart2_gpio);
}

// ------------------- USART2 Peripheral Initialization -------------------
void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;  // Select USART2
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200; // Baudrate
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX; // Enable TX and RX
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}

// ------------------- I2C1 GPIO Initialization -------------------
void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;                         // I2C uses GPIOB
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // Open-drain
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;  // I2C1 AF4
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_PeriClockControl(GPIOB, ENABLE);           // Enable GPIOB clock

    // PB6 -> I2C1 SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    // PB7 -> I2C1 SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&I2CPins);
}

// ------------------- I2C1 Peripheral Initialization -------------------
void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;          // Select I2C1
    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE; // Enable ACK
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;     // Own address
    I2C_Init(&I2C1Handle);
}

// ------------------- Small Delay -------------------
void delay(void)
{
    for(uint32_t i = 0; i < 250000; i++);
}

// ------------------- I2C Application Callback -------------------
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    if(AppEv == I2C_EV_TX_CMPLT)   // Transmission complete event
    {
        uint8_t msg1[] = "[I2C] TX Complete Interrupt Triggered\r\n";
        USART_SendData(&usart2_handle, msg1, sizeof(msg1)-1);

        delay();
        // Send data again in interrupt mode
        I2C_MasterSendDataIT(pI2CHandle, some_data, sizeof(some_data)-1, SLAVE_ADDR, 0);
    }
    else if(AppEv == I2C_ERROR_AF) // NACK received event
    {
        uint8_t msg2[] = "[I2C] NACK Received (AF Error)\r\n";
        USART_SendData(&usart2_handle, msg2, sizeof(msg2)-1);
    }
}

// ------------------- Main Function -------------------
int main(void)
{
    // Initialize USART2 for debugging messages
    USART2_GPIOInits();
    USART2_Init();
    USART_PeripheralControl(USART2, ENABLE);

    uint8_t start_msg[] = "Starting I2C Transmission via Interrupts...\r\n";
    USART_SendData(&usart2_handle, start_msg, sizeof(start_msg)-1);

    // Initialize I2C1
    I2C_PeriClockControl(I2C1, ENABLE);
    I2C1_GPIOInits();
    I2C1_Inits();
    I2C_PeripheralControl(I2C1, ENABLE);

    // Enable I2C interrupts (event + error)
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    // Start sending data via I2C (interrupt mode)
    I2C_MasterSendDataIT(&I2C1Handle, some_data, sizeof(some_data)-1, SLAVE_ADDR, 0);

    while(1)
    {
        // Infinite loop - main work is done in interrupts
    }
}

// ------------------- IRQ Handlers -------------------
void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C1Handle); // Handle I2C event interrupt
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&I2C1Handle); // Handle I2C error interrupt
}
