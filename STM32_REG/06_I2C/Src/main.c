#include "stm32l4xx.h"
#include "stm32l4xx_i2c_driver.h"

#define SLAVE_ADDR  0x68   // I2C slave device address
#define MY_ADDR     0x61   // STM32 I2C own address

I2C_Handle_t I2C1Handle;

uint8_t some_data[] = "hello from STM";  // Data to send via I2C

// Initialize I2C1 GPIO pins (SCL and SDA)
void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;   // Alternate function
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // Open-drain
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;           // AF4 for I2C
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_PeriClockControl(GPIOB, ENABLE);

    // SCL pin PB6
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    // SDA pin PB7
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&I2CPins);
}

// Initialize I2C1 peripheral
void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;  // Enable ACK
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;       // STM32 own address

    I2C_Init(&I2C1Handle);
}

// Simple delay function
void delay(void)
{
    for(uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
    // Initialize I2C1 GPIO and peripheral
    I2C_PeriClockControl(I2C1, ENABLE);
    I2C1_GPIOInits();
    I2C1_Inits();
    I2C_PeripheralControl(I2C1, ENABLE);

    while(1)
    {
        // Send data "I2C" to slave device via I2C
        I2C_MasterSendData(&I2C1Handle, some_data, sizeof(some_data)-1, SLAVE_ADDR, 0);

        // Delay between transmissions
        delay();
    }
}
