#include"stm32l4xx_i2c_driver.h"


// Function to generate a START condition on I2C bus
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    // Set the START bit (bit 13) in CR2 register
    pI2Cx->CR2 |= (1 << 13);
}

// Function to generate a STOP condition on I2C bus
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    // Set the STOP bit (bit 14) in CR2 register
    pI2Cx->CR2 |= (1 << 14);
}






// Enable or disable the given I2C peripheral
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Set bit 0 (PE - Peripheral Enable) in CR1 → I2C ON
        pI2Cx->CR1 |= (1 << 0);
    }
    else
    {
        // Clear bit 0 (PE) in CR1 → I2C OFF
        pI2Cx->CR1 &= ~(1 << 0);
    }
}



// Enable or disable peripheral clock for the given I2C instance
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        // Turn ON clock for the selected I2C peripheral
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();   // Enable clock for I2C1
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();   // Enable clock for I2C2
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();   // Enable clock for I2C3
        }
    }
    else
    {
        // Turn OFF clock for the selected I2C peripheral
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();   // Disable clock for I2C1
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();   // Disable clock for I2C2
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();   // Disable clock for I2C3
        }
    }
}


// Initialize I2C peripheral
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    // 1. Set ACK control (bit 15 of CR2)
    tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 15;
    pI2CHandle->pI2Cx->CR2 |= tempreg;

    // 2. Set timing for ~100kHz I2C speed
    pI2CHandle->pI2Cx->TIMINGR |= 0x00411313;

    // 3. Set device own address (shifted left by 1 for 7-bit addr)
    tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    pI2CHandle->pI2Cx->OAR1 |= tempreg;
}


// Master sends data in blocking mode
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
    // 1. Wait until I2C is not busy (BUSY flag in ISR, bit 15)
    while (((pI2CHandle->pI2Cx->ISR >> 15) & 1));

    // 2. Prepare CR2 register: set slave address + number of bytes
    uint32_t cr2 = 0;
    pI2CHandle->pI2Cx->CR2 |= (SlaveAddr << 1) | (Len << 16);

    // 3. If no repeated start (Sr = disable), set AUTOEND (bit 25)
    if (Sr == I2C_DISABLE_SR)
    {
        cr2 |= (1 << 25);
    }
    pI2CHandle->pI2Cx->CR2 |= cr2;

    // 4. Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 5. Send all bytes
    while (Len > 0)
    {
        // Wait until TX buffer is empty (TXE, bit 0 in ISR)
        while (!(pI2CHandle->pI2Cx->ISR & (1 << 0)));

        // Write data to TXDR
        pI2CHandle->pI2Cx->TXDR = *pTxBuffer;

        // Move to next byte
        pTxBuffer++;
        Len--;
    }

    // 6. Handle STOP condition or repeated start
    if (Sr == I2C_DISABLE_SR)
    {
        // Wait until STOP flag (bit 5 in ISR) is set
        while (!(pI2CHandle->pI2Cx->ISR & (1 << 5)));

        // Clear STOP flag by writing to ICR
        pI2CHandle->pI2Cx->ICR |= (1 << 5);
    }
    else
    {
        // If repeated start, wait for TC (transfer complete, bit 6)
        while (!(pI2CHandle->pI2Cx->ISR & (1 << 6)));
    }
}





// Master receives data in blocking mode
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
    // 1. Wait until the bus is free (BUSY flag = 0, bit 15 in ISR)
    while (((pI2CHandle->pI2Cx->ISR >> 15) & 1) == 0);

    // 2. Configure CR2 register
    uint32_t cr2 = 0;
    cr2 |= (SlaveAddr << 1);   // Slave address
    cr2 |= (Len << 16);        // Number of bytes to read
    cr2 |= (0 << 10);          // Set direction = read (RD_WRN = 1 normally, 0 = write)
    if (Sr == I2C_DISABLE_SR)
    {
        cr2 |= (1 << 25);      // AUTOEND = 1 (stop condition after transfer)
    }

    // Load CR2 with settings
    pI2CHandle->pI2Cx->CR2 |= cr2;

    // 3. Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 4. Read all bytes
    while (Len > 0)
    {
        // Wait until RXNE flag is set (bit 2 in ISR)
        while (!(pI2CHandle->pI2Cx->ISR & (1 << 2)));

        // Read received byte from RXDR
        *pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->RXDR;

        // Move buffer pointer and reduce length
        pRxBuffer++;
        Len--;
    }

    // 5. Handle STOP or repeated start
    if (Sr == I2C_DISABLE_SR)
    {
        // Wait until STOP flag (bit 5 in ISR) is set
        while (!(pI2CHandle->pI2Cx->ISR & (1 << 5)));

        // Clear STOP flag in ICR
        pI2CHandle->pI2Cx->ICR |= (1 << 5);
    }
    else
    {
        // If repeated start, wait for transfer complete (bit 6 in ISR)
        while (!(pI2CHandle->pI2Cx->ISR & (1 << 6)));
    }
}





// Configure (enable/disable) NVIC interrupt for given IRQ number
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)   // Enable interrupt
    {
        if(IRQNumber <= 31)
        {
            // IRQ 0–31 → set bit in NVIC ISER0
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            // IRQ 32–63 → set bit in NVIC ISER1
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            // IRQ 64–95 → set bit in NVIC ISER3
            *NVIC_ISER3 |= (1 << (IRQNumber % 64));
        }
    }
    else   // Disable interrupt
    {
        if(IRQNumber <= 31)
        {
            // IRQ 0–31 → clear bit in NVIC ICER0
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            // IRQ 32–63 → clear bit in NVIC ICER1
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            // IRQ 64–95 → clear bit in NVIC ICER3
            *NVIC_ICER3 |= (1 << (IRQNumber % 64));
        }
    }
}


