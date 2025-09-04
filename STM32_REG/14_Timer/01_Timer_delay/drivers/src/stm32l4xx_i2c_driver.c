#include"stm32l4xx_i2c_driver.h"

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );

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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,
                        uint8_t *pTxBuffer,
                        uint32_t Len,
                        uint8_t SlaveAddr,
                        uint8_t Sr)
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,
                           uint8_t *pRxBuffer,
                           uint8_t Len,
                           uint8_t SlaveAddr,
                           uint8_t Sr)
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




// Burst write in blocking mode
void I2C_BurstWrite(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pData, uint32_t Len)
{
    // Allocate temp buffer (register addr + data)
    uint8_t *buffer = malloc(Len + 1);
    if (!buffer)
    	return;

    buffer[0] = RegAddr;               // First byte = register address
    memcpy(&buffer[1], pData, Len);    // Copy data after register

    // Send register + data to slave
    I2C_MasterSendData(pI2CHandle, buffer, Len + 1, SlaveAddr, I2C_DISABLE_SR);

    free(buffer);  // Free temp buffer
}



// Burst write in interrupt mode
uint8_t I2C_BurstWriteIT(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pData, uint32_t Len)
{
    uint8_t *buffer = malloc(Len + 1);
    if (!buffer)
    	return 1;   // Fail if no memory

    buffer[0] = RegAddr;               // First byte = register
    memcpy(&buffer[1], pData, Len);    // Rest = data

    // Start non-blocking send
    uint8_t status = I2C_MasterSendDataIT(pI2CHandle, buffer, Len + 1, SlaveAddr, I2C_DISABLE_SR);

    return status;  // Buffer freed later in ISR
}



// Burst read in blocking mode
void I2C_BurstRead(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pBuffer, uint32_t Len)
{
    // Send register address first (with repeated start)
    I2C_MasterSendData(pI2CHandle, &RegAddr, 1, SlaveAddr, I2C_ENABLE_SR);

    // Then receive data
    I2C_MasterReceiveData(pI2CHandle, pBuffer, Len, SlaveAddr, I2C_DISABLE_SR);
}

// Burst read in interrupt mode
uint8_t I2C_BurstReadIT(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *pBuffer, uint32_t Len)
{
    // First send register address (non-blocking, repeated start)
    uint8_t status = I2C_MasterSendDataIT(pI2CHandle, &RegAddr, 1, SlaveAddr, I2C_ENABLE_SR);

    // After TX complete, ISR will trigger RX automatically
    return status;
}






// Configure NVIC IRQ priority
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;             // Which register
	uint8_t iprx_section  = IRQNumber % 4;    // Which section
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/* ---------- I2C_MasterSendDataIT (fixed non-blocking master TX) ---------- */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,
                             uint8_t *pTxBuffer,
                             uint32_t Len,
                             uint8_t SlaveAddr,
                             uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        /* save context */
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen     = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr   = SlaveAddr;
        pI2CHandle->Sr        = Sr;

        /* wait until bus is free: BUSY bit in ISR is bit 15 -> ensure it is 0 */
        while( (pI2CHandle->pI2Cx->ISR & (1U << 15)) != 0 );

        /* Prepare CR2: clear SADD[9:0] and NBYTES[7:0] and relevant control bits first */
        uint32_t tmp = pI2CHandle->pI2Cx->CR2;
        tmp &= ~((0x3FFU << 0) | (0xFFU << 16) | (1U<<24) | (1U<<25) | (1U<<10)); /* clear SADD, NBYTES, RELOAD, AUTOEND, RD_WRN */
        tmp |= ((uint32_t)(SlaveAddr & 0x3FFU) << 1);
        tmp |= ((Len & 0xFFU) << 16);
        if(Sr == I2C_DISABLE_SR)
        {
            tmp |= (1U << 25); /* AUTOEND */
        }
        pI2CHandle->pI2Cx->CR2 = tmp;

        /* Generate START (CR2 START bit is bit 13) */
        pI2CHandle->pI2Cx->CR2 |= (1U << 13);

        /* Enable interrupts: TXIS (TX interrupt), ERRIE (error interrupts), STOPIE if desired */
        pI2CHandle->pI2Cx->CR1 |= (1U << I2C_CR1_TXIE) | (1U << I2C_CR1_ERRIE) | (1U << I2C_CR1_STOPIE);
    }

    return busystate;
}



// Non-blocking master receive
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        // Save context
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen     = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize    = Len;
        pI2CHandle->DevAddr   = SlaveAddr;
        pI2CHandle->Sr        = Sr;

        // Configure CR2 for reception
        pI2CHandle->pI2Cx->CR2  = 0;
        pI2CHandle->pI2Cx->CR2 |= (SlaveAddr << 1);
        pI2CHandle->pI2Cx->CR2 |= (Len << 16);
        pI2CHandle->pI2Cx->CR2 |= (1 << 10);  // Read mode
		pI2CHandle->pI2Cx->CR2 |= (1 << 13);  // Start condition
        if (Sr == 0) pI2CHandle->pI2Cx->CR2 |= (1 << 25); // Auto-stop

        // Enable I2C interrupts
        pI2CHandle->pI2Cx->CR1 |= ((1<<1) | (1<<6) | (1<<7));
    }

    return busystate;
}

/* ---------- I2C_MasterHandleTXEInterrupt (fixed TXE/TXIS handling) ---------- */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{
    /* TXIS (transmit interrupt status) indicates TXDR can be written.*/
    if(pI2CHandle->TxLen > 0)
    {
        pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
        pI2CHandle->pTxBuffer++;
        pI2CHandle->TxLen--;

        /* If this was the last byte and AUTOEND not used, nothing extra here:
           STOPF will be generated (if AUTOEND set then STOP auto generated).
           If TxLen==0 and AUTOEND not set, TC/STOP handling in EV IRQ will finalize. */
    }
    else
    {
        /* No data left — should not normally arrive here if NBYTES was set correctly */
    }
}



// Handle RXNE (data available)
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR; // Read last byte
		pI2CHandle->RxLen--;
	}
	else if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2) I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE); // NACK before last byte

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		if(pI2CHandle->Sr == I2C_DISABLE_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle); // Reset state
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT); // Notify app
	}
}



// Close RX transfer
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 1);  // Disable RXNEIE
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 6);  // Disable STOPIE

	// Reset handle state
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen     = 0;
	pI2CHandle->RxSize    = 0;

	// Re-enable ACK if it was enabled
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
}

// Close TX transfer
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 1);  // Disable TXEIE
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 6);  // Disable STOPIE

	// Reset handle state
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen     = 0;
}


/*static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	// Clear old slave address bits (SADD[9:0]) in CR2
	pI2Cx->CR2 &= ~(0x3FFU << 0);

	// Load new slave address (shifted left 1, as LSB = R/W bit)
	pI2Cx->CR2 |= ((uint32_t)SlaveAddr << 1) & (0x3FFU << 0);

	// Clear bit10 (RD_WRN = 0 → Write operation)
	pI2Cx->CR2 &= ~(1U << 10);
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    // Clear old slave address bits (SADD[9:0]) in CR2
    pI2Cx->CR2 &= ~(0x3FFU << 0);

    // Load new slave address (shifted left 1, as LSB = R/W bit)
    pI2Cx->CR2 |= ((uint32_t)SlaveAddr << 1) & (0x3FFU << 0);

    // Set bit10 (RD_WRN = 1 → Read operation)
    pI2Cx->CR2 |= (1U << 10);
}*/




/* ---------- I2C_EV_IRQHandling (cleaned event IRQ handler) ---------- */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t isr = pI2CHandle->pI2Cx->ISR;
   // uint32_t cr1 = pI2CHandle->pI2Cx->CR1;

    /* ADDR flag: clear by reading ISR and writing ICR if required.
       Macro I2C_ISR_ADDR (bit pos) and I2C_ICR_ADDRCF (clear bit) recommended. */
    if(isr & (1U << I2C_ISR_ADDR))
    {
        /* For STM32L4, ADDR is cleared by reading ISR then reading or writing ICR.
           We'll clear by writing the ADDRCF bit in ICR (use macro I2C_ICR_ADDRCF) */
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ICR_ADDRCF);
    }

    /* TXIS: peripheral asks for data (transmit) */
    if(isr & (1U << I2C_ISR_TXIS))
    {
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_MasterHandleTXEInterrupt(pI2CHandle);
        }
    }

    /* RXNE: data received (master RX) */
    if(isr & (1U << I2C_ISR_RXNE))
    {
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            /* read byte */
            *pI2CHandle->pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->RXDR;
            pI2CHandle->pRxBuffer++;
            pI2CHandle->RxLen--;

            if(pI2CHandle->RxLen == 0)
            {
                /* If AUTOEND used STOP will arrive and be handled below */
            	I2C_MasterHandleRXNEInterrupt(pI2CHandle );
            }
        }
    }

    /* STOPF: stop detected -> transfer complete (clear STOPF via ICR STOPCF) */
    if(isr & (1U << I2C_ISR_STOPF))
    {
        /* clear STOPF */
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ICR_STOPCF);

        /* finalize based on state */
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_CloseSendData(pI2CHandle);
            I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_CloseReceiveData(pI2CHandle);
            I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
        }
    }

    /* Optional: handle other events as needed (BTF, TC, RELOAD) */
}




// Handle error interrupts
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t isr = pI2CHandle->pI2Cx->ISR;
    uint32_t cr1 = pI2CHandle->pI2Cx->CR1;

    if((isr & (1U << 8)) && (cr1 & (1U << 7)))   // Bus error
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 8);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    if((isr & (1U << 9)) && (cr1 & (1U << 7)))   // Arbitration lost
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 9);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    if((isr & (1U << 10)) && (cr1 & (1U << 7)))  // Acknowledge failure
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 10);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    if((isr & (1U << 11)) && (cr1 & (1U << 7)))  // Overrun
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 11);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    if((isr & (1U << 12)) && (cr1 & (1U << 7)))  // Timeout
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 12);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}


/* ---------- I2C_ManageAcking (ensure ACK is CR1 bit) ---------- */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= (1U << I2C_CR1_ACK);
    }
    else
    {
        pI2Cx->CR1 &= ~(1U << I2C_CR1_ACK);
    }
}



