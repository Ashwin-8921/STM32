#include"stm32l4xx_usart_driver.h"


/*
 *  Peripheral Clock Control
 */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
	}
	else{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
	}
}



// Enables or disables the USART peripheral

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);  // Set UE bit to enable USART
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE); // Clear UE bit to disable USART
	}
}






/*
 *  USART Initialization Function
 */



void USART_Init(USART_Handle_t *pUSARTHandle)
{
    uint32_t tempreg = 0;

    // 1. Enable peripheral clock for the given USART
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    // 2. Configure USART mode (TX, RX, or both)
    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        tempreg |= (1 << USART_CR1_RE);   // Enable Receiver
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        tempreg |= (1 << USART_CR1_TE);   // Enable Transmitter
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE)); // Enable both TX & RX
    }

    // 3. Configure word length
    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    // 4. Configure parity control
    if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        tempreg |= (1 << USART_CR1_PCE);  // Enable Even parity
    }
    else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
    {
        tempreg |= (1 << USART_CR1_PCE);  // Enable Parity
        tempreg |= (1 << USART_CR1_PS);   // Select Odd parity
    }

    // Load CR1 register
    pUSARTHandle->pUSARTx->CR1 = tempreg;

    // 5. Configure stop bits
    tempreg = 0;
    tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
    pUSARTHandle->pUSARTx->CR2 = tempreg;

    // 6. Configure hardware flow control
    tempreg = 0;
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        tempreg |= (1 << USART_CR3_CTSE); // Enable CTS
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        tempreg |= (1 << USART_CR3_RTSE); // Enable RTS
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        tempreg |= (1 << USART_CR3_CTSE); // Enable CTS
        tempreg |= (1 << USART_CR3_RTSE); // Enable RTS
    }

    // Load CR3 register
    pUSARTHandle->pUSARTx->CR3 = tempreg;

    // 7. Configure baud rate
    pUSARTHandle->pUSARTx->BRR = 0x23;
   // USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}



/*
 *  UART Send Data
 */

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint16_t *pdata;

    // Loop through all bytes to be transmitted
    for (uint32_t i = 0; i < Len; i++)
    {
        // 1. Wait until TXE (Transmit Data Register Empty) flag is set
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

        // 2. Check word length
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            // If 9-bit data frame
            pdata = (uint16_t*) pTxBuffer;
            pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF); // Send 9 bits

            // If parity is disabled → 9 bits used (2 bytes), so move buffer by 2
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                pTxBuffer++;
                pTxBuffer++;
            }
            else
            {
                // If parity enabled , 8 data bits + 1 parity bit, so move buffer by 1
                pTxBuffer++;
            }
        }
        else
        {
            // If 8-bit data frame
            pUSARTHandle->pUSARTx->TDR = (*pTxBuffer & (uint8_t)0xFF); // Send 8 bits
            pTxBuffer++;  // Move to next byte
        }
    }

    // 3. Wait for TC (Transmission Complete) flag before returning
    while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}



/*
 *  UART Send Data
 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    // Loop until all bytes are received
    for (uint32_t i = 0; i < Len; i++)
    {
        // Wait until RXNE (Receive Data Register Not Empty) flag is set
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

        // Check word length
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            // 9-bit data frame
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity, all 9 bits are data
                *((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x01FF);
                pRxBuffer++;
                pRxBuffer++;   // Increment by 2 bytes
            }
            else
            {
                // Parity enabled, only 8 bits are data
                *pRxBuffer = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
                pRxBuffer++;  // Increment by 1 byte
            }
        }
        else
        {
            // 8-bit data frame
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity, all 8 bits are data
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);
            }
            else
            {
                // Parity enabled, only 7 bits are data
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);
            }
            pRxBuffer++;  // Increment by 1 byte
        }
    }
}

/*
 *  Send data over USART using interrupts
 */


uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    // Get current transmission state
    uint8_t txstate = pUSARTHandle->TxBusyState;

    // Check if USART is not already busy in transmission
    if(txstate != USART_BUSY_IN_TX)
    {
        // Save the buffer address and length in the handle
        pUSARTHandle->TxLen = Len;
        pUSARTHandle->pTxBuffer = pTxBuffer;

        // Mark USART as busy in transmission
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

        // Enable TXE (transmit data register empty) interrupt
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

        // Enable TC (transmission complete) interrupt
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }

    // Return previous transmission state (0 = ready, 1 = busy)
    return txstate;
}


/*
 *  Receive data over USART using interrupts
 */

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    // Get current reception state
    uint8_t rxstate = pUSARTHandle->RxBusyState;

    // Check if USART is not already busy in reception
    if(rxstate != USART_BUSY_IN_RX)
    {
        // Save the buffer address and length in the handle
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->pRxBuffer = pRxBuffer;

        // Mark USART as busy in reception
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        // Clear the receive data register to avoid reading old data
        (void)pUSARTHandle->pUSARTx->RDR;

        // Enable RXNE (receive data register not empty) interrupt
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }

    // Return previous reception state (0 = ready, 1 = busy)
    return rxstate;
}

/*
 * Interrupt Configuration for UART
 */



void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    // Check if we need to enable the IRQ
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            // Enable interrupt in NVIC ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            // Enable interrupt in NVIC ISER1 register for IRQ numbers 32-63
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            // Enable interrupt in NVIC ISER3 register for IRQ numbers 64-95
            *NVIC_ISER3 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        // Disable IRQ if EnorDi is not ENABLE
        if(IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ICER3 |= (1 << (IRQNumber % 64));
        }
    }
}

/*
 *  Interrupt Priority Configuration
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // Calculate which NVIC IPR register holds the priority for this IRQ
    uint8_t iprx = IRQNumber / 4;

    // Calculate the section within the IPR register (0-3)
    uint8_t iprx_section  = IRQNumber % 4;

    // Calculate the bit position to shift the priority value
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    // Set the priority for the given IRQ in the NVIC
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*
 *  Interrupt Handler Function
 */


void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
    uint32_t temp1, temp2, temp3;
    uint16_t *pdata;

    // ********** Handle Transmission Complete (TC) **********
    temp1 = pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_TC);   // Check TC flag
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE); // Check TC interrupt enable

    if(temp1 && temp2)
    {
        if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
        {
            if(!pUSARTHandle->TxLen)  // Transmission finished
            {
                pUSARTHandle->pUSARTx->ISR &= ~(1 << USART_ISR_TC);  // Clear TC flag
                pUSARTHandle->TxBusyState = USART_READY;            // Reset TX state
                pUSARTHandle->pTxBuffer = NULL;
                pUSARTHandle->TxLen = 0;
                USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT); // Notify app
            }
        }
    }

    temp1 = 0; temp2 = 0;

    // ********** Handle TXE (Transmit Data Register Empty) **********
    temp1 = pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_TXE);   // Check TXE flag
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE); // Check TXE interrupt enable

    if(temp1 && temp2)
    {
        if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
        {
            if(pUSARTHandle->TxLen > 0)
            {
                // Handle 9-bit word length
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
                {
                    pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
                    pUSARTHandle->pUSARTx->TDR = (*pdata & 0x01FF); // Load 9 bits

                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        pUSARTHandle->pTxBuffer += 2; // Move buffer by 2 bytes
                        pUSARTHandle->TxLen -= 2;
                    }
                    else
                    {
                        pUSARTHandle->pTxBuffer++; // Move buffer by 1 byte (parity bit used)
                        pUSARTHandle->TxLen -= 1;
                    }
                }
                else // Handle 8-bit word
                {
                    pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTxBuffer & 0xFF);
                    pUSARTHandle->pTxBuffer++;
                    pUSARTHandle->TxLen -= 1;
                }
            }

            if(pUSARTHandle->TxLen == 0)
            {
                pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE); // Disable TXE interrupt
            }
        }
    }

    temp1 = 0; temp2 = 0;

    // ********** Handle RXNE (Receive Data Register Not Empty) **********
    temp1 = pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_RXNE);   // Check RXNE flag
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE); // Check RXNE interrupt enable

    if(temp1 && temp2)
    {
        if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
        {
            if(pUSARTHandle->RxLen > 0)
            {
                // Handle 9-bit word length
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
                {
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        *((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & 0x01FF);
                        pUSARTHandle->pRxBuffer += 2;
                        pUSARTHandle->RxLen -= 2;
                    }
                    else
                    {
                        *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->RDR & 0xFF);
                        pUSARTHandle->pRxBuffer++;
                        pUSARTHandle->RxLen -= 1;
                    }
                }
                else // Handle 8-bit word
                {
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & 0xFF);
                    }
                    else
                    {
                        *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & 0x7F);
                    }
                    pUSARTHandle->pRxBuffer++;
                    pUSARTHandle->RxLen -= 1;
                }
            }

            if(!pUSARTHandle->RxLen)
            {
                pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE); // Disable RXNE interrupt
                pUSARTHandle->RxBusyState = USART_READY;
                USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT); // Notify app
            }
        }
    }

    temp1 = 0; temp2 = 0;

    // ********** Handle CTS (Clear to Send) **********
    temp1 = pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_CTS);
    temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
    temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

    if(temp1 && temp2 && temp3)
    {
        pUSARTHandle->pUSARTx->ISR &= ~(1 << USART_ISR_CTS); // Clear CTS flag
        USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
    }

    temp1 = 0; temp2 = 0; temp3 = 0;

    // ********** Handle IDLE line detection **********
    temp1 = pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_IDLE);
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

    if(temp1 && temp2)
    {
        pUSARTHandle->pUSARTx->ISR &= ~(1 << USART_ISR_IDLE); // Clear IDLE flag
        USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
    }

    temp1 = 0; temp2 = 0;

    // ********** Handle Overrun Error (ORE) **********
    temp1 = pUSARTHandle->pUSARTx->ISR & USART_ISR_ORE;
    temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

    if(temp1 && temp2)
    {
        USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
    }

    temp1 = 0; temp2 = 0;

    // ********** Handle other USART errors **********
    temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE); // Error interrupt enable
    if(temp2)
    {
        temp1 = pUSARTHandle->pUSARTx->ISR;

        if(temp1 & (1 << USART_ISR_FE))  // Framing error
        {
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
        }

        if(temp1 & (1 << USART_ISR_NF))  // Noise error
        {
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
        }

        if(temp1 & (1 << USART_ISR_ORE)) // Overrun error
        {
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
        }
    }

    temp1 = 0; temp2 = 0;
}


// Checks the status of a given USART flag
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint8_t StatusFlagName)
{
	if(pUSARTx ->ISR & StatusFlagName)
	{
		return SET;
	}
	return RESET;
}


// Clears the specified USART status flag

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    pUSARTx->ISR &= ~(StatusFlagName);  // Clear the flag by writing 0
}

// Callback function called by the USART

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
