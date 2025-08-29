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



void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |=(1<<USART_CR1_UE);
	}else{
		pUSARTx->CR1 &=~(1<<USART_CR1_UE);
	}
}




uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint8_t StatusFlagName)
{
	if(pUSARTx ->ISR & StatusFlagName)
	{
		return SET;
	}
	return RESET;
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






