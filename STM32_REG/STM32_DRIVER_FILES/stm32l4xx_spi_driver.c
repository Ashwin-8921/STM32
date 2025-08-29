
#include"stm32l4xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHANDLE);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHANDLE);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHANDLE);

/*
 *  Peripheral Clock configuration
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
				if(pSPIx == SPI1)
				{
					SPI1_PCLK_EN();
				}
				else if(pSPIx==SPI2)
				{
					SPI2_PCLK_EN();
				}
				else if(pSPIx==SPI3)
				{
					SPI3_PCLK_EN();
				}
			}
			else
			{
				if(pSPIx == SPI1)
						{
							SPI1_PCLK_DI();
						}
						else if(pSPIx==SPI2)
						{
							SPI2_PCLK_DI();
						}
						else if(pSPIx==SPI3)
						{
							SPI3_PCLK_DI();
						}

			}

}




/*
 *  Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHANDLE)
{
	// Configure the SPI_CR1 Register
	uint32_t tempreg=0;

			SPI_PeriClockControl(pSPIHANDLE->pSPIx, ENABLE);

			// Configure the device mode
			tempreg |=pSPIHANDLE->SPIConfig.SPI_DeviceMode<<2;

			//Configure the Bus config
			if(pSPIHANDLE->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD)
			{
				//  BIDI mode should be cleared
				tempreg &=~(1<<15);
			}
			else if(pSPIHANDLE->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD)
			{
				//  BIDI mode should be set
				tempreg |=(1<<15);
			}
			else if(pSPIHANDLE->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY)
			{
				// BIDI mode should be cleared
				tempreg &=~(1<<15);

				// RXONLY bit must be set
				tempreg |=(1<<10);
			}

			// Configure the SPI Serial Clock Speed(baud rate)
			tempreg |=pSPIHANDLE->SPIConfig.SPI_SclkSpeed<<3;

			// Configure the CPOL
			tempreg |=pSPIHANDLE->SPIConfig.SPI_CPOL<<1;

			// Configure the CPHA
			tempreg |=pSPIHANDLE->SPIConfig.SPI_CPHA<<0;

			pSPIHANDLE->pSPIx->CR1 |=tempreg;

			tempreg=0;

			//// Configure the DFF
			tempreg |=pSPIHANDLE->SPIConfig.SPI_DFF<<8;

			pSPIHANDLE->pSPIx->CR2 |=tempreg;


}


void SPI_DeInit(SPI_RegDef_t *pSPIx);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t*pSPIx, uint32_t FlagName)
{
	if(pSPIx -> SR &FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 *   Data Send and Receive
 */

// Blocking (polling) send data function
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		// Wait until TXE (Transmit buffer empty) flag is set
		while(((pSPIx->SR >> 1) & 1) == 0);

		// Load data into Data Register (DR)
		pSPIx->DR = *pTxBuffer;

		Len--;          // Decrement length
		pTxBuffer++;    // Move buffer pointer
	}
}

// Blocking (polling) receive data function
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		// Wait until RXNE (Receive buffer not empty) flag is set
		while(((pSPIx->SR >> 0) & 1) == 0);

		// Read data from Data Register into buffer
		*(pRxBuffer) = pSPIx->DR;

		Len--;         // Decrement length
		pRxBuffer++;   // Move buffer pointer
	}
}

// Enable or disable SPI peripheral
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << 6);  // Set SPE bit
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 6);  // Clear SPE bit
	}
}

// Configure SSI (Internal Slave Select)
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << 8);  // Set SSI bit
	}
	else
	{
		pSPIx->CR1 &= ~(1 << 8);  // Clear SSI bit
	}
}

// Configure SSOE (Slave Select Output Enable)
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << 2);  // Set SSOE bit
	}
	else
	{
		pSPIx->CR2 &= ~(1 << 2);  // Clear SSOE bit
	}
}

// Interrupt-based (non-blocking) send data function
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)  // Check if SPI is free
	{
		// Save Tx buffer info in handle
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable TXEIE (TX buffer empty interrupt)
		pSPIHandle->pSPIx->CR2 |= (1 << 7);
	}
	return state; // Return previous state
}

// Interrupt-based (non-blocking) receive data function
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)  // Check if SPI is free
	{
		// Save Rx buffer info in handle
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable RXNEIE (RX buffer not empty interrupt)
		pSPIHandle->pSPIx->CR2 |= (1 << 6);
	}
	return state; // Return previous state
}


/*
 *  IRQ Configuration and ISR Handling
 */

// Enable or disable SPI interrupt in NVIC
void SPI_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		// NVIC ISER0 for IRQ 0-31
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		// NVIC ISER1 for IRQ 32-63
		else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		// NVIC ISER3 for IRQ 64-95
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else   // Disable interrupt
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

// Set priority for SPI interrupt
void SPI_PRIORITY_CONFIG(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;              // Find IPR register
	uint8_t iprx_section  = IRQNumber % 4;     // Section within register

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	// Write priority into register
	*(NVIC_PR_BASE_ADDR + iprx) |=  (IRQPriority << shift_amount);
}

// Main SPI IRQ handling (checks different interrupt sources)
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	// TXE (Transmit buffer empty)
	temp1 = pHandle->pSPIx->SR & (1 << 1);
	temp2 = pHandle->pSPIx->CR2 & (1 << 7);
	if(temp1 & temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	// RXNE (Receive buffer not empty)
	temp1 = pHandle->pSPIx->SR & (1 << 0);
	temp2 = pHandle->pSPIx->CR2 & (1 << 6);
	if(temp1 & temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	// OVR (Overrun error)
	temp1 = pHandle->pSPIx->SR & (1 << 6);
	temp2 = pHandle->pSPIx->CR2 & (1 << 5);
	if(temp1 & temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

// Handle TXE interrupt (send next byte)
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer; // Write data to DR
	pSPIHandle->TxLen--;                            // Decrement length
	pSPIHandle->pTxBuffer++;

	if(!pSPIHandle->TxLen) // Transmission finished
	{
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

// Handle RXNE interrupt (read received byte)
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
	pSPIHandle->RxLen--;
	pSPIHandle->pRxBuffer++;

	if(!pSPIHandle->RxLen) // Reception finished
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

// Handle Overrun Error interrupt
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		// Clear OVR flag by reading DR and SR
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

// Clear OVR flag manually
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

// Close transmission (reset state, disable interrupt)
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << 7);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

// Close reception (reset state, disable interrupt)
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << 6);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

// Weak callback function (user can override in application)
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	// User application should implement this
}
