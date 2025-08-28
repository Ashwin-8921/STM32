
#include"stm32l4xx_spi_driver.h"



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


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{




}



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

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len >0)
	{
		// wait until TXE is set
		while(((pSPIx->SR >> 1)&1)==0);

		// load the data in to the DR
		pSPIx->DR =*pTxBuffer;
		Len--;
		pTxBuffer++;
	}

}






void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len>0)
		{
			while(((pSPIx->SR >>0)&1)==0);

			*(pRxBuffer) = pSPIx -> DR;
			Len--;
			pRxBuffer++;
		}


}



void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << 6);
	}else
	{
		pSPIx->CR1 &=  ~(1 << 6);
	}
}

void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << 8);
	}else
	{
		pSPIx->CR1 &=  ~(1 << 8);
	}
}

void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << 2);
	}else
	{
		pSPIx->CR2 &=  ~(1 << 2);
	}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		pSPIHandle->pSPIx->CR2 |= ( 1 << 7 );
	}
	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |= ( 1 << 6 );
	}
	return state;
}




/*
 *  IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi)
{

}
void SPI_PRIORITY_CONFIG(uint8_t IRQNumber,uint8_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

