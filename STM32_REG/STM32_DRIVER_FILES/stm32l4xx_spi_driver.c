
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

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}


/*
 *   Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len)
{

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{

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

