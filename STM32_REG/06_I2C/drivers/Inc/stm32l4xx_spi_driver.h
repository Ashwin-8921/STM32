
#ifndef INC_STM32L4XX_SPI_DRIVER_H_
#define INC_STM32L4XX_SPI_DRIVER_H_

#include <stdint.h>
#include "stm32l4xx.h"

// Configuration structure for SPI peripheral
typedef struct{
    uint8_t SPI_DeviceMode;   // Master or Slave mode
    uint8_t SPI_BusConfig;    // Full-duplex, Half-duplex, or Simplex RX only
    uint8_t SPI_SclkSpeed;    // Clock speed (baud rate prescaler)
    uint8_t SPI_DFF;          // Data frame format (8-bit or 16-bit)
    uint8_t SPI_CPOL;         // Clock polarity
    uint8_t SPI_CPHA;         // Clock phase
    uint8_t SPI_SSM;          // Software slave management (enabled/disabled)
}SPI_Config_t;


// Handle structure for SPI (stores everything needed for SPI communication)
typedef struct{
    SPI_RegDef_t *pSPIx;     // Pointer to SPI peripheral base address
    SPI_Config_t SPIConfig;   // SPI configuration settings
    uint8_t *pTxBuffer;       // Pointer to transmit buffer
    uint8_t *pRxBuffer;       // Pointer to receive buffer
    uint32_t TxLen;           // Length of transmission data
    uint32_t RxLen;           // Length of reception data
    uint8_t TxState;          // Transmission state (busy/ready)
    uint8_t RxState;          // Reception state (busy/ready)
}SPI_Handle_t;


/*
 *  SPI Device modes
 */

#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0


/*
 *  SPI Bus Configurations
 */

#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 *  SPI SCLK Speed
 */

#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


/*
 *  SPI DFF
 */

#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1


/*
 *  CPOL
 */

#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/*
 *  CPHA
 */

#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0


/*
 *  SPI SSM
 */

#define SPI_SSM_DI							0
#define SPI_SSM_EN							1


/*SPI SSM*/
#define SPI_SSM_DI							0
#define SPI_SSM_EN							1

#define SPI_READY 							0
#define SPI_BUSY_IN_RX 						1
#define SPI_BUSY_IN_TX 						2

#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					4



/*********************** API Supported by this driver **********************/

/*
 *  Peripheral Clock configuration
 */

void SPI_PeriClockControl(SPI_RegDef_t *pGPIOx , uint8_t EnorDi);




/*
 *  Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHANDLE);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 *   Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);



/*
 *  IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void SPI_PRIORITY_CONFIG(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);



void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_PRIORITY_CONFIG(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi);



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32L4XX_SPI_DRIVER_H_ */
