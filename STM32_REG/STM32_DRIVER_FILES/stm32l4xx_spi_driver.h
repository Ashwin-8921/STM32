
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







#endif /* INC_STM32L4XX_SPI_DRIVER_H_ */
