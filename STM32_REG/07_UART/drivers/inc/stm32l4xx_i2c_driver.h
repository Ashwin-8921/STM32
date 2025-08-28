

#ifndef INC_STM32L4XX_I2C_DRIVER_H_
#define INC_STM32L4XX_I2C_DRIVER_H_


#include"stm32l4xx.h"


// Configuration structure for I2C peripheral
typedef struct{
    uint8_t I2C_DeviceAddress;  // Own device address (if acting as slave)
    uint8_t I2C_ACKControl;     // ACK enable/disable after receiving data
} I2C_Config_t;


// Handle structure for I2C (used to store state and config of I2C peripheral)
typedef struct{
    I2C_RegDef_t *pI2Cx;       // Base address of the I2C peripheral (I2C1, I2C2, etc.)
    I2C_Config_t I2C_Config;   // Configuration settings (address, ACK control)

    uint8_t *pTxBuffer;        // Pointer to application transmit buffer
    uint8_t *pRxBuffer;        // Pointer to application receive buffer
    uint32_t TxLen;            // Length of transmit data
    uint32_t RxLen;            // Length of receive data

    uint8_t TxRxState;         // Current I2C state (ready, busy in TX, busy in RX)
    uint8_t DevAddr;           // Slave/device address (for master to communicate with)
    uint32_t RxSize;           // Size of receive buffer (to manage repeated reads)
    uint8_t Sr;                // Repeated start condition flag
} I2C_Handle_t;



/*
 *  I2C_SCL Speed
 */

#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000
#define I2C_SCL_SPEED_FM2K  200000


/*
 *  I2C_ACK Control
 */

#define I2C_ACK_ENABLE		 1
#define I2C_ACK_DISABLE		 0

#define I2C_DISABLE_SR       0
#define I2C_ENABLE_SR        1

#define I2C_READY 			 0
#define I2C_BUSY_IN_RX 		 1
#define I2C_BUSY_IN_TX 		 2




/*********************** API Supported by this driver **********************/


// Enable or disable clock for given I2C peripheral
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


// This function enables or disables the I2C peripheral
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


/* Init and De-init */

// Initialize I2C peripheral with settings from handle (address, ACK, speed, etc.)
void I2C_Init(I2C_Handle_t *pI2CHandle);

// Reset/de-initialize I2C peripheral (bring registers back to reset state)
void I2C_DeInit(I2C_RegDef_t *pI2Cx);




/* Data Send and Receive (blocking) */

// Master sends data (blocking) to given slave address
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

// Master receives data (blocking) from given slave address
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);




/* Data Send and Receive (interrupt / non-blocking) */

// Master sends data (non-blocking, using interrupts)
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

// Master receives data (non-blocking, using interrupts)
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);




/* Close communication */

// Close ongoing reception and reset handle state (used in IT mode)
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

// Close ongoing transmission and reset handle state (used in IT mode)
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);











#endif /* INC_STM32L4XX_I2C_DRIVER_H_ */
