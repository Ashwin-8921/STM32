
#ifndef INC_STM32L4XX_UART_DRIVER_H_
#define INC_STM32L4XX_UART_DRIVER_H_

#include"stm32l4xx.h"

// Structure to hold USART configuration settings
typedef struct {
    uint8_t  USART_Mode;          // Mode: Transmit only, Receive only, or both
    uint32_t USART_Baud;          // Baud rate (speed of communication)
    uint8_t  USART_NoOfStopBits;  // Number of stop bits (1, 1.5, or 2)
    uint8_t  USART_WordLength;    // Word length (data bits: 8 or 9)
    uint8_t  USART_ParityControl; // Parity setting (none, even, odd)
    uint8_t  USART_HWFlowControl; // Hardware flow control (RTS, CTS, or none)
} USART_Config_t;


// Structure to handle a USART instance (configuration + data)
typedef struct {
    USART_RegDef_t *pUSARTx;     // Pointer to USART peripheral base address
    USART_Config_t USART_Config; // Configuration settings for this USART

    uint8_t *pTxBuffer;  // Pointer to transmission buffer
    uint8_t *pRxBuffer;  // Pointer to reception buffer
    uint32_t TxLen;      // Length of transmission data
    uint32_t RxLen;      // Length of reception data

    uint8_t TxBusyState; // Transmission state (busy or ready)
    uint8_t RxBusyState; // Reception state (busy or ready)
} USART_Handle_t;

/*
 *  USART Device Modes
 */

#define USART_MODE_ONLY_TX 			0
#define USART_MODE_ONLY_RX 			1
#define USART_MODE_TXRX  			2


/*
 *  Standard Baud Rate
 */

#define USART_STD_BAUD_1200  		1200
#define USART_STD_BAUD_2400  		2400
#define USART_STD_BAUD_9600  		9600
#define USART_STD_BAUD_19200  		19200
#define USART_STD_BAUD_38400  		38400
#define USART_STD_BAUD_57600  		57600
#define USART_STD_BAUD_115200  		115200
#define USART_STD_BAUD_230400  		230400
#define USART_STD_BAUD_460800  		460800
#define USART_STD_BAUD_921600  		921600
#define USART_STD_BAUD_2M 			2000000
#define USART_STD_BAUD_3M			3000000



/*
 *  Parity Control Options
 */


#define USART_PARITY_EN_ODD 		2
#define USART_PARITY_EN_EVEN 		1
#define USART_PARITY_DISABLE 		0

/*
 *  Word Length Options
 */

#define USART_WORDLEN_8BITS 		0
#define USART_WORDLEN_9BITS 		1


/*
 *   Stop Bits Options
 */

#define USART_STOPBITS_1     		0
#define USART_STOPBITS_0_5   		1
#define USART_STOPBITS_2     		2
#define USART_STOPBITS_1_5   		3


/*
 *   Hardware Flow Control Options
 */

#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS     	1
#define USART_HW_FLOW_CTRL_RTS     	2
#define USART_HW_FLOW_CTRL_CTS_RTS 	3



#define USART_FLAG_TXE 				( 1 << USART_ISR_TXE)
#define USART_FLAG_RXNE 			( 1 << USART_ISR_RXNE)
#define USART_FLAG_TC 				( 1 << USART_ISR_TC)


#define USART_BUSY_IN_RX 			1
#define USART_BUSY_IN_TX 			2
#define USART_READY 				0


#define 	USART_EVENT_TX_CMPLT   	0
#define		USART_EVENT_RX_CMPLT   	1
#define		USART_EVENT_IDLE      	2
#define		USART_EVENT_CTS       	3
#define		USART_EVENT_PE        	4
#define		USART_ERR_FE     		5
#define		USART_ERR_NE    	 	6
#define		USART_ERR_ORE    		7


/*********************** API Supported by this driver **********************/


/*
 *  Peripheral Clock configuration
 */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


// Enable or disable a USART peripheral
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


// Initializes the given USART peripheral with configuration settings
void USART_Init(USART_Handle_t *pUSARTHandle);

// Blocking (polling) API to send data
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);

// Blocking (polling) API to receive data
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

// Non-blocking (interrupt-based) API to send data
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);

// Non-blocking (interrupt-based) API to receive data
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

// Configure (enable/disable) a USART interrupt in NVIC
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

// Set priority for a given USART interrupt
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// Handle USART interrupt (called inside ISR)
void USART_IRQHandling(USART_Handle_t *pHandle);


// Check if a given status flag is set in USART (ex: TXE, RXNE, etc.)
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);

// Clear a specific status flag
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);

// Application callback (weak implementation, user can override for events)
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);



#endif /* INC_STM32L4XX_UART_DRIVER_H_ */
