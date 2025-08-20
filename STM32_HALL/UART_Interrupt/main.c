#include "stm32l4xx_hal.h"

UART_HandleTypeDef huart1;

uint8_t tx_buffer[] = "Hello UART!\r\n";
uint8_t rx_byte;
uint32_t rx_counter = 0, tx_counter = 0;

void uart_init(void);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    tx_counter++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    rx_counter++;
    HAL_UART_Transmit_IT(&huart1, &rx_byte, 1);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

int main(void) {
    HAL_Init();
    uart_init();
    HAL_UART_Transmit_IT(&huart1, tx_buffer, sizeof(tx_buffer) - 1);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    while (1) {
    }
}

void uart_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Error_Handler(void) { }
