/* This code uses DMA to transfer data from a source buffer to a destination buffer
   and sends the transferred data via UART2 to a serial terminal. */

#include "main.h"

// UART handle
UART_HandleTypeDef huart2;
// DMA handle for memory-to-memory transfer
DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

// Buffers for DMA transfer
uint8_t src_buffer[100];
uint8_t dst_buffer[100];

// Function prototypes
void SystemClock_Config(void);     // Configures the system clock
static void MX_GPIO_Init(void);    // Initializes GPIO pins
static void MX_DMA_Init(void);     // Configures and initializes DMA
static void MX_USART2_UART_Init(void); // Configures and initializes UART2

int main(void) // Main function where program execution starts
{
  HAL_Init();                 // Initialize HAL library
  SystemClock_Config();       // Configure system clock
  MX_GPIO_Init();             // Initialize GPIO
  MX_DMA_Init();              // Initialize DMA
  MX_USART2_UART_Init();      // Initialize UART2

  // Fill source buffer with 'A' to 'A'+26
  for (int i = 0; i < 27; i++) {
    src_buffer[i] = i + 'A';
  }

  // Start DMA transfer from src_buffer to dst_buffer
  HAL_DMA_Start(&hdma_memtomem_dma1_channel1,
                (uint32_t)src_buffer,
                (uint32_t)dst_buffer,
                100);

  // Wait until DMA transfer completes
  HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_channel1, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);

  // Send confirmation message via UART
  HAL_UART_Transmit(&huart2, (uint8_t *)"DMA Transfer Done\r\n", 19, HAL_MAX_DELAY);
  // Send the copied buffer via UART
  HAL_UART_Transmit(&huart2, dst_buffer, 100, HAL_MAX_DELAY);

  while (1) {} // Infinite loop
}

void SystemClock_Config(void) // Sets up the clock source and frequency
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Configure HSI and PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Configure system clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

static void MX_USART2_UART_Init(void) // Initializes UART2 for serial communication
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
}

static void MX_DMA_Init(void) // Configures DMA1 Channel1 for memory-to-memory transfer
{
  __HAL_RCC_DMA1_CLK_ENABLE(); // Enable DMA1 clock

  // Configure DMA1 channel for memory-to-memory transfer
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_0;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_memtomem_dma1_channel1);
}

static void MX_GPIO_Init(void) // Configures GPIO pins for LED and button
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // LED pin default LOW
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  // Button input
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  // LED output
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void) // Handles errors by stopping program execution
{
  __disable_irq();
  while (1) {} // Stay here on error
}
