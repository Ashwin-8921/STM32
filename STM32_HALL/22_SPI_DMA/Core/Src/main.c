/*
 * This program sends and receives 10 bytes of data using SPI1 with DMA on STM32.
 * SPI1 is configured as master, and DMA is used for efficient data transfer.
 */

#include "main.h"

SPI_HandleTypeDef hspi1;       // SPI1 handle
DMA_HandleTypeDef hdma_spi1_rx; // DMA handle for SPI1 RX
DMA_HandleTypeDef hdma_spi1_tx; // DMA handle for SPI1 TX

uint8_t tx_buffer[10] = {10, 20, 20, 40, 50, 60, 70, 90, 90, 100}; // Data to send
uint8_t rx_buffer[10];                                            // Buffer for received data

// Function prototypes
void SystemClock_Config(void);  // Configure system clock
static void MX_DMA_Init(void);  // Initialize DMA
static void MX_SPI1_Init(void); // Initialize SPI1

int main(void)
{
  HAL_Init();              // Initialize HAL library and reset peripherals
  SystemClock_Config();    // Set up clock system
  MX_DMA_Init();           // Enable and configure DMA
  MX_SPI1_Init();          // Configure SPI1 in master mode

  // Start SPI transmit/receive using DMA
  HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buffer, rx_buffer, 10);

  while (1)
  {

  }
}

void SystemClock_Config(void) // Sets up PLL and system clock speed
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); // Power scaling

  // Configure internal HSI oscillator and PLL
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

  // Configure CPU and peripheral bus clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

static void MX_SPI1_Init(void) // Configures SPI1 for master, full-duplex, 8-bit
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  HAL_SPI_Init(&hspi1);
}

static void MX_DMA_Init(void) // Enables DMA clock and configures interrupts
{
  __HAL_RCC_DMA1_CLK_ENABLE(); // Enable DMA1 clock

  // Configure DMA interrupts for SPI1 TX and RX
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void Error_Handler(void) // Runs if an error occurs
{
  __disable_irq(); // Disable interrupts
  while (1) { }    // Stay here forever
}
