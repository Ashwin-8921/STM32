/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Flash + HC-SR04 Logger (Write & Read)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * Licensed under terms in LICENSE file.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define W25Q_WRITE_ENABLE    0x06
#define W25Q_PAGE_PROGRAM    0x02
#define W25Q_READ_DATA       0x03
#define W25Q_READ_STATUS     0x05
#define W25Q_JEDEC_ID        0x9F
#define W25Q_SECTOR_ERASE    0x20

// HC-SR04 pins (with 5V->3V3 divider required on ECHO)
#define TRIG_PIN    GPIO_PIN_0
#define TRIG_PORT   GPIOA
#define ECHO_PIN    GPIO_PIN_1
#define ECHO_PORT   GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FLASH_SIZE_BYTES   (4UL * 1024UL * 1024UL)  // 4MB
#define SECTOR_SIZE_BYTES  (4UL * 1024UL)           // 4KB
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;

uint32_t flash_addr = 0; // start address for logging
char uart_buf[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM2_Init(void);

void Delay_us(uint16_t us);

void W25Q_WriteEnable(void);
uint8_t W25Q_ReadStatus(void);
void W25Q_WaitBusy(void);
void W25Q_PageProgram(uint32_t addr, uint8_t* data, uint16_t len);
void W25Q_ReadData(uint32_t addr, uint8_t* buf, uint16_t len);
void W25Q_SectorErase(uint32_t addr);
uint32_t W25Q32_ReadID(void);

uint32_t HCSR04_Read_cm(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void W25Q_SectorErase(uint32_t addr)
{
  uint8_t cmd[4];
  W25Q_WriteEnable();
  cmd[0] = W25Q_SECTOR_ERASE;
  cmd[1] = (addr >> 16) & 0xFF;
  cmd[2] = (addr >> 8) & 0xFF;
  cmd[3] = addr & 0xFF;
  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
  W25Q_WaitBusy();
}

void W25Q_WriteEnable(void)
{
  uint8_t cmd = W25Q_WRITE_ENABLE;
  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
}

uint8_t W25Q_ReadStatus(void)
{
  uint8_t cmd = W25Q_READ_STATUS;
  uint8_t status = 0;
  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi2, &status, 1, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
  return status;
}

void W25Q_WaitBusy(void)
{
  while (W25Q_ReadStatus() & 0x01) { /* BUSY bit */ }
}

void W25Q_PageProgram(uint32_t addr, uint8_t* data, uint16_t len)
{
  uint8_t cmd[4];
  W25Q_WriteEnable();
  cmd[0] = W25Q_PAGE_PROGRAM;
  cmd[1] = (addr >> 16) & 0xFF;
  cmd[2] = (addr >> 8) & 0xFF;
  cmd[3] = addr & 0xFF;

  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, data, len, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
  W25Q_WaitBusy();
}

void W25Q_ReadData(uint32_t addr, uint8_t* buf, uint16_t len)
{
  uint8_t cmd[4];
  cmd[0] = W25Q_READ_DATA;
  cmd[1] = (addr >> 16) & 0xFF;
  cmd[2] = (addr >> 8) & 0xFF;
  cmd[3] = addr & 0xFF;

  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi2, buf, len, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
}

uint32_t W25Q32_ReadID(void)
{
  uint8_t cmd = W25Q_JEDEC_ID;
  uint8_t id[3] = {0};
  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi2, id, 3, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
  return (id[0] << 16) | (id[1] << 8) | id[2];
}

/* TIM2 is configured 1 MHz -> 1 tick = 1 us */
void Delay_us(uint16_t us)
{
  uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
  while (((__HAL_TIM_GET_COUNTER(&htim2) - start) & 0xFFFF) < us) { }
}

/* HC-SR04 one-shot distance (cm) using busy-wait timing */
uint32_t HCSR04_Read_cm(void)
{
    uint32_t t1, t2;

    // 10 µs TRIG pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    Delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // Wait for ECHO high
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET);

    t1 = __HAL_TIM_GET_COUNTER(&htim2);

    // Wait for ECHO low
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET);

    t2 = __HAL_TIM_GET_COUNTER(&htim2);

    // Compute pulse width (with wrap handling)
    uint32_t pulse = (t2 >= t1) ? (t2 - t1) : (0x10000 - t1 + t2);

    return (pulse / 58U); // convert µs → cm
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2); // start microsecond timer

  sprintf(uart_buf, "W25Q32 + HC-SR04 Logger Start\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

  uint32_t id = W25Q32_ReadID();
  sprintf(uart_buf, "JEDEC ID: 0x%06lX\r\n", id);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

  // Erase first sector before first write to ensure clean 0xFF state
  W25Q_SectorErase(flash_addr);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1) Read distance
    uint32_t dist_cm = HCSR04_Read_cm();

    // 2) Print over UART
    sprintf(uart_buf, "Distance: %lu cm\r\n", dist_cm);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    // 3) Append to flash as 4-byte little-endian
    uint8_t four[4];
    four[0] = (uint8_t)(dist_cm & 0xFF);
    four[1] = (uint8_t)((dist_cm >> 8) & 0xFF);
    four[2] = (uint8_t)((dist_cm >> 16) & 0xFF);
    four[3] = (uint8_t)((dist_cm >> 24) & 0xFF);

    // If crossing a sector boundary and rewriting same area often, erase that sector first
    if ((flash_addr % SECTOR_SIZE_BYTES) == 0)
    {
      W25Q_SectorErase(flash_addr);
    }

    W25Q_PageProgram(flash_addr, four, 4);

    // Optional: read back and print
    uint8_t back[4];
    W25Q_ReadData(flash_addr, back, 4);
    uint32_t verify = (uint32_t)back[0] | ((uint32_t)back[1] << 8) | ((uint32_t)back[2] << 16) | ((uint32_t)back[3] << 24);
    sprintf(uart_buf, "Logged: %lu cm @ 0x%06lX\r\n", verify, flash_addr);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    flash_addr += 4;
    if (flash_addr >= FLASH_SIZE_BYTES) flash_addr = 0;  // wrap around

    HAL_Delay(1000); // 1 Hz logging
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 80 MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // 80 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;          // 80 MHz -> TIM2 gets 80 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // 80 MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 4 */
static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80 - 1;              // 80 MHz / 80 = 1 MHz -> 1 us per tick
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;                 // 16-bit free running
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE END 4 */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /* FLASH CS: PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* HC-SR04 TRIG: PA0 output */
  GPIO_InitStruct.Pin = TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

  /* HC-SR04 ECHO: PA1 input (with external 5V->3V3 divider on hardware) */
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) { }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
