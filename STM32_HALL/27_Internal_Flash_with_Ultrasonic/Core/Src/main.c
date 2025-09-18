#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Flash memory configuration
#define FLASH_USER_START_ADDR  0x080FF800UL   // Flash page start address
#define FLASH_PAGE_SIZE        2048           // Page size = 2 KB
#define NUM_SLOTS              (FLASH_PAGE_SIZE / 8) // Each write = 8 bytes (doubleword)

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

// Ultrasonic capture variables
uint32_t ic_val1 = 0;
uint32_t ic_val2 = 0;
uint32_t diff = 0;
uint8_t is_first_capture = 0;
uint32_t distance_cm = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

// Send trigger pulse to HC-SR04 (10 µs high)
void HCSR04_Trigger(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  for (volatile int i = 0; i < 160; i++);   // crude ~10 µs delay
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

// Start measurement: reset capture flag and send trigger
void Start_HCSR04(void)
{
  is_first_capture = 0;
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // Enable input capture interrupt
  HCSR04_Trigger();
}

// Timer interrupt callback for measuring echo pulse width
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (is_first_capture == 0)  // Rising edge (echo start)
    {
      ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      is_first_capture = 1;
    }
    else if (is_first_capture == 1) // Falling edge (echo end)
    {
      ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);

      // Compute pulse duration
      if (ic_val2 > ic_val1)
        diff = ic_val2 - ic_val1;
      else
        diff = (0xFFFF - ic_val1 + ic_val2);

      // Convert duration to distance (cm)
      distance_cm = (float)diff * 0.0343 / 2.0;
    }
  }
}

// Write one 32-bit value to flash
void Flash_Write(uint32_t addr, uint32_t data)
{
    HAL_FLASH_Unlock();
    uint64_t data64 = data;  // must write doubleword (64-bit)
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data64);
    HAL_FLASH_Lock();
}

// Read 32-bit value from flash
uint32_t Flash_Read(uint32_t addr)
{
    return *(uint32_t*)addr;
}

// Erase one flash page
void Flash_Erase_Page(uint32_t pageAddr)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Page = (pageAddr - FLASH_BASE) / FLASH_PAGE_SIZE;
    eraseInit.NbPages = 1;
    eraseInit.Banks = FLASH_BANK_2;

    HAL_FLASHEx_Erase(&eraseInit, &pageError);
    HAL_FLASH_Lock();
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  char msg[64];
  uint32_t addr = FLASH_USER_START_ADDR;
  Flash_Erase_Page(FLASH_USER_START_ADDR); // erase before writing

  while (1)
  {
    Start_HCSR04();
    for (volatile int i = 0; i < 10000; i++);  // short delay for capture

    // Print measured distance over UART
    snprintf(msg, sizeof(msg), "Measured Distance: %lu cm\r\n", distance_cm);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Store distance to flash
    Flash_Write(addr, distance_cm);

    // Read back and print from flash
    uint32_t readData = Flash_Read(addr);
    snprintf(msg, sizeof(msg), "Sensor: %lu, Addr: 0x%08lX\r\n", readData, addr);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Move to next slot, wrap if needed
    addr += 8;
    if(addr >= FLASH_USER_START_ADDR + FLASH_PAGE_SIZE)
    {
        Flash_Erase_Page(FLASH_USER_START_ADDR);
        addr = FLASH_USER_START_ADDR;
    }

    HAL_Delay(500);  // wait between measurements
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;                   // no prescaler
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;             // max 32-bit counter
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_IC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  // Configure input capture on CH1 (PA0)
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
}

static void MX_USART2_UART_Init(void)
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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  // Onboard button
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  // Trigger pin (PA1) as output
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif



