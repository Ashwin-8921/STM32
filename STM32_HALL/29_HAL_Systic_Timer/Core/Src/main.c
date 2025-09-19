#include "main.h"                // Include HAL library

void SystemClock_Config(void);   // Function to configure system clock
static void MX_GPIO_Init(void);  // Function to configure GPIO

int main(void)
{
  HAL_Init();                       // Init HAL and SysTick
  SystemClock_Config();             // Configure system clock
  MX_GPIO_Init();                   // Init LED GPIO

  uint32_t last = HAL_GetTick();    // Reference time

  while (1)
  {
    if ((HAL_GetTick() - last) >= 1000) // If 1000 ms passed
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
      last = HAL_GetTick();         // Update reference
    }
  }
}

// Configure system clock using HSI and PLL
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Oscillator config
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Clock config

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); // Set regulator scale

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Use HSI
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // Enable HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // Source = HSI
  RCC_OscInitStruct.PLL.PLLM = 1;                            // PLLM divider
  RCC_OscInitStruct.PLL.PLLN = 10;                           // PLLN multiplier
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // PLLP divider
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // PLLQ divider
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // PLLR divider
  HAL_RCC_OscConfig(&RCC_OscInitStruct);                     // Apply oscillator config

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;         // Select buses
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // SYSCLK = PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB divider
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;          // APB1 divider
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // APB2 divider
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);  // Apply clock config
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
