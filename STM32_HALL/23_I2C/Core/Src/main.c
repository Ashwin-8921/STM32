/*
 * This program communicates with an ADXL345 accelerometer via I2C on STM32.
 * It can send data, initialize the sensor, and read acceleration values.
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* ADXL345 I2C address */
#define ADXL345_ADDR (0x68 << 1) // Shifted for R/W bit

uint8_t accel_data[6];  // Buffer for X, Y, Z raw data
int16_t x, y, z;        // Acceleration values
char msg[64];           // UART message buffer
uint8_t txData[] = "Hello ADXL345!"; // Test transmit data

I2C_HandleTypeDef hi2c1;    // I2C1 handle
UART_HandleTypeDef huart2;  // UART2 handle

void SystemClock_Config(void);   // Configures system clock
static void MX_GPIO_Init(void);  // Initializes GPIO pins
static void MX_USART2_UART_Init(void); // Initializes UART2
static void MX_I2C1_Init(void);  // Initializes I2C1

/* Initializes ADXL345 for measurement mode */
void ADXL345_Init(void)
{
    uint8_t data;

    data = 0x08; // Measurement mode
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D, 1, &data, 1, HAL_MAX_DELAY);

    data = 0x08; // Full resolution, ±2g range
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x31, 1, &data, 1, HAL_MAX_DELAY);
}

/* Reads acceleration data from ADXL345 */
void ADXL345_Read(void)
{
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32, 1, accel_data, 6, HAL_MAX_DELAY);

    x = (int16_t)(accel_data[1] << 8 | accel_data[0]); // Combine X high/low
    y = (int16_t)(accel_data[3] << 8 | accel_data[2]); // Combine Y high/low
    z = (int16_t)(accel_data[5] << 8 | accel_data[4]); // Combine Z high/low
}

int main(void)
{
  HAL_Init();               // Initialize HAL library
  SystemClock_Config();     // Configure system clock
  MX_GPIO_Init();           // Initialize GPIO
  MX_USART2_UART_Init();    // Initialize UART2
  MX_I2C1_Init();           // Initialize I2C1

  // Send test message to ADXL345
  if (HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDR, txData, sizeof(txData)-1, HAL_MAX_DELAY) == HAL_OK)
  {
      HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET); // Turn LED ON if success
  }
  else
  {
      HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET); // Turn LED OFF if fail
  }
   ADXL345_Init(); // Configure ADXL345 sensor for measurement mode

  while (1) // Infinite loop to continuously read and send data
  {
      ADXL345_Read(); // Read X, Y, Z acceleration values from ADXL345

      // Format acceleration data into a text string for UART transmission
      snprintf(msg, sizeof(msg), "X=%d, Y=%d, Z=%d\r\n", x, y, z);

      // Send the formatted acceleration data over UART
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

      HAL_Delay(500); // Wait for 500 ms before next reading
  }

}

/* Configures HSI oscillator, PLL, and clock dividers */
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/* Initializes I2C1 for 7-bit addressing */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

/* Initializes UART2 for 115200 baud communication */
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

/* Initializes GPIO for LED and button */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* Handles errors by stopping execution */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
