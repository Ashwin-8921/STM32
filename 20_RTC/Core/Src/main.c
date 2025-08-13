/*
 * This program initializes RTC on STM32, sets time/date if not initialized,
 * and continuously prints current time and date via UART2.
 */

#include "main.h"
#include<stdio.h>

RTC_HandleTypeDef hrtc;   // RTC handle
UART_HandleTypeDef huart2; // UART2 handle


void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);


// Redirects printf output to UART2
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}





int main(void)
{
  RTC_TimeTypeDef sTime = {0}; // Structure for time
  RTC_DateTypeDef sDate = {0}; // Structure for date

  HAL_Init();                 // Initialize HAL library
  SystemClock_Config();       // Configure system clock
  MX_USART2_UART_Init();      // Initialize UART2
  MX_RTC_Init();              // Initialize RTC

  // If RTC not yet initialized, set default time/date
  if ((RTC->ISR & RTC_ISR_INITS) == 0)
  {

    // Set time
    sTime.Hours = 20;
    sTime.Minutes = 39;
    sTime.Seconds = 30;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    // Set date
    sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
    sDate.Month = RTC_MONTH_AUGUST;
    sDate.Date = 13;
    sDate.Year = 25;
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  }

  // Main loop: continuously read and print time/date
  while (1)
  {
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // Read time
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // Read date (must be after time read)

    // Print time and date in HH:MM:SS | DD-MM-YYYY format
    printf("Time: %02d:%02d:%02d | Date: %02d-%02d-20%02d\r\n",
           sTime.Hours, sTime.Minutes, sTime.Seconds,
           sDate.Date, sDate.Month, sDate.Year);

    HAL_Delay(1000); // Wait 1 second
  }
}

// Configures system clock using HSI and LSI oscillators
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSI and LSI, configure PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Configure CPU, AHB, and APB clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

// Initializes RTC with 24-hour format
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  // Default RTC time/date (will be overwritten if already set)
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
}

// Initializes UART2 at 115200 baud
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



// Handles errors by stopping execution
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
