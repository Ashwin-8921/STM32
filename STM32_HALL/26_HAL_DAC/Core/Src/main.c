#include "main.h"
#include <stdio.h>

/* Global handles for peripherals */
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_ADC1_Init(void);
void MX_DAC1_Init(void);
void MX_USART2_UART_Init(void);
void MX_GPIO_Init(void);

/* Redirect printf to UART2 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int main(void)
{
    HAL_Init();                  // Initialize HAL library
    SystemClock_Config();        // Configure system clock
    MX_GPIO_Init();              // Initialize GPIO
    MX_USART2_UART_Init();       // Initialize UART2
    MX_DAC1_Init();              // Initialize DAC1
    MX_ADC1_Init();              // Initialize ADC1

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // Start DAC channel 1

    while (1)
    {
        // Sweep DAC from 0 to 4095 in steps of 512
        for (uint32_t val = 0; val <= 4095; val += 512)
        {
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val); // Set DAC output
            HAL_Delay(10); // Wait for voltage to settle

            HAL_ADC_Start(&hadc1);                                // Start ADC
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);     // Wait for conversion
            uint32_t adcVal = HAL_ADC_GetValue(&hadc1);           // Read ADC result

            float voltage = (3.3f * adcVal) / 4095.0f;            // Convert ADC value to voltage

            printf("DAC: %lu -> ADC: %lu -> Voltage: %.2f V\r\n", // @suppress("Float formatting support")
                   val, adcVal, voltage); // Print result over UART

            HAL_Delay(500); // Delay between steps
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); // Set voltage regulator

    // Enable HSI oscillator and configure PLL
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

    // Set CPU, AHB, and APB clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;                               // Use ADC1
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;    // No prescaler
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;          // 12-bit resolution
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;          // Right alignment
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;          // Single channel
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;       // End of conversion flag
    hadc1.Init.ContinuousConvMode = DISABLE;             // One conversion at a time
    hadc1.Init.NbrOfConversion = 1;                      // Only 1 channel
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;    // Software trigger
    HAL_ADC_Init(&hadc1);

    multimode.Mode = ADC_MODE_INDEPENDENT;               // Independent mode
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

    sConfig.Channel = ADC_CHANNEL_5;                     // Use channel 5 (PA0)
    sConfig.Rank = ADC_REGULAR_RANK_1;                   // Rank 1
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;     // Sampling time
    sConfig.SingleDiff = ADC_SINGLE_ENDED;               // Single-ended input
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void MX_DAC1_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac1.Instance = DAC1;                  // Use DAC1
    HAL_DAC_Init(&hdac1);

    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE; // No trigger, direct output
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE; // Buffer enabled
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE; // Internal connection
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1);
}

void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;               // Use USART2
    huart2.Init.BaudRate = 115200;          // Baud rate
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;     // Enable TX and RX
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();   // Enable GPIOC clock
    __HAL_RCC_GPIOH_CLK_ENABLE();   // Enable GPIOH clock
    __HAL_RCC_GPIOA_CLK_ENABLE();   // Enable GPIOA clock
    __HAL_RCC_GPIOB_CLK_ENABLE();   // Enable GPIOB clock

    // Configure LED pin
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    // Configure button pin
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq(); // Disable interrupts
    while (1) {}     // Stay here on error
}
