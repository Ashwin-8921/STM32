#include "main.h"

// Timer 16 handler
TIM_HandleTypeDef htim16;
// UART handler
UART_HandleTypeDef huart2;

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);

int main(void)
{
    uint16_t timer_val; // Variable to store timer count value

    HAL_Init();                 // Initialize HAL library
    SystemClock_Config();       // Configure system clock
    MX_GPIO_Init();             // Initialize GPIO
    MX_USART2_UART_Init();      // Initialize USART2
    MX_TIM16_Init();            // Initialize TIM16

    HAL_TIM_Base_Start(&htim16); // Start TIM16 base timer

    timer_val = __HAL_TIM_GET_COUNTER(&htim16); // Get initial timer value

    while (1)
    {
        // Check if 10,000 timer ticks have passed
        if (__HAL_TIM_GET_COUNTER(&htim16) - timer_val >= 10000)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Toggle LED (LD2 on PA5)
            timer_val = __HAL_TIM_GET_COUNTER(&htim16); // Update last timer value
        }
    }
}

// System clock configuration
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure main internal regulator output voltage
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    // Initializes HSI + PLL for system clock
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

    // Initializes CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

// TIM16 initialization
static void MX_TIM16_Init(void)
{
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 8000 - 1;          // Timer clock = 80 MHz / 8000 = 10 kHz
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 65535;                // Max counter value
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
    {
        Error_Handler();
    }
}

// USART2 initialization
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;                 // Baud rate
    huart2.Init.WordLength = UART_WORDLENGTH_8B;   // 8-bit data
    huart2.Init.StopBits = UART_STOPBITS_1;        // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;         // No parity
    huart2.Init.Mode = UART_MODE_TX_RX;            // Enable TX and RX
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;   // No hardware flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

// GPIO initialization
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO clocks
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Set LD2 (on PA5) initially LOW
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    // Configure B1 button pin
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    // Configure LD2 LED pin
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

// Error handler
void Error_Handler(void)
{
    __disable_irq(); // Disable interrupts
    while (1)
    {
        // Stay here if error occurs
    }
}

#ifdef USE_FULL_ASSERT
// Report assert errors
void assert_failed(uint8_t *file, uint32_t line)
{
    // Can add printf here for debugging
}
#endif
