#include "main.h"

// SPI handle declaration
SPI_HandleTypeDef hspi1;

// Transmit and receive buffers for SPI communication
uint8_t tx_buffer[10] = {10, 20, 40, 50, 60, 70, 90, 90, 100, 110};
uint8_t rx_buffer[10];

// Counter to track number of completed SPI transmissions
volatile int counter = 0;

// Function prototypes
void SystemClock_Config(void);
static void spi1_init(void);

// Callback function called when SPI transmit/receive completes (Interrupt Mode)
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) // Ensure it's SPI1 triggering this
    {
        counter++; // Increment counter each time a transmission finishes
    }
}

int main(void)
{
    HAL_Init();               // Initialize HAL library
    SystemClock_Config();     // Configure the system clock
    spi1_init();              // Initialize SPI1 peripheral

    // Start SPI transmit/receive in interrupt mode
    HAL_SPI_TransmitReceive_IT(&hspi1, tx_buffer, rx_buffer, sizeof(tx_buffer));

    while (1)
    {
        // Main loop — MCU can perform other tasks while SPI works in background
    }
}

// SPI1 Initialization
static void spi1_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA and SPI1 peripheral clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Configure SPI1 pins: SCK (PA5), MISO (PA6), MOSI (PA7)
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // No pull-up or pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // High speed
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;    // SPI1 alternate function
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure SPI1 parameters
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;                   // Master mode
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;         // Full duplex
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;              // 8-bit data
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;            // Clock idle low
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;                // First clock edge capture
    hspi1.Init.NSS = SPI_NSS_SOFT;                        // Software NSS management
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // SPI clock speed
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;               // MSB first
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;               // Disable TI mode
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; // No CRC
    hspi1.Init.CRCPolynomial = 7;                         // CRC polynomial

    // Initialize SPI1
    if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }

    // Enable SPI1 interrupt
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

// System Clock Configuration
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Set voltage scaling for optimal power consumption
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure HSI (High-Speed Internal oscillator) and PLL
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

  // Configure CPU, AHB, and APB clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

// Error handler — stops execution in case of error
void Error_Handler(void)
{
  __disable_irq(); // Disable all interrupts
  while (1) { }    // Infinite loop
}
