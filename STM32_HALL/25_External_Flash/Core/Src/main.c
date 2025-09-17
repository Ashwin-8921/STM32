/*
This code logs distance data from an HC-SR04 ultrasonic sensor into a W25Q32 flash chip using SPI, 
while providing debug output via UART on an STM32 microcontroller.*/


#include "main.h"
#include <string.h>
#include <stdio.h>

// ===== Flash Chip Control (W25Q32) =====
#define FLASH_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)  // CS = 0 (select chip)
#define FLASH_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)    // CS = 1 (release chip)

// Flash command set
#define W25Q_WRITE_ENABLE    0x06
#define W25Q_PAGE_PROGRAM    0x02
#define W25Q_READ_DATA       0x03
#define W25Q_READ_STATUS     0x05
#define W25Q_JEDEC_ID        0x9F
#define W25Q_SECTOR_ERASE    0x20

// ===== Ultrasonic Sensor (HC-SR04) Pins =====
#define TRIG_PIN    GPIO_PIN_0
#define TRIG_PORT   GPIOA
#define ECHO_PIN    GPIO_PIN_1
#define ECHO_PORT   GPIOA

// ===== Flash memory layout =====
#define FLASH_SIZE_BYTES   (4UL * 1024UL * 1024UL)   // 4 MB total
#define SECTOR_SIZE_BYTES  (4UL * 1024UL)            // 4 KB per sector (smallest erasable block)

SPI_HandleTypeDef hspi2;   // SPI for flash memory
UART_HandleTypeDef huart2; // UART for serial messages
TIM_HandleTypeDef htim2;   // Timer for microsecond delays

uint32_t flash_addr = 0;   // Current write pointer in flash
char uart_buf[128];        // Buffer for UART messages


// ===== Function Prototypes =====
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);

void Delay_us(uint16_t us);   // Delay function (microseconds)

// Flash memory functions
void W25Q_WriteEnable(void);
uint8_t W25Q_ReadStatus(void);
void W25Q_WaitBusy(void);
void W25Q_PageProgram(uint32_t addr, uint8_t* data, uint16_t len);
void W25Q_ReadData(uint32_t addr, uint8_t* buf, uint16_t len);
void W25Q_SectorErase(uint32_t addr);
uint32_t W25Q32_ReadID(void);

// Ultrasonic sensor function
uint32_t HCSR04_Read_cm(void);


// ===== Flash Functions =====

// Erase a 4KB flash sector
void W25Q_SectorErase(uint32_t addr)
{
  uint8_t cmd[4];
  W25Q_WriteEnable();  // must enable write before erase
  cmd[0] = W25Q_SECTOR_ERASE;
  cmd[1] = (addr >> 16) & 0xFF;
  cmd[2] = (addr >> 8) & 0xFF;
  cmd[3] = addr & 0xFF;
  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
  W25Q_WaitBusy();     // wait until erase completes
}

// Allow writes to flash (mandatory before programming or erase)
void W25Q_WriteEnable(void)
{
  uint8_t cmd = W25Q_WRITE_ENABLE;
  FLASH_CS_LOW();
  HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
  FLASH_CS_HIGH();
}

// Read flash status register (bit0 = BUSY)
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

// Wait until flash chip finishes write/erase operation
void W25Q_WaitBusy(void)
{
  while (W25Q_ReadStatus() & 0x01) { }
}

// Program (write) up to 256 bytes to flash page
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

// Read bytes from flash memory
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

// Read JEDEC ID of flash chip (manufacturer + device ID)
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


// ===== Ultrasonic Sensor Functions =====

// Blocking microsecond delay using TIM2
void Delay_us(uint16_t us)
{
  uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
  while (((__HAL_TIM_GET_COUNTER(&htim2) - start) & 0xFFFF) < us) { }
}

// Trigger HC-SR04 and measure echo pulse → distance in cm
uint32_t HCSR04_Read_cm(void)
{
    uint32_t t1, t2;

    // Send 10 µs trigger pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    Delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // Wait until ECHO goes high
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET);
    t1 = __HAL_TIM_GET_COUNTER(&htim2);

    // Wait until ECHO goes low
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET);
    t2 = __HAL_TIM_GET_COUNTER(&htim2);

    // Calculate pulse width
    uint32_t pulse = (t2 >= t1) ? (t2 - t1) : (0x10000 - t1 + t2);

    // Convert microseconds → centimeters (speed of sound)
    return (pulse / 58U);
}


// ====== MAIN PROGRAM ======
int main(void)
{
  HAL_Init();               // Init HAL library
  SystemClock_Config();     // Setup system clock
  MX_GPIO_Init();           // Init GPIOs
  MX_USART2_UART_Init();    // Init UART for debug
  MX_SPI2_Init();           // Init SPI for flash
  MX_TIM2_Init();           // Init timer for delays
  HAL_TIM_Base_Start(&htim2);

  // Print startup message
  sprintf(uart_buf, "W25Q32 + HC-SR04 Logger Start\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

  // Print flash chip ID
  uint32_t id = W25Q32_ReadID();
  sprintf(uart_buf, "JEDEC ID: 0x%06lX\r\n", id);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

  // Erase sector 0 before logging
  W25Q_SectorErase(flash_addr);

  while (1)
  {
    // 1) Measure distance from HC-SR04
    uint32_t dist_cm = HCSR04_Read_cm();
    sprintf(uart_buf, "Distance: %lu cm\r\n", dist_cm);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    // 2) Store distance as 4 bytes (little-endian)
    uint8_t four[4];
    four[0] = (uint8_t)(dist_cm & 0xFF);
    four[1] = (uint8_t)((dist_cm >> 8) & 0xFF);
    four[2] = (uint8_t)((dist_cm >> 16) & 0xFF);
    four[3] = (uint8_t)((dist_cm >> 24) & 0xFF);

    // If we cross a sector boundary → erase new sector
    if ((flash_addr % SECTOR_SIZE_BYTES) == 0)
    {
      W25Q_SectorErase(flash_addr);
    }

    // Write measurement into flash
    W25Q_PageProgram(flash_addr, four, 4);

    // 3) Read back and verify
    uint8_t back[4];
    W25Q_ReadData(flash_addr, back, 4);
    uint32_t verify = (uint32_t)back[0] | ((uint32_t)back[1] << 8) | ((uint32_t)back[2] << 16) | ((uint32_t)back[3] << 24);
    sprintf(uart_buf, "Logged: %lu cm @ 0x%06lX\r\n", verify, flash_addr);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    // Advance flash pointer
    flash_addr += 4;
    if (flash_addr >= FLASH_SIZE_BYTES) flash_addr = 0;  // wrap if full

    HAL_Delay(1000);  // wait 1 second before next reading
  }
}


// ================= System Clock Configuration =================
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Set regulator voltage scaling (power setting)
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure HSI oscillator and PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                     // Turn on HSI (internal clock)
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                 // Enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;         // Use HSI as PLL source
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Select PLL as system clock source and configure bus clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;    // SYSCLK = PLL output
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           // AHB = SYSCLK
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;            // APB1 = HCLK
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;            // APB2 = HCLK

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}


// ================= SPI2 Initialization =================
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;                              // Use SPI2 peripheral
  hspi2.Init.Mode = SPI_MODE_MASTER;                  // MCU is master
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;        // Full duplex
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;            // 8-bit data
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;          // Clock idle low
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;              // Data valid on 1st edge
  hspi2.Init.NSS = SPI_NSS_SOFT;                      // Software control CS
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // SPI clock = SYSCLK/8
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;             // Send MSB first
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;             // Disable TI mode
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; // No CRC
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}


// ================= UART2 Initialization =================
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;                           // Use UART2 peripheral
  huart2.Init.BaudRate = 115200;                      // Baud rate = 115200
  huart2.Init.WordLength = UART_WORDLENGTH_8B;        // 8-bit data
  huart2.Init.StopBits = UART_STOPBITS_1;             // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;              // No parity
  huart2.Init.Mode = UART_MODE_TX_RX;                 // Enable TX and RX
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;        // No hardware flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;    // Oversampling by 16
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}


// ================= TIM2 Initialization =================
static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();                        // Enable TIM2 clock
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80 - 1;                      // Timer tick = 1 MHz (1 µs)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;        // Count up
  htim2.Init.Period = 0xFFFF;                         // Max count = 65535
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // No division
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}


// ================= GPIO Initialization =================
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO clocks
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure Flash CS pin (PB12) as output
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High speed
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure Trigger pin (ultrasonic) as output
  GPIO_InitStruct.Pin = TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

  // Configure Echo pin (ultrasonic) as input
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);
}


// ================= Error Handler =================
void Error_Handler(void)
{
  __disable_irq();        // Disable all interrupts
  while (1) { }           // Stay in infinite loop (for debugging)
}
