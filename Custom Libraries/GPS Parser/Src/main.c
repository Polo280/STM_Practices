/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "NMEA_Parser.h"

/*////////// MACROS //////////*/
// Time control
#define TIME_TO_BLINK 1000
#define TIME_TO_PRINT_UART 1000
#define TIME_TO_READ_IMU 500
#define TIME_TO_READ_TEMP 2000
#define TIME_TO_SAMPLE_VOLTAGE 1000
#define TIME_TO_SAMPLE_CURRENT 1000

#define TIME_TO_PARSE_GPS 2000


/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;

// Time auxiliaries
uint32_t uart_aux = 0;
uint32_t imu_aux = 0;
uint32_t blink_aux = 0;
uint32_t temp_aux = 0;
uint32_t curr_aux = 0;
uint32_t volt_aux = 0;
uint32_t gps_aux = 0;

// UART variables
volatile uint32_t gps_buffer_index = 0;
char gps_received;
char gps_buffer[256];
// Console print
char tx_buff[256];

volatile uint16_t ov_errors = 0;

// GPS
GPS_DATA gps_data;

// Test variables
uint16_t counter = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  //MX_IWDG_Init();

  // USART GPS Interrupt configuration
  USART1->CR1 |= (1 << 2);  // Enable receiver mode
  USART1->CR1 |= (1 << 5);  // Enable RXNE interrupt

  while (1)
  {
	  // Refresh the watchdog
	  //HAL_IWDG_Refresh(&hiwdg);

	  // Check if data is available to read UART
	  if(USART1->ISR & (1 << 5)){
		  gps_received = USART1->RDR; // Read received byte
		  // Clear overrun error flag (halts reception if not cleared)
		  if (USART1->ISR & (1 << 3)) {
			  USART1->ICR |= (1 << 3);
		  }
		  // Check index for overflow
		  if(gps_buffer_index >= sizeof(gps_buffer) - 1){
			  gps_buffer[sizeof(gps_buffer)] = '\0';
			  strcpy(tx_buff, gps_buffer);
			  gps_buffer_index = 0;
		  }
		  *(gps_buffer + gps_buffer_index) = gps_received;
		  gps_buffer_index ++;
	  }

	  if(HAL_GetTick() - gps_aux >= TIME_TO_PARSE_GPS){
//		  if(gps_received == '\n' || gps_received == '\0'){
//			gps_data = parseGPSData(gps_buffer);
//			memset(gps_buffer, 0, sizeof(gps_buffer));
//			gps_buffer_index = 0;
//		  }else{
//			rx_buff[gps_buffer_index] = (char)gps_received;
//			gps_buffer_index ++;
//		  }
		  gps_aux = HAL_GetTick();
	  }

	  // Transmit data through UART constantly
	  if(HAL_GetTick() - uart_aux >= TIME_TO_PRINT_UART){
		//snprintf(tx_buff, sizeof(tx_buff), "Overrun errors: %u, index: %lu\n", ov_errors, gps_buffer_index);
		HAL_UART_Transmit(&huart2, (uint8_t *)tx_buff, strlen(tx_buff), 500);
		uart_aux = HAL_GetTick();
	  }

	  // Blink builtin LED
	  if(HAL_GetTick() - blink_aux >= TIME_TO_BLINK){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		blink_aux = HAL_GetTick();
	  }
  }
}



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
