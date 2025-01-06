/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "BNO055.h"
#include "NMEA_Parser.h"
#include <stdio.h>
#include <string.h>

/*////////// MACROS //////////*/
// Time control
#define TIME_TO_BLINK 1000
#define TIME_TO_PRINT_UART 1000
#define TIME_TO_READ_IMU 500
#define TIME_TO_READ_TEMP 2000
#define TIME_TO_SAMPLE_VOLTAGE 1000
#define TIME_TO_SAMPLE_CURRENT 1000

// ADC Channels
#define ADC1_RANGE (float)(1 << 12)
#define CURRENT_SAMPLE_ADC_CHANNEL 1
#define VOLTAGE_SAMPLE_ADC_CHANNEL 2

// Vehicle properties
#define BATTERY_MAX_VOLTAGE_V 50
#define VOLTAGE_DIVIDER_FACTOR 0.058823

#define CURRENT_SENSE_GAIN 20.0
#define SHUNT_RESISTOR_VALUE 0.008

// Module Handlers
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
IWDG_HandleTypeDef hiwdg;

// Time auxiliaries
uint32_t uart_aux = 0;
uint32_t imu_aux = 0;
uint32_t blink_aux = 0;
uint32_t temp_aux = 0;
uint32_t curr_aux = 0;
uint32_t volt_aux = 0;

////////// Telemetry data //////////

// Max ADC reading obtained at max battery capacity with current voltage divider resistor values considering ADC resolution bit number
const float MAX_BATTERY_ADC_READING = BATTERY_MAX_VOLTAGE_V * VOLTAGE_DIVIDER_FACTOR * ADC1_RANGE / 3.3;
// Max amps read at voltage level 3.3V with current shunt value
const float CURRENT_SENSE_MAX_AMPS = 3.3 / (CURRENT_SENSE_GAIN * SHUNT_RESISTOR_VALUE);

// IMU
int8_t temperature = 0;
bno055_euler_t gyro_euler;
bno055_acc_t accel_data;

// ADC Measurements
float battery_voltage = 0.0;
float current_amps = 0.0;

// GPS
GPS_DATA gps_data;

///////////////////////////

// UART variables
uint8_t gps_buffer_index = 0;
uint8_t gps_received;
char rx_buff[100];
// Console print
char tx_buff[256];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);

static void telemetryInit(void);
static uint16_t pollFromChannelADC(uint8_t);
static void readADCValues(void);

char test_gps[256] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n";

int main(void)
{
  telemetryInit();

  // Start receiving from GPS with interrupt
  HAL_UART_Receive_IT(&huart1, &gps_received, 1);
  snprintf(tx_buff, sizeof(tx_buff), "Hi\n");

  while (1)
  {
	// Refresh the watchdog
	HAL_IWDG_Refresh(&hiwdg);

	// Transmit data through UART constantly
    if(HAL_GetTick() - uart_aux >= TIME_TO_PRINT_UART){
    	gps_data = parseGPSData(test_gps);
    	snprintf(tx_buff, sizeof(tx_buff), "Time fix: %ul\n", gps_data.GPGGA_data.fix_time);
    	HAL_UART_Transmit(&huart2, (uint8_t *)tx_buff, strlen(tx_buff), 100);
    	uart_aux = HAL_GetTick();
    }

    // Read current (amps) and battery voltage data
    if(HAL_GetTick() - curr_aux >= TIME_TO_SAMPLE_CURRENT){
    	readADCValues();
    	curr_aux = HAL_GetTick();
    }

    // Get IMU data
    if(HAL_GetTick() - imu_aux >= TIME_TO_READ_IMU){
    	// Read gyro euler angles
    	bno055_read_euler_hrp(&gyro_euler);
    	// Read accelerometer m/s²
    	bno055_read_acc_xyz(&accel_data);
    	// Update timestamp
        imu_aux = HAL_GetTick();
    }

    // Read temperature from BNO055 (max 1Hz)
    if(HAL_GetTick() - temp_aux >= TIME_TO_READ_TEMP){
		bno055_read_temperature(&temperature);
    	temp_aux = HAL_GetTick();
    }

    // Blink builtin LED
    if(HAL_GetTick() - blink_aux >= TIME_TO_BLINK){
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    	blink_aux = HAL_GetTick();
    }
  }
}

//////////////////////////////////////////////
////////////// CUSTOM FUNCTIONS //////////////
//////////////////////////////////////////////

static void telemetryInit(void){
	// System init
	HAL_Init();
	SystemClock_Config();

	// STM Peripheral Init
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();

	// BNO055 Init
	bno055_set_i2c_handler(&hi2c1);
	bno055_init(&default_bno055_config, &default_bno055_verification);

	// Watchdog Init
	MX_IWDG_Init();
}

static uint16_t pollFromChannelADC(uint8_t channel){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = channel;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);  // Apply the configuration for Channel 1
	// Start conversion for selected channel
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);  // Read value from Channel 1
	HAL_ADC_Stop(&hadc1);
	return adc_val;
}

static void readADCValues(void){
	uint16_t adc_val = 0;
	// Current in amps
	adc_val = pollFromChannelADC(CURRENT_SAMPLE_ADC_CHANNEL);
	current_amps = adc_val * CURRENT_SENSE_MAX_AMPS / ADC1_RANGE;

	// Voltage (should yield max approximate 3600 reading assuming 50v max)
	adc_val = pollFromChannelADC(VOLTAGE_SAMPLE_ADC_CHANNEL);
	battery_voltage = adc_val * BATTERY_MAX_VOLTAGE_V / MAX_BATTERY_ADC_READING;
	battery_voltage = (battery_voltage > BATTERY_MAX_VOLTAGE_V)? BATTERY_MAX_VOLTAGE_V : battery_voltage; // Limit to avoid errors in GUI visualization
}

// GPS data receive callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	snprintf(tx_buff, sizeof(tx_buff), "Int\n");
	if(huart->Instance == USART1){
		// If NMEA sentence is completed parse it and clear rx buffer
//		if(gps_received == '\n' || gps_received == '\0'){
//			gps_data = parseGPSData(rx_buff);
//			memset(rx_buff, 0, sizeof(rx_buff));
//			gps_buffer_index = 0;
//		}else{
//			rx_buff[gps_buffer_index] = (char)gps_received;
//			gps_buffer_index ++;
//		}
		HAL_UART_Receive_IT(&huart1, &gps_received, 1);
	}
}


//////////////////////////////////////////////
////////// PERIPHERAL CONFIGURATION //////////
//////////////////////////////////////////////

// Clock configuration
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

// ADC1 Config
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
}

// USART 1 Init (GPS)
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

// USART2 (console) Config
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
}

// I2C1 Config
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

// GPIO Config
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Independent Watchdog (IWDG Init)
static void MX_IWDG_Init(void)
{
  // HSI runs at 8Mhz, assuming a prescaler of 4 watchdog reload timeout is aprox 2s
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

///////////////////////////////////////////////

// Error handle
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
