/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "Lora.h"
#include "BNO055.h"
#include "NMEA_Parser.h"

/*/////////////// MACROS /////////////////*/
//////// Vehicle properties ////////
#define BATTERY_MAX_VOLTAGE_V 50
#define VOLTAGE_DIVIDER_FACTOR 0.058823
#define CURRENT_SENSE_GAIN 20.0
#define SHUNT_RESISTOR_VALUE 0.008

////////// Time control //////////
#define TIME_TO_SAMPLE_CURRENT 200
#define TIME_TO_SAMPLE_VOLTAGE 200
#define TIME_TO_READ_IMU 	   200
#define TIME_TO_PARSE_GPS      1000
#define TIME_TO_SEND_LORA      500
// DEBUG
#define TIME_TO_PRINT_UART     500
#define TIME_TO_BLINK 		   1000

////////// CONTROL MACROS //////////
// If surpassed triggers system reset
#define MAX_INVALID_IMU_SAMPLES 5

////// SENTENCE SPLIT SETTINGS (GPS) //////
#define MAX_SENTENCES_SPLIT 4
#define MAX_SENTENCE_LENGTH 100

////////// ADC Channels //////////
#define ADC1_RANGE (float)(1 << 12)
#define CURRENT_SAMPLE_ADC_CHANNEL ADC_CHANNEL_12
#define VOLTAGE_SAMPLE_ADC_CHANNEL ADC_CHANNEL_5

///////////////////////////////////////////////

// Module Handlers
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
SPI_HandleTypeDef hspi3;
IWDG_HandleTypeDef hiwdg;

// Max ADC reading obtained at max battery capacity with current voltage divider resistor values considering ADC resolution bit number
const float MAX_BATTERY_ADC_READING = BATTERY_MAX_VOLTAGE_V * VOLTAGE_DIVIDER_FACTOR * ADC1_RANGE / 3.3;
// Max amps read at voltage level 3.3V with current shunt value
const float CURRENT_SENSE_MAX_AMPS = 3.3 / (CURRENT_SENSE_GAIN * SHUNT_RESISTOR_VALUE);

// Time auxiliaries
uint32_t uart_aux = 0;
uint32_t lora_aux = 0;
uint32_t imu_aux = 0;
uint32_t blink_aux = 0;
uint32_t temp_aux = 0;
uint32_t curr_aux = 0;
uint32_t volt_aux = 0;
uint32_t gps_aux = 0;

// ADC Measurements
float battery_voltage = 0.0;
float current_amps = 0.0;

// IMU
int8_t temperature = 0;
bno055_euler_t gyro_euler;
bno055_acc_t accel_data;

// GPS
GPS_DATA gps_data;     // Struct to store all NMEA containing information
char gps_received;     // New byte received
char gps_buffer[512];  // Received bytes buffer
volatile uint32_t gps_buffer_index = 0;  // Control variable to know where to store new data
char nmea_sentences [MAX_SENTENCES_SPLIT][MAX_SENTENCE_LENGTH]; // NMEA Sentence store

// Console print
char tx_buff[256];

// Control variables
uint8_t invalid_imu_samples = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_IWDG_Init(void);

static void telemetryInit(void);
static void GPS_interruptHandler(void);
static uint16_t pollFromChannelADC(uint32_t);
static void buildPacketRF(void);
static void readADCValues(void);
static void processGPS(void);

// Main program
int main(void)
{
  // Configure and start peripherals and sensors
  telemetryInit();

  while (1)
  {
	  // Reload watchdog
	  HAL_IWDG_Refresh(&hiwdg);
	  // Check for GPS new data interrupt
	  GPS_interruptHandler();
	  // Check if IMU started well, if not trigger system reset to re-establish connection
	  if(invalid_imu_samples >= MAX_INVALID_IMU_SAMPLES){
		  __disable_irq();
		  NVIC_SystemReset();
	  }

	  ////////////////// READ DATA //////////////////
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
		// Check data integrity
		invalid_imu_samples = (accel_data.z == 0)? invalid_imu_samples + 1 : 0;
		imu_aux = HAL_GetTick();
	  }
	  // Parse GPS data in buffer
	  if(HAL_GetTick() - gps_aux >= TIME_TO_PARSE_GPS){
		  processGPS();
		  gps_aux = HAL_GetTick();
	  }
	  ///////////////////////////////////////////////////

	  // Construct string and send data over RF
	  if(HAL_GetTick() - lora_aux >= TIME_TO_SEND_LORA){
		  buildPacketRF();
		  lora_aux = HAL_GetTick();
	  }

	  /////////////////// DEBUG TOOLS ///////////////////
	  // Transmit data through UART constantly
	  if(HAL_GetTick() - uart_aux >= TIME_TO_PRINT_UART){
//		strcpy(tx_buff, "lol\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)tx_buff, strlen(tx_buff), 200);
		uart_aux = HAL_GetTick();
	  }
	  // BLINK
	  if(HAL_GetTick() - blink_aux >= TIME_TO_BLINK){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		  blink_aux = HAL_GetTick();
	  }
	  ///////////////////////////////////////////////////
  }
}

//////////////////////////////////////////////
////////////// CUSTOM FUNCTIONS //////////////
//////////////////////////////////////////////

static void telemetryInit(void){
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  //// IMU Init ////
  bno055_set_i2c_handler(&hi2c1);
  bno055_init(&default_bno055_config, &default_bno055_verification);

  // Lora RFM95W Init
  LoraSetSPIHandler(&hspi3);
  if(LoraDefaultInit() != LORA_OK){
	//Error_Handler();
  }

  // USART GPS Interrupt configuration
  USART1->CR1 |= (1 << 2);  // Enable receiver mode
  USART1->CR1 |= (1 << 5);  // Enable RXNE interrupt

  // Init Watchdog
  MX_IWDG_Init();

  HAL_UART_Transmit(&huart2, "System Reset\n", strlen("System Reset\n"), 200);
}

static void GPS_interruptHandler(void){
  // Check if RX interrupt flag is set
  if(USART1->ISR & (1 << 5)){
	  // Read received byte
	  gps_received = USART1->RDR;
	  // Clear overrun error flag (halts reception if not cleared)
	  if (USART1->ISR & (1 << 3)) {
		  USART1->ICR |= (1 << 3);
	  }
	  // Check index for overflow
	  if(gps_buffer_index >= sizeof(gps_buffer) - 1){
		  gps_buffer[sizeof(gps_buffer)] = '\0';
		  gps_buffer_index = 0;
	  }
	  *(gps_buffer + gps_buffer_index) = gps_received;
	  gps_buffer_index ++;
  }
}

static void processGPS(void){
	splitNMEASentences(gps_buffer, nmea_sentences);
	// First sentence generally is trash
	for(uint8_t i=1; i < MAX_SENTENCES_SPLIT; i++){
	    parseGPSData(nmea_sentences[i], &gps_data);
	}
}

static uint16_t pollFromChannelADC(uint32_t channel){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
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

static void buildPacketRF(void){
  /*
   * CURRENT FORMAT (COMMA SEPARATED)
   * 1) S = Start character
   * 2) Current in amps (float)
   * 3) Battery voltage (float)
   * 4,5,6) Accelerometer readings (m/s²)(float)
   * 7,8,9) Gyro readings euler (°)(float)
   * 10,11) Latitude, longitude (raw output - NOT degrees)(float)
   * 12) GPS Status character (A | V)
   *
   * EXTRAS:
   * RPMS if they will be measured
   * Temperature is relevant?
  */
  snprintf(tx_buff, sizeof(tx_buff), "S,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%c\n",
  current_amps, battery_voltage, accel_data.x, accel_data.y, accel_data.z,
  gyro_euler.h, gyro_euler.r, gyro_euler.p, gps_data.GPRMC_data.latitude, gps_data.GPRMC_data.longitude,
  gps_data.GPRMC_data.status);
}

//////////////////////////////////////////////
////////// PERIPHERAL CONFIGURATION //////////
//////////////////////////////////////////////
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ADC_OVERSAMPLING_RATIO_16;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

//  sConfig.Channel = ADC_CHANNEL_5;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
}

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
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
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

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI3_Init(void)
{
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
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

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
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
