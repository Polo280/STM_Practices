#include "main.h"
#include <stdio.h>

/////////// GLOBAL SCOPE ///////////
static void UART_Config(void);
static void ErrorHandler(void);
static void GPIO_Config(void);
static void MX_ADC1_Init(void);

UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;

unsigned long time_aux1 = 0, time_aux2 = 0, time_aux3 = 0;
uint8_t rx_buff[UARTRX_BUFF_SIZE];
uint8_t rx_byte, buffer_index;
uint8_t clear_buff = 0;

// Variables to store ADC values for Channel 1 and Channel 2
uint16_t adc_value_channel_1 = 0;
uint16_t adc_value_channel_2 = 0;
uint16_t adc_value_channel_3 = 0;

////////////// MAIN ///////////////
int main(void){
	// Configuration functions
	HAL_Init();

	UART_Config();
	GPIO_Config();
	MX_ADC1_Init();

	while(1){
		if(HAL_GetTick() - time_aux1 >= 200){
			ADC_ChannelConfTypeDef sConfig = {0};
			// Configure ADC for Channel 1 (PA_0, ADC1_IN1)
			sConfig.Channel = ADC_CHANNEL_1;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);  // Apply the configuration for Channel 1

			// Start conversion for Channel 1
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adc_value_channel_1 = HAL_ADC_GetValue(&hadc1);  // Read value from Channel 1
			HAL_ADC_Stop(&hadc1);
			/////////////////////////////////////////////////////
			// Configure ADC for Channel 2 (PA_1, ADC1_IN2)
			sConfig.Channel = ADC_CHANNEL_2;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);  // Apply the configuration for Channel 2

			// Start conversion for Channel 2
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adc_value_channel_2 = HAL_ADC_GetValue(&hadc1);  // Read value from Channel 2
			HAL_ADC_Stop(&hadc1);
			/////////////////////////////////////////////////////
			// Configure ADC for Channel 7 (PC_1, ADC1_IN7)
			sConfig.Channel = ADC_CHANNEL_7;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);  // Apply the configuration for Channel 2

			// Start conversion for Channel 7
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adc_value_channel_3 = HAL_ADC_GetValue(&hadc1);  // Read value from Channel 2
			HAL_ADC_Stop(&hadc1);

			time_aux1 = HAL_GetTick();
		}


		// Transmit the data in the buffer to the console (for debugging)
		if(HAL_GetTick() - time_aux2 >= 500){
			adc_value_channel_1 = (adc_value_channel_1 * 100) / 4095;
			adc_value_channel_2 = (adc_value_channel_2 * 100) / 4095;
			adc_value_channel_3 = (adc_value_channel_3 * 100) / 4095;
			sprintf((char *)rx_buff, "V%d", adc_value_channel_1);
			HAL_UART_Transmit(&huart2, rx_buff, UARTRX_BUFF_SIZE, 50);
			sprintf((char *)rx_buff, "G%d", adc_value_channel_2);
			HAL_UART_Transmit(&huart2, rx_buff, UARTRX_BUFF_SIZE, 50);
			sprintf((char *)rx_buff, "T%d", adc_value_channel_3);
			HAL_UART_Transmit(&huart2, rx_buff, UARTRX_BUFF_SIZE, 50);
			time_aux2 = HAL_GetTick();

			// Just blink an LED
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
	}
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;

  // Init the ADC
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    ErrorHandler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    ErrorHandler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    ErrorHandler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;  // Rank 2 for Channel 2
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      ErrorHandler();
  }

  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;  // Rank 3 for channel 7
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	ErrorHandler();
  }
}

//// UART CONFIGURATION
void UART_Config(void){
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

	if (HAL_UART_Init(&huart2) != HAL_OK){
		ErrorHandler();
	}
}

void GPIO_Config(void){
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	  /* Configure GPIO pin : PC13 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL; // Use GPIO_PULLUP if needed
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Error Handler
void ErrorHandler(void){
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	while(1){
	}
}
