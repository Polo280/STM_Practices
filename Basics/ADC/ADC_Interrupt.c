#include "main.h"
#include <stdio.h>
#include <string.h>

#define TIME_TO_BLINK 1000
#define TIME_TO_PRINT_ADC 500
#define TIME_TO_IWGD_TIMEOUT 10000

// Handlers
UART_HandleTypeDef usb_uart;
ADC_HandleTypeDef adc1;

// Variables for interrupt handling
volatile uint8_t adc_conv_completed = 0;

// TIM_Base_InitTypeDef pwm_tim_config;   // Only for configuring purposes

// Variables to control
unsigned long time_aux = 0;
unsigned long time_aux2 = 0;

uint16_t adc_reading = 0;

void ErrorHandler(void);


void ClockConfig(void){
	// Configure RCC oscillators
	RCC_OscInitTypeDef source_osc = {0};
	source_osc.HSIState = RCC_HSI_ON;
	source_osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	source_osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	// PLL config
	source_osc.PLL.PLLState = RCC_PLL_ON;
	source_osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	source_osc.PLL.PLLMUL = RCC_PLL_MUL2;   // Multiply signal by 2

	// If something unexpected happens handle the error
	if(HAL_RCC_OscConfig(&source_osc) != HAL_OK){
		ErrorHandler();
	}

	// Configure clock tree
	RCC_ClkInitTypeDef clk_init_struct = {0};
	clk_init_struct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
	clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clk_init_struct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clk_init_struct.APB1CLKDivider = RCC_HCLK_DIV2;
	clk_init_struct.APB2CLKDivider = RCC_HCLK_DIV2;

	if(HAL_RCC_ClockConfig(&clk_init_struct, FLASH_LATENCY_1) != HAL_OK){
		ErrorHandler();
	}
}

void GPIOConfigs(void){
	// Clock for GPIO port A
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef builtin_LED;
	// LED config (bultin LED is GPIOA PIN 5)
	builtin_LED.Mode = GPIO_MODE_OUTPUT_PP;
	builtin_LED.Pin = GPIO_PIN_5;
	builtin_LED.Pull = GPIO_NOPULL;
	builtin_LED.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &builtin_LED);
}

void ADC_Config(void){
	__HAL_RCC_ADC1_CLK_ENABLE();  // Synchronous clock derived from main PLL

	adc1.Instance = ADC1;
	adc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1; // DO NOT FORGET PRESCALER
	adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;  // Received data is kept in the least significant bits of the buufer (uint 16 in this case)
	adc1.Init.Resolution = ADC_RESOLUTION10b;
	adc1.Init.ContinuousConvMode = DISABLE;      // Convert a channel continuously
	adc1.Init.DiscontinuousConvMode = DISABLE;
	adc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;  // Flag to know when conversion is done (EOC = End of conversion)
	adc1.Init.NbrOfConversion = 1;

	ADC_ChannelConfTypeDef channel_conf = {0};
	channel_conf.Channel = ADC_CHANNEL_1;
	channel_conf.Rank = ADC_REGULAR_RANK_1;
	//channel_conf.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	// Configure and calibrate for manufacturing imperfections
	HAL_ADC_Init(&adc1);

	// Init channel
	if(HAL_ADC_ConfigChannel(&adc1, &channel_conf) != HAL_OK){
		ErrorHandler();
	}

	if(HAL_ADCEx_Calibration_Start(&adc1, 1) != HAL_OK){
		ErrorHandler();
	}
}

void UARTConfig(void){
	// USART channel 2 is connected to the USB port
	usb_uart.Instance = USART2;
	usb_uart.Init.BaudRate = 115200;
	usb_uart.Init.Mode = UART_MODE_TX_RX;
	usb_uart.Init.WordLength = UART_WORDLENGTH_8B;
	HAL_UART_Init(&usb_uart);
}

void ErrorHandler(){
	// Turn on bultin LED, restart micro
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(TIME_TO_IWGD_TIMEOUT);
	HAL_NVIC_SystemReset();
}

// Called automatically when EOC flag is set
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	adc_reading = HAL_ADC_GetValue(hadc);
	adc_conv_completed = 1;

	/* ADC continuous conversion is disabled so interrupt has to be restarted each time
	 * a conversion occurs, its not recommended to start the interrupt inside its callback function
	 * because it will probably lead to errors regarding not waiting some "recovering" time after adc sampling
	 * or triggering another interrupt while already handling one, its better to update a specific flag */
}

// Interrupt handler
void ADC1_IRQHandler(void) {
	HAL_ADC_IRQHandler(&adc1);
}

int main(void){
	// Initialization of HAL
	HAL_Init();
	// Clocks initialization
	ClockConfig();
	// Configure GPIOs
	GPIOConfigs();
	// Init UART
	UARTConfig();
	//ADC STart
	ADC_Config();

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);  // All bits are used for preempt priority
	HAL_NVIC_SetPriority(ADC1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(ADC1_IRQn);

	HAL_ADC_Start_IT(&adc1);

	// Variables to manage data
	char data[30];

	while(1){

		// If an adc coversion is completed, reset eoc flag
		if(adc_conv_completed == 1){
			adc_conv_completed = 0;
			HAL_ADC_Start_IT(&adc1);
		}

		// LED TOGGLE
		if(HAL_GetTick() - time_aux >= TIME_TO_BLINK){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			time_aux = HAL_GetTick();
		}

		// Send ADC reading
		if(HAL_GetTick() - time_aux2 >= TIME_TO_PRINT_ADC){

			snprintf(data, sizeof(data), "ADC value -> %d\n", adc_reading);
			HAL_UART_Transmit(&usb_uart, (uint8_t *)data, strlen(data), 50);
			time_aux2 = HAL_GetTick();
		}
	}
}
