// A code for controlling an LED PWM with adc interrupt readings of a potenciometer
#include "main.h"
#include <stdio.h>
#include <string.h>

/*
 * SCHEMATIC DESCRIPTION
 * 1) Connect Potenciometer Mid pin to A0 in stm
 * 2) Connect LED to GPIO PA_6 (D12)
 * */

#define TIME_TO_BLINK 1000
#define TIME_TO_PRINT_ADC 500
#define TIME_TO_UPDATE_PWM 1
#define TIME_TO_IWGD_TIMEOUT 10000

// Handlers
UART_HandleTypeDef usb_uart;
ADC_HandleTypeDef adc1;
TIM_HandleTypeDef pwm_tim;

// Variables for interrupt handling
volatile uint8_t adc_conv_completed = 0;

// TIM_Base_InitTypeDef pwm_tim_config;   // Only for configuring purposes

// Variables to control
unsigned long time_aux = 0;
unsigned long time_aux2 = 0;
unsigned long time_aux3 = 0;

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
	__HAL_RCC_TIM16_CLK_ENABLE();

	GPIO_InitTypeDef builtin_LED;
	// LED config (bultin LED is GPIOA PIN 5)
	builtin_LED.Mode = GPIO_MODE_OUTPUT_PP;
	builtin_LED.Pin = GPIO_PIN_5;
	builtin_LED.Pull = GPIO_NOPULL;
	builtin_LED.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &builtin_LED);

	// PWM PIN
	GPIO_InitTypeDef pwm_pin;
	pwm_pin.Mode = GPIO_MODE_AF_PP;
	pwm_pin.Speed = GPIO_SPEED_FREQ_HIGH;
	pwm_pin.Alternate = GPIO_AF1_TIM16;  // TIM 16 Channel 1 (hardware mapped)
	pwm_pin.Pin = GPIO_PIN_6;
	pwm_pin.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &pwm_pin);
}

void PWM_Config(void){
	// CONFIGURE TIMER
	pwm_tim.Instance = TIM16;
	pwm_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	pwm_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pwm_tim.Init.Prescaler = 1;
	pwm_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
	pwm_tim.Init.Period = 1022;   // Determines resolution
	HAL_TIM_PWM_Init(&pwm_tim);

	// CONFIGURE TIM OUTPUT COMPARE
	TIM_OC_InitTypeDef pwm_tim_oc = {0};
	//pwm_tim_oc.OCFastMode = TIM_OCFAST_ENABLE;
	pwm_tim_oc.Pulse = 0;
	pwm_tim_oc.OCMode = TIM_OCMODE_PWM1;

	// CONFIGURE TIM CHANNEL MAPPED IN THE PIN
	HAL_TIM_PWM_ConfigChannel(&pwm_tim, &pwm_tim_oc, TIM_CHANNEL_1);

	// START PWM
	HAL_TIM_PWM_Start(&pwm_tim, TIM_CHANNEL_1);
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
	//ADC Start
	ADC_Config();
	// PWM Config
	PWM_Config();

	// CONFIGURE NVIC
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);  // All bits are used for preempt priority
	HAL_NVIC_SetPriority(ADC1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(ADC1_IRQn);

	// Initial ADC interrupt
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

		// UPDATE PWM
		if(HAL_GetTick() - time_aux3 >= TIME_TO_UPDATE_PWM){
			__HAL_TIM_SET_COMPARE(&pwm_tim, TIM_CHANNEL_1, adc_reading);
			time_aux3 = HAL_GetTick();
		}
	}
}
