#include "main.h"
#include <stdio.h>
#include <string.h>

#define TIME_TO_SAMPLE 10000
#define TIME_TO_BLINK 1000
#define TIME_TO_UPDATE_PWM 100
#define TIME_TO_PRINT_ADC 500

// Handlers
UART_HandleTypeDef usb_uart;
TIM_HandleTypeDef pwm_tim;
ADC_HandleTypeDef adc1;

// TIM_Base_InitTypeDef pwm_tim_config;   // Only for configuring purposes

// Variables to control
unsigned long time_aux = 0;
unsigned long time_aux2 = 0;
unsigned long time_aux3 = 0;
unsigned long time_aux4 = 0;

uint16_t pwm_pulse_value = 0;
uint16_t adc_reading = 0;

void ErrorHandler(void);

void PWM_Start(void){
	// Source clock is derived from PLL
	pwm_tim.Instance = TIM16;
	pwm_tim.Init.Period = 999;  // A period of 1000 (starts counting from 0)
	pwm_tim.Init.Prescaler = 1;
	pwm_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	pwm_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pwm_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&pwm_tim);

	// Configure PWM Channel (TIM output compare)
	TIM_OC_InitTypeDef outcomp_config = {0};
	outcomp_config.Pulse = 200;  // The period is 1000, so 500 generates a 50% duty cycle
	outcomp_config.OCMode = TIM_OCMODE_PWM1;
	HAL_TIM_PWM_ConfigChannel(&pwm_tim, &outcomp_config, TIM_CHANNEL_1);  // Configure specified channel

	// Start PWM
	HAL_TIM_PWM_Start(&pwm_tim, TIM_CHANNEL_1);

	/* Description for OC Modes
	TIM_OCMODE_TIMING	This mode is used for timing purposes only. No actual output is driven on the pin.
	TIM_OCMODE_ACTIVE	On a compare match, the output goes active (high), useful for generating a single pulse.
	TIM_OCMODE_INACTIVE	On a compare match, the output goes inactive (low).
	TIM_OCMODE_TOGGLE	The output toggles on each compare match. Useful for generating a simple square wave.
	TIM_OCMODE_PWM1	In PWM mode 1, the output is high as long as the counter value is less than the compare value (pulse).
	TIM_OCMODE_PWM2	In PWM mode 2, the output is low as long as the counter value is less than the compare value (pulse).
	TIM_OCMODE_FORCED_ACTIVE	The output is forced to the active level (high). Often used for initialization purposes.
	TIM_OCMODE_FORCED_INACTIVE	The output is forced to the inactive level (low). Often used for initialization purposes.
	TIM_OCMODE_RETRIGERRABLE_OPM1	Similar to PWM1 but in a one-pulse mode that stops counting when a rising edge is detected on the trigger.
	TIM_OCMODE_RETRIGERRABLE_OPM2	Similar to PWM2 but in a one-pulse mode that stops counting when a rising edge is detected on the trigger.
	*/
}

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
	__HAL_RCC_TIM16_CLK_ENABLE();  // DONT FORGET TO ENABLE SYSCLOCK AS TIM SOURCE

	GPIO_InitTypeDef builtin_LED;
	// LED config (bultin LED is GPIOA PIN 5)
	builtin_LED.Mode = GPIO_MODE_OUTPUT_PP;
	builtin_LED.Pin = GPIO_PIN_5;
	builtin_LED.Pull = GPIO_NOPULL;
	builtin_LED.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &builtin_LED);

	// PWM GPIO
	GPIO_InitTypeDef pwm_gpio;
	pwm_gpio.Mode = GPIO_MODE_AF_PP;
	pwm_gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
	pwm_gpio.Pin = GPIO_PIN_6;
	pwm_gpio.Pull = GPIO_NOPULL;
	pwm_gpio.Alternate = GPIO_AF1_TIM16;
	HAL_GPIO_Init(GPIOA, &pwm_gpio);

	// ADC GPIO
	GPIO_InitTypeDef adc_gpio;
	adc_gpio.Mode = GPIO_MODE_ANALOG;
	adc_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	adc_gpio.Pin = GPIO_PIN_7;
	adc_gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &adc_gpio);
}

void ADC_Config(void){
	__HAL_RCC_ADC1_CLK_ENABLE();  // Synchronous clock derived from main PLL

	adc1.Instance = ADC1;
	adc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1; // DO NOT FORGET PRESCALER
	adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;  // Received data is kept in the least significant bits of the buufer (uint 16 in this case)
	adc1.Init.Resolution = ADC_RESOLUTION10b;
	adc1.Init.ContinuousConvMode = ENABLE;      // Convert a channel continuously
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
	// Turn on bultin LED and enter an infinite loop
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	while(1){

	}
}

void NVIC_Config(void){
	HAL_NVIC_SetPriority(SysTick_IRQn, 12, 10);
}

int main(void){
	// Initialization of HAL
	HAL_Init();
	//IRQ config
	//NVIC_Config();
	// Clocks initialization
	ClockConfig();
	// Configure GPIOs
	GPIOConfigs();
	// Init UART
	UARTConfig();
	// PWM Start
	PWM_Start();
	//ADC STart
	ADC_Config();

	// Variables to manage data
	char data[30];

	while(1){
		// LED TOGGLE
		if(HAL_GetTick() - time_aux >= TIME_TO_BLINK){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			time_aux = HAL_GetTick();
		}

		// Sending data
		if(HAL_GetTick() - time_aux2 >= TIME_TO_SAMPLE){
			snprintf(data, sizeof(data), "SysClock Freq: %lu\n", HAL_RCC_GetSysClockFreq());
			HAL_UART_Transmit(&usb_uart, (uint8_t *)data, strlen(data), 50);
			snprintf(data, sizeof(data), "Tick Freq: %d\n", HAL_GetTickFreq());
			HAL_UART_Transmit(&usb_uart, (uint8_t *)data, strlen(data), 50);
			snprintf(data, sizeof(data), "Tick Priority: %lu\n", HAL_GetTickPrio());
			HAL_UART_Transmit(&usb_uart, (uint8_t *)data, strlen(data), 50);
			time_aux2 = HAL_GetTick();
		}

		// Send ADC reading
		if(HAL_GetTick() - time_aux4 >= TIME_TO_PRINT_ADC){
			// Read ADC1
			HAL_ADC_Start(&adc1);

			// Start conversion and wait until its done
			if(HAL_ADC_PollForConversion(&adc1, 50) != HAL_OK){
				ErrorHandler();
			}

			// Get converted value & stop
			adc_reading = HAL_ADC_GetValue(&adc1);
			HAL_ADC_Stop(&adc1);

			snprintf(data, sizeof(data), "ADC value -> %d\n", adc_reading);
			HAL_UART_Transmit(&usb_uart, (uint8_t *)data, strlen(data), 50);
			time_aux4 = HAL_GetTick();
		}

		// PWM on GPIO handling
		if(HAL_GetTick() - time_aux3 >= TIME_TO_UPDATE_PWM){
			// Change PWM duty cycle
			__HAL_TIM_SET_COMPARE(&pwm_tim, TIM_CHANNEL_1, pwm_pulse_value);
			pwm_pulse_value += 20;

			if(pwm_pulse_value > 999){
				pwm_pulse_value = 0;
			}

			time_aux3 = HAL_GetTick();
		}

	}
}
