#include "main.h"
#include <stdio.h>
#include <string.h>

#define TIME_TO_SAMPLE 3000
#define TIME_TO_BLINK 1000

UART_HandleTypeDef usb_uart;

// Variables to control
unsigned long time_aux = 0;
unsigned long time_aux2 = 0;

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
	source_osc.PLL.PLLMUL = RCC_PLL_MUL3;   // Multiply signal by 2

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

int main(void){
	// Initialization of HAL
	HAL_Init();
	// Clocks initialization
	ClockConfig();
	// Configure GPIOs
	GPIOConfigs();
	// Init UART
	UARTConfig();

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
			time_aux2 = HAL_GetTick();
		}

	}
}
