#include "main.h"
#include <stdio.h>
#include <string.h>

#define TIME_TO_SAMPLE 3000
#define TIME_TO_BLINK 1000

UART_HandleTypeDef usb_uart;

// Variables to control
unsigned long timer; 

void ClockConfigs(void){
}

void GPIOConfigs(void){
	// Clock for GPIO port A
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef builtin_LED;
	// LED config (bultin LED is GPIOA PIN 5)
	builtin_LED.Mode = GPIO_MODE_OUTPUT_PP;
	builtin_LED.Pin = GPIO_PIN_5;
	builtin_LED.Pull = GPIO_NOPULL;
	builtin_LED.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &builtin_LED);
	
	// ADC configuring (PA 0)
	GPIO_InitTypeDef analog0;
	analog0.Mode = GPIO_MODE_ANALOG;
	analog0.Speed = GPIO_SPEED_FREQ_MEDIUM;
	analog0.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &analog0);
}

void UARTConfig(void){
	// USART channel 2 is connected to the USB port
	usb_uart.Instance = USART2;
	usb_uart.Init.BaudRate = 115200;
	usb_uart.Init.Mode = UART_MODE_TX_RX;
	usb_uart.Init.WordLength = UART_WORDLENGTH_8B;
	HAL_UART_Init(&usb_uart);
}

int main(void){
	// Initialization of HAL
	HAL_Init();

	// Configure GPIOs
	GPIOConfigs();
	// Init UART
	UARTConfig();

	// Data to send testing
	char message[50];
	int counter = 0;

	// Testing for string management
	sprintf(message, "Tamaño de Hola -> %d", strlen("Hola"));
	HAL_UART_Transmit(&usb_uart, (uint8_t *)message, strlen(message), 50);


	while(1){
		sprintf(message, "Cuenta -> %d\n", counter);
		counter ++;

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_UART_Transmit(&usb_uart, (uint8_t *)message, strlen(message), 50);
		HAL_Delay(1000);

		// Is count is larger than 20 reset the stm
		if(counter >= 20){
			HAL_NVIC_SystemReset();
		}
	}
}
