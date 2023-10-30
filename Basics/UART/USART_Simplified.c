#include "main.h"

// Global
UART_HandleTypeDef uart;

// Function declarations
void ConfigClock(void);
void UsartConfig(void);

int main(void){
	// Initialize HAL
	HAL_Init();

	// Initialize clock and USART1 peripheral
	ConfigClock();
	UsartConfig();

	// Data to be sent
	uint8_t message[] = "Hola xd\n";

	while(1){
		HAL_UART_Transmit(&uart, message, sizeof(message), 10);
		HAL_Delay(1000);
	}
	return 0;
}

void ConfigClock(void){
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Initialize UART interface peripheral clock
	RCC_PeriphCLKInitTypeDef periph_clock;
	periph_clock.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	periph_clock.Usart1ClockSelection = RCC_USART2CLKSOURCE_PCLK1; // Clock source to peripheral group clock

	HAL_RCCEx_PeriphCLKConfig(&periph_clock);
}

void UsartConfig(void){
	// CONFIGURATION OF UART CHANNEL 1
	uart.Instance = USART2;
	uart.Init.BaudRate = 115200;
	uart.Init.WordLength = UART_WORDLENGTH_8B;
	uart.Init.Parity = UART_PARITY_NONE;
	uart.Init.Mode = UART_MODE_TX;
	uart.Init.StopBits = UART_STOPBITS_1;

	HAL_UART_Init(&uart);  // Hopefully doesnt trigger an error
}
