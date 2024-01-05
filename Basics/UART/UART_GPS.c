#include "main.h"

// Global
GPIO_InitTypeDef gpio_struct;
UART_HandleTypeDef uart;
UART_HandleTypeDef uart_gps;

// Function declarations
void ConfigClock(void);
void UsartConfig(void);
void GpioConfig(void);


int main(void){
	// Initialize HAL
	HAL_Init();

	// Initialize clock and USART1 peripheral
	ConfigClock();
	UsartConfig();
	
	// RX Buffer
	uint8_t rx_buff[10];
	HAL_UART_Receive_IT(&uart_gps, rx_buff, sizeof(rx_buff));

	while(1){
		
	    HAL_Delay(1000);
	}
	return 0;
}

void ConfigClock(void){
	// Enable Port A GPIOs
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// Initialize UART interface peripheral clock (USART2 which is connected to the usb serial)
	RCC_PeriphCLKInitTypeDef periph_clock;
	periph_clock.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART1;
	periph_clock.Usart1ClockSelection = RCC_USART2CLKSOURCE_PCLK1; // Clock source to peripheral group clock

	HAL_RCCEx_PeriphCLKConfig(&periph_clock);

}

void UsartConfig(void){
	// CONFIGURATION OF UART CHANNEL 1
	uart.Instance = USART2;
	uart.Init.BaudRate = 115200;
	uart.Init.WordLength = UART_WORDLENGTH_8B;
	uart.Init.Parity = UART_PARITY_NONE;
	uart.Init.Mode = UART_MODE_TX_RX;
	uart.Init.StopBits = UART_STOPBITS_1;

	HAL_UART_Init(&uart);  // Hopefully doesnt trigger an error

	uart_gps.Instance = USART1;
	uart_gps.Init.BaudRate = 9600;
	uart_gps.Init.WordLength = UART_WORDLENGTH_8B;
	uart_gps.Init.Parity = UART_PARITY_NONE;
	uart_gps.Init.Mode = UART_MODE_TX_RX;
	uart_gps.Init.StopBits = UART_STOPBITS_1;
	uart_gps.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&uart_gps);  // Hopefully doesnt trigger an error
}

void GpioConfig(void){
	// Led pin
	gpio_struct.Pin = GPIO_PIN_5;
	gpio_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &gpio_struct);

	// UART GPS TX
	gpio_struct.Pin = GPIO_PIN_2;
	gpio_struct.Mode = GPIO_MODE_AF_PP;
	gpio_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio_struct);

	// UART GPS RX
	gpio_struct.Pin = GPIO_PIN_3;
	gpio_struct.Mode = GPIO_MODE_AF_PP;
	gpio_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio_struct);
}
