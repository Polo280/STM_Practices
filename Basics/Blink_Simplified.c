#include "main.h"

int main(void){
	// Initialize SDK
	HAL_Init();
	
	// Initialize clock
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Configure GPIO pins
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Output push-pull mode
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Loop
	while(1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		HAL_Delay(1000);
	}
}