#include "MFRC522.h"
#include "stm32f4xx_hal_gpio.h"
/**
 * HAL_SPI_MspInit() - Override of SPI init function to configure SPI handle
 * and SPI low level resources
 *
 * Configures SPI follow guidline from STM32 user manual for STM32F4 HAL and
 * low-layer drivers.
 * SPI handle is declared in `MFRC522_t`. Configures SPI GPIO clock, pins are
 * set to alternate function push-pull. Interrupt and DMA are not implemented
 * yet, thus not configured within the SPI init.
 *
 *
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//SPI Clock 
	if(hspi->Instance == SPI1){
		__HAL_RCC_GPIOB_CLK_ENABLE();	
		__HAL_RCC_SPI1_CLK_ENABLE();	

		//SPI GPIO pin
		//SCK
		GPIO_InitStruct.Pin = SCK_PIN; 
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		//MISO
		GPIO_InitStruct.Pin = MISO_PIN; 
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		//MOSI
		GPIO_InitStruct.Pin = MOSI_PIN; 
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		//CSS
		GPIO_InitStruct.Pin = CSS_PIN;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}		
}
