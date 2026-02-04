/*
 * ads1299lib_STM32.h
 *
 *  Created on: 3 feb 2026
 *      Author: Usuario
 */

#ifndef PORTS_STM32_ADS1299LIB_STM32_H_
#define PORTS_STM32_ADS1299LIB_STM32_H_

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "freertos.h"
#include "semphr.h"

typedef struct {
	//SPI y dma
	SPI_TypeDef *spi;
	DMA_TypeDef *dma;
	uint32_t spi_rx_dma_stream;
	uint32_t spi_tx_dma_stream;

	//gpios
	GPIO_TypeDef *cs_port;
	uint32_t cs_pin;
	GPIO_TypeDef *prst_port;
	uint32_t prst_pin;
	GPIO_TypeDef *drdy_port;
	uint32_t drdy_pin;

	// drdy irq
	IRQn_Type drdy_EXTI_IRQn;
	uint32_t drdy_EXTI_line;

	//freertos
	SemaphoreHandle_t mutex;

	//delay handler
	void (*delay)(uint32_t millis);
}	STM32_interface_handler_t;

void ads_spi_rx_DMA_stop_and_prepare(ads_t *self, uint8_t * buff, uint16_t len);
void ads_spi_rx_DMA_start_fast(ads_t *self);

#endif /* PORTS_STM32_ADS1299LIB_STM32_H_ */
