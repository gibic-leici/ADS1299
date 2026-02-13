/**
 * @file ads1299lib_STM32.h
 * @brief STM32-specific interface handler definitions for the ADS1299 port
 *
 * Defines the `STM32_interface_handler_t` used by the STM32 port
 * implementation in `ads1299lib_STM32.c`.
 *
 * @author Marcelo Haberman <marcelo.haberman@gmail.com>,
 *         marcelo.haberman@ing.unlp.edu.ar - GIBIC (gibic.ar)
 * @date 2026-02-05
 */

#ifndef PORTS_STM32_ADS1299LIB_STM32_H_
#define PORTS_STM32_ADS1299LIB_STM32_H_

#define SPI_DMA 1
#define DRDY_IT 1
#define FREERTOS 0

#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"


#if SPI_DMA
#include "stm32f4xx_ll_dma.h"
#endif // SPI_DMA

#if DRDY_IT
#include "stm32f4xx_ll_exti.h"
#endif // DRDY_IT

#if FREERTOS
#include "freertos.h"
#include "semphr.h"
#endif // FREERTOS

typedef struct {
  // SPI and DMA
  SPI_TypeDef *spi;
#if SPI_DMA
  DMA_TypeDef *dma;
  uint32_t spi_rx_dma_stream;
  uint32_t spi_tx_dma_stream;
#endif // SPI_DMA

  // GPIOs
  GPIO_TypeDef *cs_port;
  uint32_t cs_pin;
  GPIO_TypeDef *prst_port;
  uint32_t prst_pin;
  GPIO_TypeDef *drdy_port;
  uint32_t drdy_pin;

  // DRDY IRQ
#if DRDY_IT
  IRQn_Type drdy_EXTI_IRQn;
  uint32_t drdy_EXTI_line;
#endif // DRDY_IT

  // FreeRTOS
#if FREERTOS
  SemaphoreHandle_t mutex;
#endif // FREERTOS
       //  Delay handler
  void (*delay)(uint32_t millis);
} STM32_interface_handler_t;

#if SPI_DMA
#include "ads1299lib.h"
void ads_interface_spi_rx_sample_DMA(ads_t *self);
void ads_interface_spi_rx_sample_DMA_prepare_next(ads_t *self, uint8_t *buff);
#endif // SPI_DMA

#endif /* PORTS_STM32_ADS1299LIB_STM32_H_ */
