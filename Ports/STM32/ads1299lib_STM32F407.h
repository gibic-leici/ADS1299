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

#include "stm32f4xx.h"

#if SPI_DMA
// No specific include needed for DMA registers in CMSIS
#endif // SPI_DMA

#if DRDY_IT
// No specific include needed for EXTI registers in CMSIS
#endif // DRDY_IT

#if FREERTOS
#include "freertos.h"
#include "semphr.h"
#endif // FREERTOS

/**
 * @struct STM32_interface_handler_t
 * @brief Platform-specific configuration data for the STM32 ADS1299 port
 *
 * Must be populated by the application and passed as `interface_Handler`
 * inside the `ads_init_t` structure before calling `ads_init()`.
 */
typedef struct {
  // SPI and DMA
  SPI_TypeDef *spi; ///< SPI peripheral to use (e.g. SPI1, SPI2, SPI3)
#if SPI_DMA
  DMA_TypeDef *dma;           ///< DMA controller (DMA1 or DMA2)
  uint32_t spi_rx_dma_stream; ///< DMA stream index for SPI RX (0-7)
  uint32_t spi_tx_dma_stream; ///< DMA stream index for SPI TX (0-7)
#endif                        // SPI_DMA

  // GPIOs
  GPIO_TypeDef *cs_port;   ///< GPIO port for the chip-select (CS) pin
  uint32_t cs_pin;         ///< CS pin bitmask (e.g. 1 << 4)
  GPIO_TypeDef *prst_port; ///< GPIO port for the RESET/PWDN pin
  uint32_t prst_pin;       ///< RESET/PWDN pin bitmask
  GPIO_TypeDef *drdy_port; ///< GPIO port for the DRDY pin
  uint32_t drdy_pin;       ///< DRDY pin bitmask

  // DRDY IRQ
#if DRDY_IT
  IRQn_Type drdy_EXTI_IRQn; ///< NVIC IRQ number for the DRDY EXTI line
  uint32_t drdy_EXTI_line;  ///< EXTI line bitmask (e.g. EXTI_IMR_MR5)
#endif                      // DRDY_IT

  // FreeRTOS
#if FREERTOS
  SemaphoreHandle_t mutex; ///< Binary semaphore for SPI bus mutual exclusion
#endif                     // FREERTOS
                           //  Delay handler
  void (*delay)(
      uint32_t millis); ///< Platform delay callback (millisecond resolution)
} STM32_interface_handler_t;

#if SPI_DMA
#include "ads1299lib.h"
/**
 * @brief Enable DMA streams to start the SPI RX sample transfer
 *
 * Drives CS LOW and enables both the RX and TX DMA streams. Must be called
 * after ads_spi_rx_DMA_stop_and_prepare() or
 * ads_interface_spi_rx_sample_DMA_prepare_next() has configured the streams.
 *
 * @param self Pointer to ads_t structure
 */
void ads_interface_spi_rx_sample_DMA(ads_t *self);

/**
 * @brief Prepare DMA streams for the next full ADS1299 sample frame reception
 *
 * Reconfigures and re-arms the DMA streams for the next sample transfer.
 * Intended to be called from the DMA TC interrupt handler.
 *
 * @param self Pointer to ads_t structure
 * @param buff Pointer to the destination buffer for the next sample frame
 */
void ads_interface_spi_rx_sample_DMA_prepare_next(ads_t *self, uint8_t *buff);
#endif // SPI_DMA

#endif /* PORTS_STM32_ADS1299LIB_STM32_H_ */
