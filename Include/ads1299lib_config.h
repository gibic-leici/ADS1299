/**
 * @file ads1299lib_config.h
 * @brief Hardware configuration and platform-specific settings
 *
 * Define platform-specific options such as number of channels, chip-select
 * pin, PRST pin, DRDY EXTI lines, and default SPI/DMA selections.
 *
 * @author Marcelo Haberman <marcelo.haberman@gmail.com>,
 *         marcelo.haberman@ing.unlp.edu.ar - GIBIC (gibic.ar)
 * @date 2026-02-05
 */

#ifndef INCLUDE_ADS1299LIB_CONFIG_H_
#define INCLUDE_ADS1299LIB_CONFIG_H_

/**
 * Number of active channels in ADS1299 (4, 6, or 8)
 * This must match your actual hardware configuration
 */
#define ADS_CONFIG_N_CH 4

/**
 * DATA READY INTERRUPT (DRDY)
 * 
 * The DRDY pin signals when new ADC data is available.
 * Configure the EXTI line and IRQ number here.
 * 
 * @note These must match your STM32CubeMX configuration
 */
#define DRDY_EXTI_IRQn EXTI4_IRQn      // External interrupt line selection
#define DRDY_EXTI_NUM LL_EXTI_LINE_4   // EXTI line number (0-15)
// Previous unused: LL_EXTI_LINE_15, EXTI15_10_IRQn

/**
 * CHIP SELECT (CS) Pin Configuration
 * 
 * Controls SPI slave selection for the ADS1299 device.
 * CS must be pulled LOW to select the device and HIGH when deselected.
 */
#define CS_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_5

/**
 * POWER DOWN/RESET (PRST) Pin Configuration
 * 
 * Controls hardware reset and power down of ADS1299.
 * Pull LOW for reset, hold HIGH for normal operation.
 */
#define PRST_GPIO_Port  GPIOB
#define PRST_Pin GPIO_PIN_0

/**
 * SPI CONFIGURATION FOR STM32F4xx
 * 
 * STM32-SPECIFIC MODULE FOR ADS1299 LIBRARY
 * 
 * DEPENDENCIES:
 *   - Low-Level (LL) libraries for SPI and GPIO control
 *   - CMSIS-RTOS API V2 (with FreeRTOS)
 *   - DMA for high-speed data transfers
 * 
 * DESCRIPTION:
 * This module provides STM32-specific implementations of the ADS1299
 * interface functions for SPI communication, GPIO control, and
 * interrupt handling using LL and CMSIS-RTOS APIs.
 * 
 * NOTE: SPI and DMA instances must be pre-configured using STM32CubeMX
 */

// SPI Interface Selection (SPI1, SPI2, SPI3, etc.)
// Must be configured in STM32CubeMX with DMA support
#define ADS_SPI SPI2

// DMA Interface Selection (DMA1 or DMA2)
#define ADS_DMA DMA1

// DMA Stream Selection
// STM32F4xx has 8 streams per DMA (0-7)
// Verify these match your CubeMX configuration
#define ADS_STREAM_RX LL_DMA_STREAM_3  // Receive stream
#define ADS_STREAM_TX LL_DMA_STREAM_4  // Transmit stream

#endif /* INCLUDE_ADS1299LIB_CONFIG_H_ */
