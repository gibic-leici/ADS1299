/**
 * @file ads1299lib_interface.h
 * @brief Platform interface declarations used by the ADS1299 driver
 *
 * The user must provide implementations for these functions for their
 * specific platform (SPI transmit/receive, delay, reset, start/stop).
 *
 * @author Marcelo Haberman <marcelo.haberman@gmail.com>,
 *         marcelo.haberman@ing.unlp.edu.ar - GIBIC (gibic.ar)
 * @date 2026-02-05
 */

#ifndef INCLUDE_ADS1299LIB_INTERFACE_H_
#define INCLUDE_ADS1299LIB_INTERFACE_H_
#include "ads1299lib.h"

/**
 * @brief Initialize the platform-specific ADS1299 interface
 *
 * Sets up SPI, DMA, and any synchronization primitives required by the
 * platform-specific implementation.
 *
 * @param self Pointer to the `ads_t` structure
 * @return ads_result_t R_OK on success, R_FAIL on error
 */
ads_result_t 	ads_interface_init		(ads_t *self);

/**
 * @brief Perform a hardware reset using PRST/RESET pin
 *
 * Toggling the hardware reset line will perform a full chip reset.
 *
 * @param self Pointer to the `ads_t` structure
 */
void 			ads_interface_hard_reset(ads_t *self);

/**
 * @brief Transmit a buffer via SPI
 *
 * Platform-specific SPI transmit routine. Chip select should be handled by
 * the implementation and the function should be thread-safe if the platform
 * requires it.
 *
 * @param self Pointer to the `ads_t` structure
 * @param buff Data buffer to transmit
 * @param len Length in bytes
 */
void 		 	ads_interface_spi_tx	(ads_t *self, uint8_t *buff, uint16_t len);

/**
 * @brief Receive data via SPI
 *
 * Platform-specific SPI receive routine.
 *
 * @param self Pointer to the `ads_t` structure
 * @param buff Buffer to receive data into
 * @param len Number of bytes to receive
 */
void 		 	ads_interface_spi_rx	(ads_t *self, uint8_t *buff, uint16_t len);

/**
 * @brief Delay helper using platform timing
 *
 * Provides a millisecond delay using the platform-specific implementation.
 *
 * @param self Pointer to the `ads_t` structure
 * @param millis Delay in milliseconds
 */
void 			ads_interface_delay		(ads_t *self, uint16_t millis);

/**
 * @brief Stop data acquisition / disable data-ready interrupt
 *
 * Implementation should ensure data transfers are stopped and interrupts
 * disabled to allow safe register reads/writes.
 *
 * @param self Pointer to the `ads_t` structure
 */
void 			ads_interface_stop		(ads_t *self);

/**
 * @brief Start data acquisition / enable data-ready interrupt
 *
 * Implementation should enable data-ready signaling and any required
 * DMA or interrupt mechanisms.
 *
 * @param self Pointer to the `ads_t` structure
 */
void 			ads_interface_start		(ads_t *self);
#endif /* INCLUDE_ADS1299LIB_INTERFACE_H_ */
