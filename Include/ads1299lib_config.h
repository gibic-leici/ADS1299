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

#endif /* INCLUDE_ADS1299LIB_CONFIG_H_ */
