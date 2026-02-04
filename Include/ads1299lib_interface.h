/*
 * ads1299lib_interface.h
 *
 *  Created on: 3 feb 2026
 *      Author: Usuario
 */

#ifndef INCLUDE_ADS1299LIB_INTERFACE_H_
#define INCLUDE_ADS1299LIB_INTERFACE_H_
#include "ads1299lib.h"

ads_result_t 	ads_interface_init		(ads_t *self);
void 			ads_interface_hard_reset(ads_t *self);
void		 	ads_interface_spi_tx	(ads_t *self, uint8_t *buff, uint16_t len);
void		 	ads_interface_spi_rx	(ads_t *self, uint8_t *buff, uint16_t len);
void 			ads_interface_delay		(ads_t *self, uint16_t millis);
void 			ads_interface_stop		(ads_t *self);
void 			ads_interface_start		(ads_t *self);
#endif /* INCLUDE_ADS1299LIB_INTERFACE_H_ */
