#ifndef _ADS1299LIB_SPI_H
#define _ADS1299LIB_SPI_H

/*** EL USUARIO DEBE DEFINIR LAS FUNCIONES ***/

#include <stdint.h>
#include "freertos.h"

void ADS_spi_rx(uint8_t * buf, int n);	/* Transferencia SPI **********/
void ADS_spi_tx(uint8_t * buf, int n);	/* Recepcion SPI **************/
void ADS_spi_rx_DMA_start(volatile uint8_t * tBuf, int n);
void ADS_spi_rx_DMA_start_FromISR(volatile uint8_t * tBuf, int n/*,BaseType_t *pxHigherPriorityTaskWoken*/);
void ADS_spi_rx_DMA_stop(void);
void ADS_spi_rx_DMA_stop_FromISR(/*BaseType_t *pxHigherPriorityTaskWoken*/);
void ADS_spi_rx_DMA_start_fast(void);
void ADS_spi_rx_DMA_stop_and_prepare(volatile uint8_t * tBuf, int n);

void ADS_Delay_ms(int t_ms);									/* Delay ms *******************/
void ADS_PWRRST_LOW(void);							/* Clear linea PWDN/RESET *****/
void ADS_PWRRST_HIGH(void);						/* Set linea PWDN/RESET *******/
void ADS_DRDY_IT_DISABLE(void);								/* Deshabilitar interp DRDY ***/
void ADS_DRDY_IT_ENABLE(void);								/* Habilitar interp DRDY ******/
void ADS_DMA_DISABLE(void);

#endif
