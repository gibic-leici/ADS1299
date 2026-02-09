/**
 * @file ads1299lib_STM32.c
 * @brief STM32-specific hardware interface for the ADS1299 library
 *
 * Implements SPI, DMA, GPIO and interrupt glue code used by the core
 * ADS1299 driver. Designed for STM32F4xx using the LL drivers and CMSIS-RTOS.
 *
 * @author Marcelo Haberman <marcelo.haberman@gmail.com>,
 *         marcelo.haberman@ing.unlp.edu.ar - GIBIC (gibic.ar)
 * @date 2026-02-05
 */

#include "ads1299lib.h"
#include "ads1299lib_interface.h"
#include "ads1299lib_STM32.h"

#include <assert.h>

/**
 * @brief Perform a delay using the hardware interface handler
 * 
 * @param self Pointer to ads_t structure
 * @param millis Delay duration in milliseconds
 */
void ads_interface_delay(ads_t *self, uint16_t millis) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t*) self->interface_Handler;
	assert(interface->delay != NULL);

	interface->delay(millis);
}

/**
 * @brief Perform hardware reset of ADS1299 via RESET/PWDN pin
 * 
 * Toggles the RESET pin low then high to perform a complete hardware reset.
 * This resets all internal registers to their default values and
 * re-initializes the serial interface state machine.
 * 
 * @param self Pointer to ads_t structure
 * 
 * @note Duration: ~1.1 seconds (100ms low + 1000ms high)
 */
void ads_interface_hard_reset(ads_t *self) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t*) self->interface_Handler;
	assert(interface->delay != NULL);

	LL_GPIO_ResetOutputPin(interface->prst_port, interface->prst_pin);
	interface->delay(100);
	LL_GPIO_SetOutputPin(interface->prst_port, interface->prst_pin);
	interface->delay(1000);
}

/**
 * @brief Initialize ADS1299 hardware interface
 * 
 * Sets up the SPI interface, DMA streams, and synchronization primitives
 * (mutex semaphore) for thread-safe SPI communication.
 * 
 * @param self Pointer to ads_t structure
 * @return ads_result_t R_OK if initialization successful
 */
ads_result_t ads_interface_init(ads_t *self) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t*) self->interface_Handler;
	assert(interface->delay != NULL);

	// Create binary semaphore for SPI access synchronization
#if FREERTOS
	if(interface->mutex == NULL)
		interface->mutex = xSemaphoreCreateBinary();

	// Release semaphore for initial access
	xSemaphoreGive( interface->mutex );
#endif //FREERTOS

	// Enable SPI peripheral
	LL_SPI_Enable(interface->spi);
#if SPI_DMA
	// Disable DMA streams initially
	LL_DMA_DisableStream(interface->dma, interface->spi_rx_dma_stream);
	LL_DMA_DisableStream(interface->dma, interface->spi_tx_dma_stream);
	
	// Configure TX DMA for single byte mode (no memory increment)
	LL_DMA_SetMemoryIncMode(interface->dma, interface->spi_tx_dma_stream, LL_DMA_MEMORY_NOINCREMENT);
#endif //SPI_DMA
	return R_OK;
}

/**
 * @brief Stop ADC data acquisition and disable interrupts
 * 
 * Disables the DRDY external interrupt, sends SDATAC and STOP commands
 * to the ADS1299, and updates the device status.
 * 
 * @param self Pointer to ads_t structure
 * 
 * @warning Blocks interrupts; ensure proper synchronization
 */
void ads_interface_stop(ads_t *self) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

#if DRDY_IT
	STM32_interface_handler_t *interface = (STM32_interface_handler_t*)self->interface_Handler;
	LL_EXTI_DisableIT_0_31(interface->drdy_EXTI_line);
	NVIC_DisableIRQ(interface->drdy_EXTI_IRQn);
#endif //DRDY_IT


}

/**
 * @brief Start ADC data acquisition and enable interrupts
 * 
 * Sends START and RDATAC commands to the ADS1299 and enables
 * the DRDY external interrupt to signal new data availability.
 * 
 * @param self Pointer to ads_t structure
 * 
 */
void ads_interface_start(ads_t *self) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);


#if DRDY_IT
	STM32_interface_handler_t *interface = (STM32_interface_handler_t*)self->interface_Handler;
	LL_EXTI_ClearFlag_0_31(interface->drdy_EXTI_line);
	LL_EXTI_EnableIT_0_31(interface->drdy_EXTI_line);
	NVIC_EnableIRQ(interface->drdy_EXTI_IRQn);
#endif //DRDY_IT
}

/**
 * @brief Wait for SPI bus to be ready for the next operation
 * 
 * Polls the SPI status flags to ensure the bus is not busy
 * and the transmit buffer is empty before proceeding.
 */
#define WAIT(spi) while(LL_SPI_IsActiveFlag_BSY(spi) || !LL_SPI_IsActiveFlag_TXE(spi)){;}

/**
 * @brief Transmit data via SPI with byte-level control
 * 
 * Thread-safe SPI transmission using binary semaphore for mutual exclusion.
 * Transmits data byte-by-byte with interleaved delays to meet
 * ADS1299 multi-byte command timing requirements (section 9.5.3.1 of datasheet).
 * 
 * @param self Pointer to ads_t structure
 * @param buff Pointer to transmit buffer
 * @param len Number of bytes to transmit
 * 
 * @note CS pin is controlled: LOW during transmission, HIGH after completion
 * @note Includes 1ms delay between bytes for multi-byte command timing
 */
void ads_interface_spi_tx(ads_t *self, uint8_t *buff, uint16_t len) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t*) self->interface_Handler;
	assert(interface->delay != NULL);
	assert(buff != NULL);
#if FREERTOS
	xSemaphoreTake( interface->mutex, portMAX_DELAY);
#endif //FREERTOS
	LL_GPIO_ResetOutputPin(interface->cs_port, interface->cs_pin);
	for (int j = 0; j < len; j++) {
		WAIT(interface->spi);
		LL_SPI_TransmitData8(interface->spi, *(buff + j));
		WAIT(interface->spi);

		LL_SPI_ReceiveData8(interface->spi);
		interface->delay(1);
		// Delay for multi-byte command timing per ADS1299 datasheet section 9.5.3.1
	}
	LL_GPIO_SetOutputPin(interface->cs_port, interface->cs_pin);
#if FREERTOS
	xSemaphoreGive( interface->mutex );
#endif //FREERTOS

}


inline void ads_interface_spi_rx_sample(ads_t *self, uint8_t *buff) {

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t*) self->interface_Handler;

	int bytes_to_read = self->num_channels*3 + 3;
#if FREERTOS
	xSemaphoreTake( interface->mutex, portMAX_DELAY);
#endif //FREERTOS
	interface->cs_port->BSRR = interface->cs_pin << 16;
	for (int j = 0; j < bytes_to_read; j++) {
		WAIT(interface->spi);
		LL_SPI_TransmitData8(interface->spi, 0x00);
		WAIT(interface->spi);

		*(buff + j) = LL_SPI_ReceiveData8(interface->spi);
	}
	interface->cs_port->BSRR = interface->cs_pin;
#if FREERTOS
	xSemaphoreGive( interface->mutex );
#endif //FREERTOS

}

/**
 * @brief Receive data via SPI with byte-level control
 * 
 * Thread-safe SPI reception using binary semaphore for mutual exclusion.
 * Receives data byte-by-byte by transmitting 0x00 dummy bytes
 * while clocking in the response.
 * 
 * @param self Pointer to ads_t structure
 * @param buff Pointer to receive buffer
 * @param len Number of bytes to receive
 * 
 * @note CS pin is controlled: LOW during reception, HIGH after completion
 * @note Includes ~7µs NOP delay before CS HIGH to meet tCS timing
 */
void ads_interface_spi_rx(ads_t *self, uint8_t *buff, uint16_t len) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t*) self->interface_Handler;
	assert(interface->delay != NULL);
	assert(buff != NULL);
#if FREERTOS
	xSemaphoreTake( interface->mutex, portMAX_DELAY);
#endif //FREERTOS
	LL_GPIO_ResetOutputPin(interface->cs_port, interface->cs_pin);
	for (int j = 0; j < len; j++) {
		WAIT(interface->spi);
		LL_SPI_TransmitData8(interface->spi, 0x00);
		WAIT(interface->spi);

		*(buff + j) = LL_SPI_ReceiveData8(interface->spi);
	}
	// Wait minimum 4 Tclk (approx 7µs) before raising CS per ADS1299 datasheet
	for (int k = 100; k > 0; k--)
		__NOP();
	LL_GPIO_SetOutputPin(interface->cs_port, interface->cs_pin);
#if FREERTOS
	xSemaphoreGive( interface->mutex );
#endif //FREERTOS

}

/**
 * @brief Transmit data via SPI with byte-level control
 *
 * Thread-safe SPI transmission using binary semaphore for mutual exclusion.
 * Transmits data byte-by-byte with interleaved delays to meet
 * ADS1299 multi-byte command timing requirements (section 9.5.3.1 of datasheet).
 *
 * @param self Pointer to ads_t structure
 * @param buff Pointer to transmit buffer
 * @param len Number of bytes to transmit
 *
 * @note CS pin is controlled: LOW during transmission, HIGH after completion
 * @note Includes 1ms delay between bytes for multi-byte command timing
 */
uint8_t ads_interface_read_reg(ads_t *self, ads_regs_enum_t reg) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t*) self->interface_Handler;
	assert(interface->delay != NULL);

#if FREERTOS
	xSemaphoreTake( interface->mutex, portMAX_DELAY);
#endif //FREERTOS

	uint8_t buff[3] = { 0 };
	buff[0] = ADS_CMD_RREG + (uint8_t) reg;

	volatile uint8_t ret = 0;

	LL_GPIO_ResetOutputPin(interface->cs_port, interface->cs_pin);
	for (int j = 0; j < 3; j++) {
		WAIT(interface->spi);
		LL_SPI_TransmitData8(interface->spi, *(buff + j));
		WAIT(interface->spi);

		ret = LL_SPI_ReceiveData8(interface->spi);
		interface->delay(1);
		// Delay for multi-byte command timing per ADS1299 datasheet section 9.5.3.1
	}
	LL_GPIO_SetOutputPin(interface->cs_port, interface->cs_pin);
#if FREERTOS
	xSemaphoreGive( interface->mutex );
#endif //FREERTOS

	return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////
#if SPI_DMA
uint32_t dummy =0;


void ads_spi_rx_DMA_stop_and_prepare(ads_t *self, uint8_t * buff, uint16_t len)
{
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface = (STM32_interface_handler_t*)self->interface_Handler;
	assert(buff != NULL);

	LL_GPIO_SetOutputPin(interface->cs_port,interface->cs_pin);

	LL_DMA_DisableStream(interface->dma, interface->spi_rx_dma_stream); // RX
	LL_DMA_DisableStream(interface->dma, interface->spi_tx_dma_stream); // TX


	LL_DMA_ConfigAddresses(interface->dma, interface->spi_rx_dma_stream, 	LL_SPI_DMA_GetRegAddr(interface->spi), 	(uint32_t)buff,							LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_ConfigAddresses(interface->dma, interface->spi_tx_dma_stream,  	(uint32_t)&dummy, 						LL_SPI_DMA_GetRegAddr(interface->spi),	LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	// Configure data length
	LL_DMA_SetDataLength(interface->dma, interface->spi_rx_dma_stream, len);
	LL_DMA_SetDataLength(interface->dma, interface->spi_tx_dma_stream, len);


	LL_SPI_EnableDMAReq_RX(interface->spi);
	LL_SPI_EnableDMAReq_TX(interface->spi);

	// Clear DMA transfer complete flags
	LL_DMA_ClearFlag_TC3(interface->dma); // Stream 3 - RX
	LL_DMA_ClearFlag_TC4(interface->dma); // Stream 4 - TX
	// Enable DMA Transfer complete interrupt
	LL_DMA_EnableIT_TC(interface->dma, interface->spi_rx_dma_stream);
}

void ads_spi_rx_DMA_start_fast(ads_t *self)
{
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface = (STM32_interface_handler_t*)self->interface_Handler;
	LL_GPIO_ResetOutputPin(interface->cs_port,interface->cs_pin);

	LL_DMA_EnableStream(interface->dma, interface->spi_rx_dma_stream); // RX
	LL_DMA_EnableStream(interface->dma, interface->spi_tx_dma_stream); // TX
}

#endif //SPI_DMA
