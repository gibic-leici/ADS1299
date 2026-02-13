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

#include "ads1299lib_STM32F407.h"
#include "ads1299lib.h"
#include "ads1299lib_interface.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
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
			(STM32_interface_handler_t *)self->interface_Handler;
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
			(STM32_interface_handler_t *)self->interface_Handler;
	assert(interface->delay != NULL);

	LL_GPIO_ResetOutputPin(interface->prst_port, interface->prst_pin);
	interface->delay(100);
	LL_GPIO_SetOutputPin(interface->prst_port, interface->prst_pin);
	interface->delay(1000);
}

/**
 * @brief Initialize GPIO pins for ADS1299 communication
 *
 * Configures the CS (Chip Select), RESET/PWDN, and DRDY pins as required
 * for the ADS1299 interface. CS and RESET pins are set as push-pull outputs,
 * and DRDY pin is configured based on interrupt mode.
 *
 * @param interface Pointer to STM32_interface_handler_t structure
 */
static void init_gpios(STM32_interface_handler_t *interface) {
	assert(interface != NULL);
	assert(interface->cs_port != NULL);
	assert(interface->prst_port != NULL);
	assert(interface->drdy_port != NULL);

	// Enable GPIO port clocks
	// Enable only the GPIO port clocks that are actually used
	if (interface->cs_port != NULL) {
		//GPIOA:0, GPIOB:1, GPIOC:2...
		const uint16_t GPIO_SIZE = 0x400;
		uint8_t port_index = ((uint32_t)interface->cs_port - GPIOA_BASE) / GPIO_SIZE;
		if(port_index <= 8) LL_AHB1_GRP1_EnableClock(1<<port_index);
	}
	if (interface->drdy_port != NULL) {
		//GPIOA:0, GPIOB:1, GPIOC:2...
		const uint16_t GPIO_SIZE = 0x400;
		uint8_t port_index = ((uint32_t)interface->drdy_port - GPIOA_BASE) / GPIO_SIZE;
		if(port_index <= 8) LL_AHB1_GRP1_EnableClock(1<<port_index);
	}
	if (interface->prst_port != NULL) {
		//GPIOA:0, GPIOB:1, GPIOC:2...
		const uint16_t GPIO_SIZE = 0x400;
		uint8_t port_index = ((uint32_t)interface->prst_port - GPIOA_BASE) / GPIO_SIZE;
		if(port_index <= 8) LL_AHB1_GRP1_EnableClock(1<<port_index);
	}

	//LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE4);

	// Configure CS pin as output (push-pull, no pull)
	LL_GPIO_SetOutputPin(interface->cs_port,
			interface->cs_pin); // CS inactive HIGH
	LL_GPIO_SetPinMode(interface->cs_port, interface->cs_pin,
			LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(interface->cs_port, interface->cs_pin,
			LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(interface->cs_port, interface->cs_pin,
			LL_GPIO_SPEED_FREQ_HIGH);

	// Configure RESET/PWDN pin as output (push-pull, no pull)
	LL_GPIO_SetOutputPin(interface->prst_port,
			interface->prst_pin); // RESET inactive HIGH
	LL_GPIO_SetPinMode(interface->prst_port, interface->prst_pin,
			LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(interface->prst_port, interface->prst_pin,
			LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(interface->prst_port, interface->prst_pin,
			LL_GPIO_SPEED_FREQ_HIGH);

	// Configure DRDY pin as input for data ready interrupt/polling
	LL_GPIO_SetPinMode(interface->drdy_port, interface->drdy_pin,
			LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(interface->drdy_port, interface->drdy_pin,
			LL_GPIO_PULL_UP); // Pull-up for DRDY line
}

/**
 * @brief Configure SPI peripheral clock and GPIO pins based on SPI instance
 *
 * Automatically detects which SPI peripheral is configured and enables:
 * - Appropriate clock (APB1 for SPI2/SPI3, APB2 for SPI1)
 * - Required GPIO ports and pins (MISO, MOSI, SCK)
 *
 * Supported configurations for STM32F407:
 * - SPI1: PA5(SCK), PA6(MISO), PA7(MOSI), AF5
 * - SPI2: PB10(SCK), PC2(MISO), PC3(MOSI), AF5
 * - SPI3: PC10(SCK), PC11(MISO), PC12(MOSI), AF6
 *
 * @param spi Pointer to SPI peripheral (SPI1, SPI2, or SPI3)
 */
static void init_spi_gpio_and_clock(SPI_TypeDef *spi) {
	assert(spi != NULL);

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

#if defined(SPI1)
	if (spi == SPI1) {
		// Enable SPI1 peripheral clock (APB2)
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

		// Enable GPIOA clock
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

		// Configure PA5(SCK), PA6(MISO), PA7(MOSI) as alternate function AF5
		GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
		LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		return;
	}
#endif

#if defined(SPI2)
	if (spi == SPI2) {
		// Enable SPI2 peripheral clock (APB1)
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

		// Enable GPIOB and GPIOC clocks
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

		// Configure PB10(SCK) as alternate function AF5
		GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		// Configure PC2(MISO), PC3(MOSI) as alternate function AF5
		GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
		LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		return;
	}
#endif

#if defined(SPI3)
	if (spi == SPI3) {
		// Enable SPI3 peripheral clock (APB1)
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

		// Enable GPIOC clock
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

		// Configure PC10(SCK), PC11(MISO), PC12(MOSI) as alternate function AF6
		GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
		LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		return;
	}
#endif

	// If we reach here, unsupported SPI peripheral
	assert(0);
}

/**
 * @brief Initialize SPI peripheral and conditionally DMA for ADS1299
 * communication
 *
 * Configures SPI Mode 1 (CPOL=0, CPHA=1), 8-bit data width, MSB first,
 * with software NSS management. Automatically enables SPI peripheral clock
 * and configures GPIO pins based on the SPI instance. Clock speed is configured
 * to meet ADS1299's maximum 10 MHz requirement. If SPI_DMA is enabled,
 * initializes DMA streams for RX and TX transfers with proper channel and
 * memory increment settings.
 *
 * @param self Pointer to ads_t structure containing interface handler
 *
 * @note Uses software-managed Chip Select (CS) via GPIO
 * @note DMA channels must be configured in STM32_interface_handler_t
 */
static void init_spi(ads_t *self) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t *)self->interface_Handler;
	SPI_TypeDef *spi = interface->spi;
	assert(spi != NULL);

	// Configure SPI peripheral clock and GPIO pins
	init_spi_gpio_and_clock(spi);

	// Configure SPI using LL_SPI_InitTypeDef structure
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2; // < 18MHz
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(spi, &SPI_InitStruct);
	LL_SPI_SetStandard(spi, LL_SPI_PROTOCOL_MOTOROLA);

#if SPI_DMA
	// Initialize DMA streams for SPI transfers
	assert(interface->dma != NULL);

	/* DMA controller clock enable */

	/* Enable the appropriate DMA controller clock depending on the
	 * configured DMA instance (DMA1 or DMA2). Also enable the NVIC IRQ
	 * for the RX stream transfer-complete only. This makes the code
	 * follow the selected configuration in `interface` rather than
	 * hard-coding DMA1_Stream3. */
	if (interface->dma == DMA1) {
		__HAL_RCC_DMA1_CLK_ENABLE();
		/* Select IRQ based on configured RX stream */
		IRQn_Type rx_irq = 0;
#if defined(LL_DMA_STREAM_0)
		if (interface->spi_rx_dma_stream == LL_DMA_STREAM_0)
			rx_irq = DMA1_Stream0_IRQn;
		else
#endif
#if defined(LL_DMA_STREAM_1)
			if (interface->spi_rx_dma_stream == LL_DMA_STREAM_1)
				rx_irq = DMA1_Stream1_IRQn;
			else
#endif
#if defined(LL_DMA_STREAM_2)
				if (interface->spi_rx_dma_stream == LL_DMA_STREAM_2)
					rx_irq = DMA1_Stream2_IRQn;
				else
#endif
#if defined(LL_DMA_STREAM_3)
					if (interface->spi_rx_dma_stream == LL_DMA_STREAM_3)
						rx_irq = DMA1_Stream3_IRQn;
					else
#endif
#if defined(LL_DMA_STREAM_4)
						if (interface->spi_rx_dma_stream == LL_DMA_STREAM_4)
							rx_irq = DMA1_Stream4_IRQn;
						else
#endif
#if defined(LL_DMA_STREAM_5)
							if (interface->spi_rx_dma_stream == LL_DMA_STREAM_5)
								rx_irq = DMA1_Stream5_IRQn;
							else
#endif
#if defined(LL_DMA_STREAM_6)
								if (interface->spi_rx_dma_stream == LL_DMA_STREAM_6)
									rx_irq = DMA1_Stream6_IRQn;
								else
#endif
#if defined(LL_DMA_STREAM_7)
									if (interface->spi_rx_dma_stream == LL_DMA_STREAM_7)
										rx_irq = DMA1_Stream7_IRQn;
#endif
		if (rx_irq) {
			NVIC_SetPriority(rx_irq,
					NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
			NVIC_EnableIRQ(rx_irq);
		}
	} else if (interface->dma == DMA2) {
		__HAL_RCC_DMA2_CLK_ENABLE();
		IRQn_Type rx_irq = 0;
#if defined(LL_DMA_STREAM_0)
		if (interface->spi_rx_dma_stream == LL_DMA_STREAM_0)
			rx_irq = DMA2_Stream0_IRQn;
		else
#endif
#if defined(LL_DMA_STREAM_1)
			if (interface->spi_rx_dma_stream == LL_DMA_STREAM_1)
				rx_irq = DMA2_Stream1_IRQn;
			else
#endif
#if defined(LL_DMA_STREAM_2)
				if (interface->spi_rx_dma_stream == LL_DMA_STREAM_2)
					rx_irq = DMA2_Stream2_IRQn;
				else
#endif
#if defined(LL_DMA_STREAM_3)
					if (interface->spi_rx_dma_stream == LL_DMA_STREAM_3)
						rx_irq = DMA2_Stream3_IRQn;
					else
#endif
#if defined(LL_DMA_STREAM_4)
						if (interface->spi_rx_dma_stream == LL_DMA_STREAM_4)
							rx_irq = DMA2_Stream4_IRQn;
						else
#endif
#if defined(LL_DMA_STREAM_5)
							if (interface->spi_rx_dma_stream == LL_DMA_STREAM_5)
								rx_irq = DMA2_Stream5_IRQn;
							else
#endif
#if defined(LL_DMA_STREAM_6)
								if (interface->spi_rx_dma_stream == LL_DMA_STREAM_6)
									rx_irq = DMA2_Stream6_IRQn;
								else
#endif
#if defined(LL_DMA_STREAM_7)
									if (interface->spi_rx_dma_stream == LL_DMA_STREAM_7)
										rx_irq = DMA2_Stream7_IRQn;
#endif
		if (rx_irq) {
			NVIC_SetPriority(rx_irq,
					NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
			NVIC_EnableIRQ(rx_irq);
		}
	}

	// Disable DMA streams initially
	LL_DMA_DisableStream(interface->dma, interface->spi_rx_dma_stream);
	LL_DMA_DisableStream(interface->dma, interface->spi_tx_dma_stream);

	// Configure RX DMA stream (Peripheral to Memory)
	LL_DMA_SetDataTransferDirection(interface->dma, interface->spi_rx_dma_stream,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(interface->dma, interface->spi_rx_dma_stream,
			LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(interface->dma, interface->spi_rx_dma_stream,
			LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(interface->dma, interface->spi_rx_dma_stream,
			LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(interface->dma, interface->spi_rx_dma_stream,
			LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(interface->dma, interface->spi_rx_dma_stream,
			LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(interface->dma, interface->spi_rx_dma_stream,
			LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_DisableFifoMode(interface->dma, interface->spi_rx_dma_stream);

	// Configure TX DMA stream (Memory to Peripheral)
	LL_DMA_SetDataTransferDirection(interface->dma, interface->spi_tx_dma_stream,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(interface->dma, interface->spi_tx_dma_stream,
			LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(interface->dma, interface->spi_tx_dma_stream,
			LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(interface->dma, interface->spi_tx_dma_stream,
			LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(interface->dma, interface->spi_tx_dma_stream,
			LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetPeriphSize(interface->dma, interface->spi_tx_dma_stream,
			LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(interface->dma, interface->spi_tx_dma_stream,
			LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_DisableFifoMode(interface->dma, interface->spi_tx_dma_stream);
#endif // SPI_DMA
}

void init_drdy_interrupt(STM32_interface_handler_t *interface) {
	assert(interface != NULL);

	// Configure EXTI line for DRDY pin (falling edge detection)
	// Map GPIO port/pin to SYSCFG EXTICR so EXTI line is routed correctly.
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	{
		uint32_t port_index = ((uint32_t)interface->drdy_port - GPIOA_BASE) / 0x400;
		uint32_t pin_mask = interface->drdy_pin;
		uint32_t pin_index = 0;
		while ((pin_mask > 1) && ((pin_mask & 0x1) == 0)) {
			pin_mask >>= 1;
			pin_index++;
		}
		uint32_t exticr_idx = pin_index >> 2;
		uint32_t exticr_pos = (pin_index & 0x3) * 4;
		uint32_t tmp = SYSCFG->EXTICR[exticr_idx];
		tmp &= ~(0xF << exticr_pos);
		tmp |= ((port_index & 0xF) << exticr_pos);
		SYSCFG->EXTICR[exticr_idx] = tmp;
	}
	LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	EXTI_InitStruct.Line_0_31 = interface->drdy_EXTI_line;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	// Configure NVIC for DRDY EXTI interrupt
	NVIC_SetPriority(interface->drdy_EXTI_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(interface->drdy_EXTI_IRQn);
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
			(STM32_interface_handler_t *)self->interface_Handler;
	assert(interface->delay != NULL);

	// Create binary semaphore for SPI access synchronization
#if FREERTOS
	if (interface->mutex == NULL)
		interface->mutex = xSemaphoreCreateBinary();

	// Release semaphore for initial access
	xSemaphoreGive(interface->mutex);
#endif // FREERTOS

	// init GPIOS
	init_gpios(interface);

	// init SPI
	init_spi(self);

#if DRDY_IT
	// Init EXTI IRQ for DRDY pin
	init_drdy_interrupt(interface);
#endif // DRDY_IT

	// Enable SPI peripheral
	LL_SPI_Enable(interface->spi);
	return ADS_R_OK;
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
	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t *)self->interface_Handler;
	LL_EXTI_DisableIT_0_31(interface->drdy_EXTI_line);
	NVIC_DisableIRQ(interface->drdy_EXTI_IRQn);
#endif // DRDY_IT
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
	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t *)self->interface_Handler;
	LL_EXTI_ClearFlag_0_31(interface->drdy_EXTI_line);
	LL_EXTI_EnableIT_0_31(interface->drdy_EXTI_line);
	NVIC_EnableIRQ(interface->drdy_EXTI_IRQn);
#endif // DRDY_IT
}

/**
 * @brief Wait for SPI bus to be ready for the next operation
 *
 * Polls the SPI status flags to ensure the bus is not busy
 * and the transmit buffer is empty before proceeding.
 */
#define WAIT(spi)                                                              \
		while (LL_SPI_IsActiveFlag_BSY(spi) || !LL_SPI_IsActiveFlag_TXE(spi)) {      \
			;                                                                          \
		}

/**
 * @brief Transmit data via SPI with byte-level control
 *
 * Thread-safe SPI transmission using binary semaphore for mutual exclusion.
 * Transmits data byte-by-byte with interleaved delays to meet
 * ADS1299 multi-byte command timing requirements (section 9.5.3.1 of
 * datasheet).
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
			(STM32_interface_handler_t *)self->interface_Handler;
	assert(interface->delay != NULL);
	assert(buff != NULL);
#if FREERTOS
	xSemaphoreTake(interface->mutex, portMAX_DELAY);
#endif // FREERTOS
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
	xSemaphoreGive(interface->mutex);
#endif // FREERTOS
}

inline void ads_interface_spi_rx_sample(ads_t *self, uint8_t *buff) {

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t *)self->interface_Handler;

	int bytes_to_read = self->num_channels * 3 + 3;
#if FREERTOS
	xSemaphoreTake(interface->mutex, portMAX_DELAY);
#endif // FREERTOS
	interface->cs_port->BSRR = interface->cs_pin << 16;
	for (int j = 0; j < bytes_to_read; j++) {
		WAIT(interface->spi);
		LL_SPI_TransmitData8(interface->spi, 0x00);
		WAIT(interface->spi);

		*(buff + j) = LL_SPI_ReceiveData8(interface->spi);
	}
	interface->cs_port->BSRR = interface->cs_pin;
#if FREERTOS
	xSemaphoreGive(interface->mutex);
#endif // FREERTOS
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
			(STM32_interface_handler_t *)self->interface_Handler;
	assert(interface->delay != NULL);
	assert(buff != NULL);
#if FREERTOS
	xSemaphoreTake(interface->mutex, portMAX_DELAY);
#endif // FREERTOS
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
	xSemaphoreGive(interface->mutex);
#endif // FREERTOS
}

/**
 * @brief Transmit data via SPI with byte-level control
 *
 * Thread-safe SPI transmission using binary semaphore for mutual exclusion.
 * Transmits data byte-by-byte with interleaved delays to meet
 * ADS1299 multi-byte command timing requirements (section 9.5.3.1 of
 * datasheet).
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
			(STM32_interface_handler_t *)self->interface_Handler;
	assert(interface->delay != NULL);

#if FREERTOS
	xSemaphoreTake(interface->mutex, portMAX_DELAY);
#endif // FREERTOS

	uint8_t buff[3] = {0};
	buff[0] = ADS_CMD_RREG + (uint8_t)reg;

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
	xSemaphoreGive(interface->mutex);
#endif // FREERTOS

	return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////
#if SPI_DMA
uint32_t dummy = 0;

void ads_spi_rx_DMA_stop_and_prepare(ads_t *self, uint8_t *buff, uint16_t len) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t *)self->interface_Handler;
	assert(buff != NULL);

	LL_GPIO_SetOutputPin(interface->cs_port, interface->cs_pin);

	LL_DMA_DisableStream(interface->dma, interface->spi_rx_dma_stream); // RX
	LL_DMA_DisableStream(interface->dma, interface->spi_tx_dma_stream); // TX

	LL_DMA_ConfigAddresses(interface->dma, interface->spi_rx_dma_stream,
			LL_SPI_DMA_GetRegAddr(interface->spi), (uint32_t)buff,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_ConfigAddresses(
			interface->dma, interface->spi_tx_dma_stream, (uint32_t)&dummy,
			LL_SPI_DMA_GetRegAddr(interface->spi), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
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

void ads_interface_spi_rx_sample_DMA_prepare_next(ads_t *self, uint8_t *buff) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t *)self->interface_Handler;

	int bytes_to_read = self->num_channels * 3 + 3;
	assert(buff != NULL);

	LL_GPIO_SetOutputPin(interface->cs_port, interface->cs_pin);

	LL_DMA_DisableStream(interface->dma, interface->spi_rx_dma_stream); // RX
	LL_DMA_DisableStream(interface->dma, interface->spi_tx_dma_stream); // TX

	LL_DMA_ConfigAddresses(interface->dma, interface->spi_rx_dma_stream,
			LL_SPI_DMA_GetRegAddr(interface->spi), (uint32_t)buff,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_ConfigAddresses(
			interface->dma, interface->spi_tx_dma_stream, (uint32_t)&dummy,
			LL_SPI_DMA_GetRegAddr(interface->spi), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	// Configure data length
	LL_DMA_SetDataLength(interface->dma, interface->spi_rx_dma_stream,
			bytes_to_read);
	LL_DMA_SetDataLength(interface->dma, interface->spi_tx_dma_stream,
			bytes_to_read);

	LL_SPI_EnableDMAReq_RX(interface->spi);
	LL_SPI_EnableDMAReq_TX(interface->spi);

	// Clear DMA transfer complete flags
	LL_DMA_ClearFlag_TC3(interface->dma); // Stream 3 - RX
	LL_DMA_ClearFlag_TC4(interface->dma); // Stream 4 - TX
	// Enable DMA Transfer complete interrupt
	LL_DMA_EnableIT_TC(interface->dma, interface->spi_rx_dma_stream);
}

void ads_interface_spi_rx_sample_DMA(ads_t *self) {
	assert(self != NULL);
	assert(self->interface_Handler != NULL);

	STM32_interface_handler_t *interface =
			(STM32_interface_handler_t *)self->interface_Handler;
	LL_GPIO_ResetOutputPin(interface->cs_port, interface->cs_pin);

	LL_DMA_EnableStream(interface->dma, interface->spi_rx_dma_stream); // RX
	LL_DMA_EnableStream(interface->dma, interface->spi_tx_dma_stream); // TX
}

#endif // SPI_DMA
