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
// LL includes removed
#include <assert.h>

/**
 * @brief Enable GPIO Clock
 */
static void enable_gpio_clock(GPIO_TypeDef *port) {
  if (port == GPIOA)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  else if (port == GPIOB)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  else if (port == GPIOC)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  else if (port == GPIOD)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  else if (port == GPIOE)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  else if (port == GPIOF)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
  else if (port == GPIOG)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
  else if (port == GPIOH)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
  else if (port == GPIOI)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
}

/**
 * @brief Configure GPIO Pin
 */
static void config_gpio_pin(GPIO_TypeDef *port, uint32_t pin_mask,
                            uint32_t mode, uint32_t type, uint32_t speed,
                            uint32_t pull, uint32_t af) {
  uint32_t pin_pos = 0;
  while ((pin_mask >> pin_pos) != 1)
    pin_pos++;

  // Mode
  port->MODER &= ~(3U << (pin_pos * 2));
  port->MODER |= (mode << (pin_pos * 2));

  // Type
  port->OTYPER &= ~(1U << pin_pos);
  port->OTYPER |= (type << pin_pos);

  // Speed
  port->OSPEEDR &= ~(3U << (pin_pos * 2));
  port->OSPEEDR |= (speed << (pin_pos * 2));

  // Pull
  port->PUPDR &= ~(3U << (pin_pos * 2));
  port->PUPDR |= (pull << (pin_pos * 2));

  // AF
  if (mode == 2) { // AF Mode
    if (pin_pos < 8) {
      port->AFR[0] &= ~(0xF << (pin_pos * 4));
      port->AFR[0] |= (af << (pin_pos * 4));
    } else {
      port->AFR[1] &= ~(0xF << ((pin_pos - 8) * 4));
      port->AFR[1] |= (af << ((pin_pos - 8) * 4));
    }
  }
}

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

  interface->prst_port->BSRR = (interface->prst_pin << 16); // Reset Low
  interface->delay(100);
  interface->prst_port->BSRR = interface->prst_pin; // Reset High
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
  if (interface->cs_port != NULL)
    enable_gpio_clock(interface->cs_port);
  if (interface->drdy_port != NULL)
    enable_gpio_clock(interface->drdy_port);
  if (interface->prst_port != NULL)
    enable_gpio_clock(interface->prst_port);

  // Configure CS pin as output (push-pull, no pull, High Speed)
  // Output, PushPull, HighSpeed, NoPull, NoAF
  config_gpio_pin(interface->cs_port, interface->cs_pin, 1, 0, 3, 0, 0);
  interface->cs_port->BSRR = interface->cs_pin; // Set High (Inactive)

  // Configure RESET/PWDN pin as output (push-pull, no pull, High Speed)
  config_gpio_pin(interface->prst_port, interface->prst_pin, 1, 0, 3, 0, 0);
  interface->prst_port->BSRR = interface->prst_pin; // Set High (Inactive)

  // Configure DRDY pin as input for data ready interrupt/polling with Pull-up
  // Input(0), PushPull(0), HighSpeed(0), PullUp(1), NoAF
  config_gpio_pin(interface->drdy_port, interface->drdy_pin, 0, 0, 0, 1, 0);
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

#if defined(SPI1)
  if (spi == SPI1) {
    // Enable SPI1 peripheral clock (APB2)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA5(SCK), PA6(MISO), PA7(MOSI) as alternate function AF5
    // Mode: AF(2), Output Type: PushPull(0), Speed: VeryHigh(3), Pull: No(0)
    config_gpio_pin(GPIOA, (1 << 5) | (1 << 6) | (1 << 7), 2, 0, 3, 0, 5);
    return;
  }
#endif

#if defined(SPI2)
  if (spi == SPI2) {
    // Enable SPI2 peripheral clock (APB1)
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // Enable GPIOB and GPIOC clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure PB10(SCK) as alternate function AF5
    config_gpio_pin(GPIOB, (1 << 10), 2, 0, 3, 0, 5);

    // Configure PC2(MISO), PC3(MOSI) as alternate function AF5
    config_gpio_pin(GPIOC, (1 << 2) | (1 << 3), 2, 0, 3, 0, 5);
    return;
  }
#endif

#if defined(SPI3)
  if (spi == SPI3) {
    // Enable SPI3 peripheral clock (APB1)
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    // Enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure PC10(SCK), PC11(MISO), PC12(MOSI) as alternate function AF6
    config_gpio_pin(GPIOC, (1 << 10) | (1 << 11) | (1 << 12), 2, 0, 3, 0, 6);
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

  // Configure SPI CR1
  // BR: Div8 (010) for safe speed.
  // ADS1299 max SCLK is 20MHz.
  // APB2 (SPI1) 84MHz / 8 = 10.5MHz.
  // APB1 (SPI2/3) 42MHz / 8 = 5.25MHz.
  // This is safe and reliable.
  // CPOL=0, CPHA=1 (Mode 1).
  // DFF=8bit (0). MSB First (0). SSM=1, SSI=1, MSTR=1.
  spi->CR1 = (2 << 3) |  // BR: Div8
             (0 << 1) |  // CPOL: Low
             (1 << 0) |  // CPHA: 2Edge
             (0 << 11) | // DFF: 8-bit
             (0 << 7) |  // LSBFIRST: MSB First
             (1 << 9) |  // SSM: Enabled
             (1 << 8) |  // SSI: Internal High
             (1 << 2);   // MSTR: Master

  spi->CR1 &= ~(1U << 13); // CRC disable
  spi->CR2 = 0;            // SSOE disabled, TXEIE/RXNEIE disabled

#if SPI_DMA
  // Initialize DMA streams for SPI transfers
  assert(interface->dma != NULL);

  /* DMA controller clock enable */
  if (interface->dma == DMA1) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    /* Select IRQ based on configured RX stream */
    // To match original code structure, we check stream index using
    // (stream_address - base) logic? No, let's just use the stream index
    // provided by the user (simulated) or just assume valid IRQ logic. Original
    // code mapped LL_DMA_STREAM_X to IRQn. Since we removed LL, we need a way
    // to map stream pointer to IRQn if generic. But wait,
    // interface->spi_rx_dma_stream is uint32_t. In LL it was likely the Stream
    // Index (0..7). Let's assume it IS the stream index (0..7).
    uint32_t stream_idx = interface->spi_rx_dma_stream;
    IRQn_Type rx_irq = (IRQn_Type)0;

    switch (stream_idx) {
    case 0:
      rx_irq = DMA1_Stream0_IRQn;
      break;
    case 1:
      rx_irq = DMA1_Stream1_IRQn;
      break;
    case 2:
      rx_irq = DMA1_Stream2_IRQn;
      break;
    case 3:
      rx_irq = DMA1_Stream3_IRQn;
      break;
    case 4:
      rx_irq = DMA1_Stream4_IRQn;
      break;
    case 5:
      rx_irq = DMA1_Stream5_IRQn;
      break;
    case 6:
      rx_irq = DMA1_Stream6_IRQn;
      break;
    case 7:
      rx_irq = DMA1_Stream7_IRQn;
      break;
    }

    if (rx_irq) {
      NVIC_SetPriority(rx_irq,
                       NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
      NVIC_EnableIRQ(rx_irq);
    }
  } else if (interface->dma == DMA2) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    uint32_t stream_idx = interface->spi_rx_dma_stream;
    IRQn_Type rx_irq = (IRQn_Type)0;

    switch (stream_idx) {
    case 0:
      rx_irq = DMA2_Stream0_IRQn;
      break;
    case 1:
      rx_irq = DMA2_Stream1_IRQn;
      break;
    case 2:
      rx_irq = DMA2_Stream2_IRQn;
      break;
    case 3:
      rx_irq = DMA2_Stream3_IRQn;
      break;
    case 4:
      rx_irq = DMA2_Stream4_IRQn;
      break;
    case 5:
      rx_irq = DMA2_Stream5_IRQn;
      break;
    case 6:
      rx_irq = DMA2_Stream6_IRQn;
      break;
    case 7:
      rx_irq = DMA2_Stream7_IRQn;
      break;
    }

    if (rx_irq) {
      NVIC_SetPriority(rx_irq,
                       NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
      NVIC_EnableIRQ(rx_irq);
    }
  }

  DMA_Stream_TypeDef *rx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_rx_dma_stream);
  DMA_Stream_TypeDef *tx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_tx_dma_stream);

  // Disable DMA streams
  rx_stream->CR &= ~DMA_SxCR_EN;
  while (rx_stream->CR & DMA_SxCR_EN)
    ;
  tx_stream->CR &= ~DMA_SxCR_EN;
  while (tx_stream->CR & DMA_SxCR_EN)
    ;

  // Configure RX DMA stream
  // Channel 0 (Bits 25-27 = 000). Priority Low (00).
  // Dir: PeriphToMem (00). Mode: Normal (0).
  // Inc: P=0, M=1.
  // Data: Byte (00).
  rx_stream->CR = (0 << 25) |         // Channel 0
                  (0 << 16) |         // Priority Low
                  (0 << 13) |         // MSIZE Byte
                  (0 << 11) |         // PSIZE Byte
                  (1 << 10) |         // MINC Enable
                  (0 << 9) |          // PINC Disable
                  (0 << 6);           // Dir PeriphToMem
  rx_stream->FCR &= ~DMA_SxFCR_DMDIS; // Direct mode enabled (Fifo disabled)

  // Configure TX DMA stream
  // Channel 0 (Bits 25-27 = 000). Priority Low (00).
  // Dir: MemToPeriph (01). Mode: Normal (0).
  // Inc: P=0, M=0 (No Increment for dummy writes).
  tx_stream->CR = (0 << 25) |         // Channel 0
                  (0 << 16) |         // Priority Low
                  (0 << 13) |         // MSIZE Byte
                  (0 << 11) |         // PSIZE Byte
                  (0 << 10) |         // MINC Disable
                  (0 << 9) |          // PINC Disable
                  (1 << 6);           // Dir MemToPeriph
  tx_stream->FCR &= ~DMA_SxFCR_DMDIS; // Direct mode enabled
#endif                                // SPI_DMA
}

void init_drdy_interrupt(STM32_interface_handler_t *interface) {
  assert(interface != NULL);

  // Configure EXTI line for DRDY pin (falling edge detection)
  // Map GPIO port/pin to SYSCFG EXTICR so EXTI line is routed correctly.
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
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

  // Configure EXTI Line
  // Enable Interrupt Mask (IMR)
  EXTI->IMR |= interface->drdy_EXTI_line;
  // Rising/Falling Edge
  EXTI->RTSR &= ~interface->drdy_EXTI_line; // Disable Rising
  EXTI->FTSR |= interface->drdy_EXTI_line;  // Enable Falling

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
  interface->spi->CR1 |= SPI_CR1_SPE;
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
  EXTI->IMR &= ~interface->drdy_EXTI_line;
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
  EXTI->PR = interface->drdy_EXTI_line; // Clear Pending
  EXTI->IMR |= interface->drdy_EXTI_line;
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
  while ((spi->SR & SPI_SR_BSY) || !(spi->SR & SPI_SR_TXE)) {                  \
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
#endif                                                  // FREERTOS
  interface->cs_port->BSRR = (interface->cs_pin << 16); // CS Low
  for (int j = 0; j < len; j++) {
    WAIT(interface->spi);
    *(__IO uint8_t *)&interface->spi->DR = *(buff + j);
    WAIT(interface->spi);

    volatile uint8_t tmp =
        *(__IO uint8_t *)&interface->spi->DR; // Read to clear RXNE
    (void)tmp;
    interface->delay(1);
    // Delay for multi-byte command timing per ADS1299 datasheet section 9.5.3.1
  }
  interface->cs_port->BSRR = interface->cs_pin; // CS High
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
#endif                                                  // FREERTOS
  interface->cs_port->BSRR = (interface->cs_pin << 16); // CS Low
  for (int j = 0; j < bytes_to_read; j++) {
    WAIT(interface->spi);
    *(__IO uint8_t *)&interface->spi->DR = 0x00;
    WAIT(interface->spi);

    *(buff + j) = *(__IO uint8_t *)&interface->spi->DR;
  }
  interface->cs_port->BSRR = interface->cs_pin; // CS High
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
#endif                                                  // FREERTOS
  interface->cs_port->BSRR = (interface->cs_pin << 16); // CS Low
  for (int j = 0; j < len; j++) {
    WAIT(interface->spi);
    *(__IO uint8_t *)&interface->spi->DR = 0x00;
    WAIT(interface->spi);

    *(buff + j) = *(__IO uint8_t *)&interface->spi->DR;
  }
  // Wait minimum 4 Tclk (approx 7µs) before raising CS per ADS1299 datasheet
  for (int k = 100; k > 0; k--)
    __NOP();
  interface->cs_port->BSRR = interface->cs_pin; // CS High
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

  interface->cs_port->BSRR = (interface->cs_pin << 16); // CS Low
  for (int j = 0; j < 3; j++) {
    WAIT(interface->spi);
    *(__IO uint8_t *)&interface->spi->DR = *(buff + j);
    WAIT(interface->spi);

    ret = *(__IO uint8_t *)&interface->spi->DR;
    interface->delay(1);
    // Delay for multi-byte command timing per ADS1299 datasheet section 9.5.3.1
  }
  interface->cs_port->BSRR = interface->cs_pin; // CS High
#if FREERTOS
  xSemaphoreGive(interface->mutex);
#endif // FREERTOS

  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////
#if SPI_DMA
uint32_t dummy = 0;

static void dma_clear_tc(DMA_TypeDef *dma, uint32_t stream_idx) {
  if (stream_idx < 4) {
    // LIFCR: Streams 0-3
    // Stream 0: Bit 5 (ZF), 4 (DME), 3 (TE), 2 (HT), 0 (FE)?? No, TC is
    // Transfer Complete. TC Flag positions: Stream 0: Bit 5 Stream 1: Bit 11
    // Stream 2: Bit 21
    // Stream 3: Bit 27
    static const uint8_t shifts[] = {5, 11, 21, 27};
    dma->LIFCR |= (1U << shifts[stream_idx]);
  } else {
    // HIFCR: Streams 4-7
    // Same shifts
    static const uint8_t shifts[] = {5, 11, 21, 27};
    dma->HIFCR |= (1U << shifts[stream_idx - 4]);
  }
}

void ads_spi_rx_DMA_stop_and_prepare(ads_t *self, uint8_t *buff, uint16_t len) {
  assert(self != NULL);
  assert(self->interface_Handler != NULL);

  STM32_interface_handler_t *interface =
      (STM32_interface_handler_t *)self->interface_Handler;
  assert(buff != NULL);

  DMA_Stream_TypeDef *rx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_rx_dma_stream);
  DMA_Stream_TypeDef *tx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_tx_dma_stream);

  interface->cs_port->BSRR = (interface->cs_pin << 16); // CS Low

  rx_stream->CR &= ~DMA_SxCR_EN; // Disable RX
  tx_stream->CR &= ~DMA_SxCR_EN; // Disable TX
  while (rx_stream->CR & DMA_SxCR_EN)
    ;
  while (tx_stream->CR & DMA_SxCR_EN)
    ;

  // Config Addresses
  // RX: Valid Buffer
  rx_stream->PAR = (uint32_t)&interface->spi->DR;
  rx_stream->M0AR = (uint32_t)buff;
  // TX: Dummy
  tx_stream->PAR = (uint32_t)&interface->spi->DR;
  tx_stream->M0AR = (uint32_t)&dummy;

  // Config Length
  rx_stream->NDTR = len;
  tx_stream->NDTR = len;

  // Enable SPI DMA Req
  interface->spi->CR2 |= SPI_CR2_RXDMAEN;
  interface->spi->CR2 |= SPI_CR2_TXDMAEN;

  // Clear DMA TC Flags
  dma_clear_tc(interface->dma, interface->spi_rx_dma_stream);
  dma_clear_tc(interface->dma, interface->spi_tx_dma_stream);

  // Enable TC Interrupt for RX
  rx_stream->CR |= DMA_SxCR_TCIE;
}

void ads_interface_spi_rx_sample_DMA_prepare_next(ads_t *self, uint8_t *buff) {
  assert(self != NULL);
  assert(self->interface_Handler != NULL);

  STM32_interface_handler_t *interface =
      (STM32_interface_handler_t *)self->interface_Handler;

  int bytes_to_read = self->num_channels * 3 + 3;
  assert(buff != NULL);

  DMA_Stream_TypeDef *rx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_rx_dma_stream);
  DMA_Stream_TypeDef *tx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_tx_dma_stream);

  interface->cs_port->BSRR = (interface->cs_pin << 16); // CS Low

  rx_stream->CR &= ~DMA_SxCR_EN; // Disable RX
  tx_stream->CR &= ~DMA_SxCR_EN; // Disable TX
  while (rx_stream->CR & DMA_SxCR_EN)
    ;
  while (tx_stream->CR & DMA_SxCR_EN)
    ;

  // Config Addresses
  rx_stream->PAR = (uint32_t)&interface->spi->DR;
  rx_stream->M0AR = (uint32_t)buff;
  tx_stream->PAR = (uint32_t)&interface->spi->DR;
  tx_stream->M0AR = (uint32_t)&dummy;

  // Config Length
  rx_stream->NDTR = bytes_to_read;
  tx_stream->NDTR = bytes_to_read;

  // Enable SPI DMA Req
  interface->spi->CR2 |= SPI_CR2_RXDMAEN;
  interface->spi->CR2 |= SPI_CR2_TXDMAEN;

  // Clear DMA TC Flags
  dma_clear_tc(interface->dma, interface->spi_rx_dma_stream);
  dma_clear_tc(interface->dma, interface->spi_tx_dma_stream);

  // Enable TC Interrupt for RX
  rx_stream->CR |= DMA_SxCR_TCIE;
}

void ads_interface_spi_rx_sample_DMA(ads_t *self) {
  assert(self != NULL);
  assert(self->interface_Handler != NULL);

  STM32_interface_handler_t *interface =
      (STM32_interface_handler_t *)self->interface_Handler;

  DMA_Stream_TypeDef *rx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_rx_dma_stream);
  DMA_Stream_TypeDef *tx_stream =
      (DMA_Stream_TypeDef *)((uint32_t)interface->dma + 0x10 +
                             0x18 * interface->spi_tx_dma_stream);

  interface->cs_port->BSRR = (interface->cs_pin << 16); // CS Low

  rx_stream->CR |= DMA_SxCR_EN; // Enable RX
  tx_stream->CR |= DMA_SxCR_EN; // Enable TX
}

#endif // SPI_DMA
