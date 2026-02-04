# ADS1299 Library - Best Practices & Usage Guide

**Version:** 1.0  
**Language:** English  
**Last Updated:** February 4, 2026

---

## 📖 Table of Contents

1. [Initialization](#initialization)
2. [Configuration](#configuration)
3. [Data Acquisition](#data-acquisition)
4. [Interrupt Handling](#interrupt-handling)
5. [Error Handling](#error-handling)
6. [Thread Safety](#thread-safety)
7. [Performance Tips](#performance-tips)
8. [Troubleshooting](#troubleshooting)

---

## Initialization

### Basic Initialization Example

```c
#include "ads1299lib.h"

// Create ADS1299 instance and configuration
ads_t ads_device;
STM32_interface_handler_t hw_interface = {
    .spi = SPI2,
    .dma = DMA1,
    .spi_rx_dma_stream = LL_DMA_STREAM_3,
    .spi_tx_dma_stream = LL_DMA_STREAM_4,
    .cs_port = GPIOC,
    .cs_pin = GPIO_PIN_5,
    .prst_port = GPIOB,
    .prst_pin = GPIO_PIN_0,
    .drdy_port = GPIOB,
    .drdy_pin = GPIO_PIN_1,
    .drdy_EXTI_IRQn = EXTI4_IRQn,
    .drdy_EXTI_line = LL_EXTI_LINE_4,
    .delay = HAL_Delay,  // Provide your delay function
    .mutex = NULL  // Will be created by ads_interface_init()
};

// Configure 8 channels with equal settings
ads_init_t init_config = {
    .num_channels = 8,
    .data_rate = ADS_DR_1KSPS,  // 1 kHz sampling rate
    .interface_Handler = &hw_interface,
};

// Initialize all channels with same configuration
for(int i = 0; i < 8; i++) {
    init_config.channel_config[i].enabled = ADS_CHANNEL_ENABLED;
    init_config.channel_config[i].gain = ADS_GAIN_24;  // 24x gain
    init_config.channel_config[i].mode = ADS_CHMOD_NORMAL;  // Normal electrode mode
}

// Perform initialization
ads_result_t result = ads_init(&ads_device, &init_config);
if(result == R_OK) {
    printf("ADS1299 initialized successfully\n");
} else {
    printf("ADS1299 initialization failed\n");
    // Handle error
}
```

### Initialization States

```
UNINITIALIZED (sin_iniciar)
    ↓
HARDWARE RESET
    ↓
SOFTWARE RESET
    ↓
CONFIGURATION WRITING
    ↓
CONFIGURATION VERIFICATION
    ↓
STOPPED (detenido) ← Ready to start acquisition
    ↓
START COMMAND
    ↓
ACQUIRING (adquiriendo) ← Data ready on DRDY interrupt
```

---

## Configuration

### Setting Data Rate

```c
// Change sampling rate (device must be stopped)
ads_result_t result = ads_set_data_rate(&ads_device, ADS_DR_500SPS);

if(result == R_OK) {
    printf("Data rate changed to 500 SPS\n");
} else {
    printf("Failed to change data rate\n");
}

// Available rates:
// ADS_DR_16KSPS (0) - 16,000 samples/second
// ADS_DR_8KSPS  (1) - 8,000 samples/second
// ADS_DR_4KSPS  (2) - 4,000 samples/second
// ADS_DR_2KSPS  (3) - 2,000 samples/second
// ADS_DR_1KSPS  (4) - 1,000 samples/second [DEFAULT]
// ADS_DR_500SPS (5) - 500 samples/second
// ADS_DR_250SPS (6) - 250 samples/second
```

### Setting Channel Gain

```c
// Set individual channel gains
ads_gain_t gains[8] = {
    ADS_GAIN_24,  // Channel 1: 24x
    ADS_GAIN_12,  // Channel 2: 12x
    ADS_GAIN_8,   // Channel 3: 8x
    ADS_GAIN_6,   // Channel 4: 6x
    ADS_GAIN_4,   // Channel 5: 4x
    ADS_GAIN_2,   // Channel 6: 2x
    ADS_GAIN_1,   // Channel 7: 1x
    ADS_GAIN_24   // Channel 8: 24x
};

ads_result_t result = ads_set_ch_gain(&ads_device, gains);

if(result == R_OK) {
    printf("Channel gains configured\n");
}

// Reference voltage: 4.5V (defined as ADS_REF)
// Voltage per LSB = 4.5V / (2^24) = 268.45 nV
// With 24x gain: 268.45 nV * 24 = 6.44 µV per LSB
```

### Enabling/Disabling Channels

```c
// Disable specific channels for power savings
ads_channel_enabled_t ch_state[8] = {
    ADS_CHANNEL_ENABLED,      // Channel 1: On
    ADS_CHANNEL_ENABLED,      // Channel 2: On
    ADS_CHANNEL_DISABLED,     // Channel 3: Off (powered down)
    ADS_CHANNEL_DISABLED,     // Channel 4: Off
    ADS_CHANNEL_ENABLED,      // Channel 5: On
    ADS_CHANNEL_ENABLED,      // Channel 6: On
    ADS_CHANNEL_DISABLED,     // Channel 7: Off
    ADS_CHANNEL_DISABLED      // Channel 8: Off
};

ads_set_ch_enabled(&ads_device, ch_state);
```

### Setting Channel Input Mode

```c
// Configure input multiplexer for different measurement types
ads_channel_mode_t ch_mode[8] = {
    ADS_CHMOD_NORMAL,    // Normal electrode input
    ADS_CHMOD_NORMAL,    // Normal electrode input
    ADS_CHMOD_SHORT,     // Shorted to ground (for noise measurement)
    ADS_CHMOD_TEMP,      // Internal temperature sensor
    ADS_CHMOD_VCC,       // Supply voltage (VCC/2)
    ADS_CHMOD_BIAS,      // BIAS signal
    ADS_CHMOD_BIASP,     // BIAS positive derivation
    ADS_CHMOD_BIASN      // BIAS negative derivation
};

ads_set_ch_mode(&ads_device, ch_mode);

// Input modes:
// 0: NORMAL    - External electrode input
// 1: SHORT     - Input shorted to ground
// 2: BIAS      - Bias signal
// 3: VCC       - VCC/2 supply voltage reference
// 4: TEMP      - Internal temperature sensor output
// 5: TEST      - Internal test signal
// 6: BIASP     - BIAS drive positive
// 7: BIASN     - BIAS drive negative
```

---

## Data Acquisition

### Starting Acquisition

```c
// Start continuous data acquisition
ads_start(&ads_device);

if(ads_device.status == adquiriendo) {
    printf("ADC acquiring data\n");
}
// Data now becomes available on DRDY pin interrupt
```

### Stopping Acquisition

```c
// Stop data acquisition
ads_stop(&ads_device);

if(ads_device.status == detenido) {
    printf("ADC stopped\n");
}
// Device ready for reconfiguration
```

### Monitoring Device Status

```c
// Check device state
switch(ads_device.status) {
    case sin_iniciar:
        printf("Device not initialized\n");
        break;
    case iniciado_default:
        printf("Device initialized with defaults\n");
        break;
    case conf_programada:
        printf("Configuration programmed\n");
        break;
    case adquiriendo:
        printf("Device acquiring data\n");
        break;
    case detenido:
        printf("Device stopped\n");
        break;
    case adquiriendo_test:
        printf("Device acquiring test signal\n");
        break;
    case falla:
        printf("Device error state\n");
        break;
}
```

### Reading Configuration Back

```c
// Verify current configuration from device
ads_datarate_t current_rate = ads_get_data_rate(&ads_device);
printf("Current data rate: %d\n", current_rate);

// Get channel gains
ads_gain_t current_gains[8];
ads_get_ch_gain(&ads_device, current_gains);
for(int i = 0; i < 8; i++) {
    printf("Channel %d gain: %d\n", i+1, current_gains[i]);
}

// Get channel modes
ads_channel_mode_t current_modes[8];
ads_get_ch_mode(&ads_device, current_modes);

// Read device ID
ADS_ID_reg_t device_id = ads_get_ID(&ads_device);
printf("Device ID - Dev: %d, Channels: %d, Rev: %d\n", 
    device_id.bits.DEV_ID,
    device_id.bits.NU_CH,
    device_id.bits.REV_ID
);
```

---

## Interrupt Handling

### DRDY Interrupt Handler

```c
// DRDY interrupt handler (called when new data is ready)
void EXTI4_IRQHandler(void)
{
    // Clear interrupt flag
    if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
        
        // Read ADC data
        uint8_t adc_frame[ADS_BYTES_PER_SAMPLE];  // 27 bytes for 8 channels (3 status + 8*3 data)
        
        ads_interface_spi_rx(&ads_device, adc_frame, ADS_BYTES_PER_SAMPLE);
        
        // Parse status byte
        uint8_t status = adc_frame[0];
        
        // Parse channel data (24-bit signed integers)
        for(int i = 0; i < 8; i++) {
            int32_t raw_sample = 0;
            raw_sample |= (adc_frame[3*i + 1] << 16);
            raw_sample |= (adc_frame[3*i + 2] << 8);
            raw_sample |= (adc_frame[3*i + 3]);
            
            // Sign extend if negative
            if(raw_sample & 0x800000) {
                raw_sample |= 0xFF000000;
            }
            
            // Convert to voltage
            // Assuming 24x gain on all channels
            float voltage = (raw_sample / (float)(1 << 23)) * (ADS_REF / 24.0f);
            
            printf("Channel %d: %ld counts, %.3f mV\n", i+1, raw_sample, voltage * 1000);
        }
    }
}
```

### Using Semaphore to Signal Main Task

```c
// Global semaphore for data ready notification
SemaphoreHandle_t data_ready_semaphore = NULL;

void EXTI4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
        
        // Signal main task that data is ready
        xSemaphoreGiveFromISR(data_ready_semaphore, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// In main task
void adc_task(void *arguments)
{
    // Create semaphore
    data_ready_semaphore = xSemaphoreCreateBinary();
    
    ads_start(&ads_device);
    
    while(1) {
        // Wait for DRDY interrupt
        if(xSemaphoreTake(data_ready_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Read ADC data
            uint8_t adc_frame[ADS_BYTES_PER_SAMPLE];
            ads_interface_spi_rx(&ads_device, adc_frame, ADS_BYTES_PER_SAMPLE);
            
            // Process data...
        } else {
            printf("ADC timeout - DRDY not received\n");
        }
    }
}
```

---

## Error Handling

### Checking Operation Results

```c
ads_result_t result = ads_set_ch_gain(&ads_device, gains);

if(result == R_OK) {
    printf("Operation successful\n");
} else if(result == R_FAIL) {
    printf("Operation failed\n");
    printf("Device status: %d\n", ads_device.status);
    
    // Attempt recovery
    ads_reset(&ads_device);
}
```

### Parameter Validation

```c
// All public functions use assert() for parameter validation
// Enable assertions in debug builds:
// #define NDEBUG  (undefine to enable)

// Example invalid call (will assert):
ads_datarate_t invalid_rate = 99;  // Out of range
ads_set_data_rate(&ads_device, invalid_rate);  // ASSERT FAILS

// Always validate before calling:
if(data_rate >= ADS_DR_MIN && data_rate <= ADS_DR_MAX) {
    ads_set_data_rate(&ads_device, data_rate);
}
```

### Configuration Verification

```c
// After writing configuration, device automatically verifies
// If verification fails, status becomes 'falla'

if(ads_device.status == falla) {
    printf("Configuration verification failed\n");
    printf("Possible causes:\n");
    printf("- SPI communication error\n");
    printf("- Device not responding\n");
    printf("- Register write failed\n");
    
    // Recovery:
    ads_reset(&ads_device);
    // Re-initialize device
}
```

---

## Thread Safety

### Mutex Protection

```c
// The library uses binary semaphore for SPI mutual exclusion
// Each SPI transaction automatically acquires/releases the mutex

// Safe to call from multiple tasks:
osThreadCreate(osThread(task1), NULL);  // Calls ads_set_ch_gain()
osThreadCreate(osThread(task2), NULL);  // Calls ads_get_data_rate()

// Both tasks will wait if another is using SPI

// DO NOT hold SPI lock across long operations:
// ✗ WRONG - Do not do this:
xSemaphoreTake(interface->mutex, portMAX_DELAY);
ads_interface_spi_tx(...);
delay_long_operation();  // SPI locked for long time!
ads_interface_spi_rx(...);
xSemaphoreGive(interface->mutex);

// ✓ CORRECT - Release lock between operations:
xSemaphoreTake(interface->mutex, portMAX_DELAY);
ads_interface_spi_tx(...);
xSemaphoreGive(interface->mutex);

delay_long_operation();

xSemaphoreTake(interface->mutex, portMAX_DELAY);
ads_interface_spi_rx(...);
xSemaphoreGive(interface->mutex);
```

### Task-Safe Data Sharing

```c
// Create queue for passing ADC data between interrupt and task
QueueHandle_t adc_queue = xQueueCreate(10, ADS_BYTES_PER_SAMPLE);

void EXTI4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t adc_frame[ADS_BYTES_PER_SAMPLE];
    
    if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
        
        ads_interface_spi_rx(&ads_device, adc_frame, ADS_BYTES_PER_SAMPLE);
        
        // Queue data for processing task
        xQueueSendFromISR(adc_queue, adc_frame, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void data_processing_task(void *arg)
{
    uint8_t received_frame[ADS_BYTES_PER_SAMPLE];
    
    while(1) {
        if(xQueueReceive(adc_queue, received_frame, portMAX_DELAY)) {
            // Safe to process received_frame
            // No conflicts with interrupt handler
        }
    }
}
```

---

## Performance Tips

### 1. **Optimize SPI Clock Speed**

```c
// STM32F4 SPI clock dividers:
// Fclk / 2, 4, 8, 16, 32, 64, 128, 256
// For APB2 = 84 MHz:
// Fclk/2 = 42 MHz (may exceed ADS1299 max ~10 MHz)
// Fclk/8 = 10.5 MHz (acceptable)
// Fclk/16 = 5.25 MHz (safer)

LL_SPI_SetBaudRatePrescaler(SPI2, LL_SPI_BAUDRATE_PRESCALER_8);  // 10.5 MHz
```

### 2. **Use DMA for High-Throughput Data**

```c
// For continuous high-speed data acquisition (16 kHz, 8 channels)
// Recommended: Use DMA with interrupt-driven transfers

void ads_enable_dma_fast_acquisition(ads_t *self, uint8_t *buffer, uint16_t samples_per_buffer)
{
    // Setup DMA for continuous transfers
    ads_spi_rx_DMA_stop_and_prepare(self, buffer, samples_per_buffer);
    ads_spi_rx_DMA_start_fast(self);
    
    // Enable DMA transfer complete interrupt
    // Handle buffer swap in interrupt handler
}
```

### 3. **Minimize Configuration Changes**

```c
// Configuration changes require:
// 1. Stop ADC
// 2. Write all configuration registers
// 3. Verify configuration (SPI reads)
// 4. Restart ADC

// Consequence: ~100ms dead time per configuration change

// STRATEGY: Pre-configure multiple channel sets, then switch:
// Instead of changing gain dynamically, configure 8 different gain scenarios
// during initialization, then select at runtime
```

### 4. **Reduce Interrupt Latency**

```c
// DRDY interrupt must be high priority
// Recommended priority: 0-1 (higher than 15)

NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
```

---

## Troubleshooting

### Issue: Device Not Responding

```c
// Symptoms: ads_init() fails with R_FAIL
// Possible causes:
// 1. SPI not configured correctly
// 2. CS or RESET pin not connected
// 3. No power to ADS1299

// Debug steps:
printf("Checking hardware...\n");

// 1. Verify SPI is enabled
printf("SPI enabled: %d\n", LL_SPI_IsEnabled(ADS_SPI));

// 2. Check clock speed (should be < 10 MHz)
printf("SPI clock divider: %d\n", LL_SPI_GetBaudRatePrescaler(ADS_SPI));

// 3. Verify pins
printf("CS pin state: %d\n", LL_GPIO_IsInputPinSet(CS_GPIO_Port, CS_Pin));
printf("RESET pin state: %d\n", LL_GPIO_IsInputPinSet(PRST_GPIO_Port, PRST_Pin));

// 4. Manual reset test
LL_GPIO_ResetOutputPin(PRST_GPIO_Port, PRST_Pin);
HAL_Delay(100);
LL_GPIO_SetOutputPin(PRST_GPIO_Port, PRST_Pin);
HAL_Delay(1000);
printf("Reset completed\n");
```

### Issue: Configuration Verification Fails

```c
// Symptoms: ads_device.status == falla
// Possible causes:
// 1. SPI timing issue
// 2. Multi-byte command timing violation
// 3. DMA stream misconfiguration

// The library includes 1ms delay between SPI bytes
// This should be sufficient for ~10 MHz SPI clock

// If still failing, increase delay:
// Modify ads_interface_spi_tx() and ads_interface_spi_rx()
// to use longer delay: interface->delay(2);  // 2ms instead of 1ms
```

### Issue: DRDY Interrupt Not Triggering

```c
// Symptoms: No interrupt received after ads_start()
// Possible causes:
// 1. EXTI line not configured
// 2. NVIC interrupt not enabled
// 3. DRDY pin not connected

// Debug steps:
// 1. Verify EXTI configuration in CubeMX
// 2. Check DRDY pin state during normal operation
// 3. Verify EXTI line matches actual pin
printf("DRDY pin state: %d\n", LL_GPIO_IsInputPinSet(GPIOB, GPIO_PIN_1));

// 4. If pin never goes LOW, ADS1299 not sending data
// Verify ads_device.status == adquiriendo
```

---

## Summary Checklist

- [ ] Read entire ADS1299 datasheet section 9 (register descriptions)
- [ ] Configure STM32CubeMX with SPI and EXTI for DRDY
- [ ] Implement `delay_function(uint32_t millis)`
- [ ] Test basic initialization sequence
- [ ] Verify register readback in `ads_verify_config()`
- [ ] Test each configuration function individually
- [ ] Implement DRDY interrupt handler
- [ ] Test continuous data acquisition
- [ ] Validate ADC output against known signals
- [ ] Profile performance and optimize as needed

---

**For more information, refer to the ADS1299 datasheet and STM32F4 reference manual.**
