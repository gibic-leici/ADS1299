# ADS1299 Interrupt Example for STM32F407

This project provides an example of using the `ads1299` library via the `/DRDY` falling edge interrupt on an STM32F407 microcontroller.

## Application Overview

The main program demonstrates the usage of the library through the following sequence:

1.  **Interface Initialization:** Initializes the STM32-specific interface structure, mapping the required GPIOs and the SPI port.
2.  **Configuration:** Initializes the setup structure with the desired configuration (16 ksps, 24 V/V gain, and test mode enabled on all channels) and links it with the interface structure.
3.  **Device Initialization:** Calls `ads_init()` to initialize and configure the ADS1299 device.
4.  **Start Acquisition:** Calls `ads_start()` to begin continuous data acquisition.
5.  **Interrupt Handling:** The program enables the EXTI interrupt for the `/DRDY` pin (falling edge). When a new sample is ready, the `EXTI4_IRQHandler()` interrupt function reads and stores the data in a ping-pong buffer perfectly sized to stream appropriately over USB CDC, instead of reading it from the `main()` loop.

## Hardware Configuration

The project is an STM32CubeIDE project, generated with STM32CubeMX, and configured with the following settings:

*   **Main Clock:** 144 MHz (sourced from an 8 MHz external crystal).
*   **USB:** USB OTG HS port configured with the CDC profile at FS (Full Speed). DMA is enabled to prevent USB transmissions from consuming CPU time.
*   **SPI:** SPI2 used for communication with the ADS1299.
*   **`/DRDY`:** PC4 configured as an external interrupt input (EXTI, falling edge trigger).
*   **`/CS`:** PC5 configured as an output.
*   **`/PWDN` & `/RESET`:** PB0 configured as an output (both ADS1299 inputs are short-circuited together).
*   **Debug:** SWD interface enabled.
*   **Profiling Outputs:** PD12, PD13, and PD14 configured as outputs to measure CPU utilization times for different tasks.

## Performance Measurements

Measurements of CPU utilization time were conducted for the various software tasks. The code was compiled using the `-O0` and `-Og` optimization flags.

In both build configurations, it is evident that the USB transmission consumes practically no CPU time, owing to the use of DMA transmissions and the configuration of large 9600-byte transfers.

The overall CPU utilization is practically the same as in the polling case, and is entirely caused by reading the samples from within the `EXTI4_IRQHandler()`:
*   **66% CPU utilization** when compiled with the `-O0` flag.
*   **28% CPU utilization** when compiled with the `-Og` flag.

### Logic Analyzer Captures

**Interrupt with `-O0` Optimization (Overall View):**
![Interrupt -O0 Capture 1](polling_O0.png)

**Interrupt with `-O0` Optimization (Zoomed View):**
![Interrupt -O0 Capture 2](polling_O0_2.png)

**Interrupt with `-Og` Optimization:**
![Interrupt -Og Capture](polling_Og.png)
