# ADS1299 Library

ADS1299 driver library for microcontrollers. This project provides a compact, well-documented C driver for the Texas Instruments ADS1299 8-channel, 24-bit ADC, and includes a reference STM32F4 port implementation.

## Features

- Full register-level driver for ADS1299
- Per-channel configuration (gain, input multiplexer, enable/disable)
- Data rate configuration and readback
- Register read/write helpers with verification
- STM32F4-specific interface implementation (SPI, DMA, EXTI)
- Thread-safe SPI access (semaphores)
- Doxygen-style documentation throughout the codebase

## Quick Start

1. Configure hardware-specific options in `Include/ads1299lib_config.h` (pins, SPI, DMA, EXTI)
2. Implement or provide the platform interface handler (`ads_interface_t`) and populate required callbacks
3. Initialize the device:

```c
ads_t device;
ads_init_t cfg = {0};
cfg.num_channels = 8;
cfg.data_rate = ADS_DR_1KSPS;
cfg.interface_Handler = &hw_interface; // platform-specific handler
// configure channel settings in cfg.channel_config[]

if (ads_init(&device, &cfg) != R_OK) {
    // handle initialization failure
}
ads_start(&device);
```

## API Overview

- `ads_init()` — Initialize and configure ADS1299
- `ads_start()` / `ads_stop()` — Control continuous conversion
- `ads_set_config()` — Program CONFIG1..4 and CHxSET registers (verified)
- `ads_read_reg()` / `ads_write_reg()` — Low-level register access helpers
- `ads_set_ch_mode()`, `ads_set_ch_gain()`, `ads_set_ch_enabled()` — Per-channel configuration
- `ads_get_ch_mode()`, `ads_get_ch_gain()`, `ads_get_ID()` — Readback helpers

For full function descriptions and signatures, see `Include/ads1299lib.h` and the Doxygen comments in the source files.

## Configuration and Porting Notes

- The STM32F4 port (`Ports/STM32`) uses LL drivers + CMSIS-RTOS primitives. Adapt the interface handler for other platforms.
- When changing configuration registers at runtime, the API forces the device into the stopped state — ensure this is acceptable for your application.
- Some register reads/writes will call `ads_interface_stop()` and may interrupt conversions.

## Examples

See `USAGE_GUIDE.md` for practical usage examples, interrupt usage, DMA configuration, and troubleshooting tips.

## Files

- `Include/` — Public headers and register definitions
- `Source/` — Core driver implementation
- `Ports/STM32/` — STM32F4 port implementation (reference)
- `USAGE_GUIDE.md` — Usage examples and best practices
- `LIBRARY_REVIEW_REPORT.md` — Review & documentation notes

## Author & Contact

Marcelo Haberman
- Email: marcelo.haberman@gmail.com
- Email: marcelo.haberman@ing.unlp.edu.ar
- Organization: GIBIC (gibic.ar)

## License

Please add or confirm a license for this project. (No license declared in repository.)

---

If you want, I can:
- Add a Doxygen configuration for generating HTML docs
- Port the interface implementation to other MCUs
- Add a simple example app demonstrating acquisition, parsing and storage

For next steps, tell me which of the options above you'd like to prioritize.

---

## Generating HTML documentation (Doxygen) ✅

You can generate the HTML documentation from the Doxygen comments included in the code.

- Windows (PowerShell):
  - Run: `scripts\generate_docs.ps1`
- Linux/macOS:
  - Run: `doxygen Doxyfile`

Output will be placed at `docs/doxygen/html/index.html`.

Tip: If you don't have Doxygen installed, visit https://www.doxygen.nl/ for installers and documentation.

