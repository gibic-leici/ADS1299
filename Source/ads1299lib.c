/**
 * @file ads1299lib.c
 * @brief Core implementation of the ADS1299 driver library
 *
 * Implements initialization, configuration programming, register read/write,
 * and basic data acquisition control for the ADS1299 ADC.
 *
 * @author Marcelo Haberman <marcelo.haberman@gmail.com>,
 *         marcelo.haberman@ing.unlp.edu.ar - GIBIC (gibic.ar)
 * @date 2026-02-05
 */

#include "ads1299lib.h"
#include "ads1299lib_interface.h"
#include <assert.h>
#include <stddef.h>

// Forward declarations of internal functions
void ads_hard_reset(ads_t *self);
void ads_reset(ads_t *self);
ads_result_t ads_set_config(ads_t *self);
ads_result_t ads_verify_config(ads_t *self);

/**
 * @brief Initialize ADS1299 with specified configuration
 *
 * This function performs the complete initialization of the ADS1299 ADC:
 * - Initializes hardware interface
 * - Configures global settings (clock, data rate, BIAS)
 * - Configures individual channel settings (gain, mode)
 * - Performs hardware reset and verification
 *
 * @param self Pointer to ads_t structure
 * @param init Pointer to initialization configuration
 * @return ads_result_t ADS_R_OK if initialization successful, ADS_R_FAIL
 * otherwise
 *
 * @note Asserts will fail if:
 *   - self or init are NULL
 *   - num_channels is not 4, 6, or 8
 *   - data_rate is out of valid range
 *   - interface_Handler is NULL
 *   - channel configurations are invalid
 */
ads_result_t ads_init(ads_t *self, ads_init_t *init) {
  assert(self != NULL);
  assert(init->num_channels == 4 || init->num_channels == 6 ||
         init->num_channels == 8);
  assert(init->data_rate >= ADS_DR_MIN && init->data_rate <= ADS_DR_MAX);
  assert(init->interface_Handler != NULL);
  for (int i = 0; i < init->num_channels; i++) {
    assert(init->channel_config[i].enabled == ADS_CHANNEL_ENABLED ||
           init->channel_config[i].enabled == ADS_CHANNEL_DISABLED);
    assert(init->channel_config[i].gain >= ADS_GAIN_MIN &&
           init->channel_config[i].gain <= ADS_GAIN_MAX);
    assert(init->channel_config[i].mode >= ADS_CHMOD_MIN &&
           init->channel_config[i].mode <= ADS_CHMOD_MAX);
  }

  *self = (ads_t){0};
  self->status = ADS_STATE_MIN;
  self->num_channels = init->num_channels;
  self->interface_Handler = init->interface_Handler;

  if (ADS_R_OK != ads_interface_init(self)) {
    return ADS_R_FAIL;
  }

  self->registers.config1.bits.CLK_EN = ADS_CLCK_OUTPUT_ENABLED;
  self->registers.config1.bits.DAISY_EN = ADS_DAISY_MODE_DISABLED;
  self->registers.config1.bits.DR = init->data_rate;
  self->registers.config1.bits.RESERVED_43 =
      ADS_WRITE_CONFIG1_RESERVED_43; // Always write 2h
  self->registers.config1.bits.RESERVED_7 =
      ADS_WRITE_CONFIG1_RESERVED_7; // Always write 1h

  self->registers.config2.bits.INT_CAL = 1; // Test signal generated internally
  self->registers.config2.bits.CAL_FREQ =
      0; // 00: Pulsed at fCLK / 2^21 (approx. 1Hz)
  self->registers.config2.bits.CAL_AMP =
      0; // 0: 1 × –(VREFP – VREFN) / 2400 (approx. 1.9mV)
  self->registers.config2.bits.RESERVED_3 =
      ADS_WRITE_CONFIG2_RESERVED_3; // Always write 0h
  self->registers.config2.bits.RESERVED_75 =
      ADS_WRITE_CONFIG2_RESERVED_75; // Always write 6h

  self->registers.config3.bits.BIAS_STAT = 0;      // Read-only
  self->registers.config3.bits.BIAS_LOFF_SENS = 0; // 0: BIAS sense is disabled
  self->registers.config3.bits.PD_BIAS = 1;        // 1: BIAS buffer is enabled
  self->registers.config3.bits.BIASREF_INT =
      1; // 1: BIASREF signal (AVDD + AVSS) / 2 generated internally
  self->registers.config3.bits.BIAS_MEAS = 0; // 0: Open
  self->registers.config3.bits.RESERVED_65 =
      ADS_WRITE_CONFIG3_RESERVED_65; // Always write 3h
  self->registers.config3.bits.PD_REFBUF =
      1; // 1: Enable internal reference buffer

  self->registers.config4.bits.RESERVED_0 =
      ADS_WRITE_CONFIG4_RESERVED_0; // Always write 0h
  self->registers.config4.bits.PD_LOFF_COMP =
      0; // 0: Lead-off comparators disabled
  self->registers.config4.bits.RESERVED_2 =
      ADS_WRITE_CONFIG4_RESERVED_2;             // Always write 0h
  self->registers.config4.bits.SINGLE_SHOT = 0; // 0: Continuous conversion mode
  self->registers.config4.bits.RESERVED_47 =
      ADS_WRITE_CONFIG4_RESERVED_47; // Always write 0h

  for (int i = 0; i < self->num_channels; i++) {
    self->registers.chnset[i].bits.PD = init->channel_config[i].enabled;
    self->registers.chnset[i].bits.GAIN = init->channel_config[i].gain;
    self->registers.chnset[i].bits.MUX = init->channel_config[i].mode;
  }

  ads_interface_hard_reset(self);
  ads_reset(self);
  ads_stop(self);

  // Reading and testing ID
  self->registers.id.byte = ads_read_reg(self, ADS_ID);
  if (self->registers.id.bits.DEV_ID != ADS_ID_DEVICE_ID) {
    self->status = ADS_STATE_FAIL;
    return ADS_R_FAIL;
  }
  switch (self->num_channels) {
  case 4:
    if (self->registers.id.bits.NU_CH != ADS_ID_NUCH_4) {
      self->status = ADS_STATE_FAIL;
      return ADS_R_FAIL;
    }
    break;
  case 6:
    if (self->registers.id.bits.NU_CH != ADS_ID_NUCH_6) {
      self->status = ADS_STATE_FAIL;
      return ADS_R_FAIL;
    }
    break;
  case 8:
    if (self->registers.id.bits.NU_CH != ADS_ID_NUCH_8) {
      self->status = ADS_STATE_FAIL;
      return ADS_R_FAIL;
    }
    break;
  default:
    self->status = ADS_STATE_FAIL;
    return ADS_R_FAIL;
  }

  if (ads_set_config(self) == ADS_R_OK) {
    self->status = ADS_STATE_STOPPED;
  } else {
    self->status = ADS_STATE_FAIL;
    return ADS_R_FAIL;
  }

  // Clear ADS1299 FIFO by sending dummy data
  uint8_t buff[3] = {0};
  ads_interface_spi_tx(self, buff, 3);
  ads_interface_delay(self, 100);
  return ADS_R_OK;
}

/**
 * @brief Perform hardware reset on ADS1299 chip
 *
 * Toggles the RESET pin to perform a complete hardware reset.
 * After reset, the chip returns to its default state with all
 * registers loaded with their reset values.
 *
 * @param self Pointer to ads_t structure
 *
 * @warning This function does not verify the reset completion
 * @see ads_interface_hard_reset() for hardware-specific implementation
 */
void ads_hard_reset(ads_t *self) { ads_interface_hard_reset(self); }

/**
 * @brief Perform software reset via SPI command
 *
 * Sends the RESET command (0x06) via SPI to reset the device.
 * This is a soft reset that initializes the serial interface
 * and resets all registers to their default values.
 *
 * @param self Pointer to ads_t structure
 *
 * @note Requires SPI interface to be functional
 */
void ads_reset(ads_t *self) {
  uint8_t buff[1];
  buff[0] = ADS_CMD_RESET;

  ads_interface_stop(self);
  ads_interface_spi_tx(self, buff, 1);
  ads_interface_delay(self, 100);

  self->status = ADS_STATE_MIN;
}

/**
 * @brief Set ADS1299 configuration
 *
 * Writes the configuration registers CONFIG1 through CONFIG4 and the
 * per-channel settings (CHxSET) according to the values stored in the
 * internal `self->registers` structure.
 *
 * @param self Pointer to the `ads_t` structure
 * @return ads_result_t ADS_R_OK if the write and verification succeed,
 *         ADS_R_FAIL if an error occurs while writing or verifying registers
 *
 * @note The function verifies the written values by reading them back using
 *       `ads_verify_config`. It updates `self->status` to ADS_STATE_STOPPED
 *       or ADS_STATE_FAIL depending on the result.
 * @warning This function forces the device into the stopped state. Call it
 *          only when the device is stopped or when forcing a stop is
 *          acceptable.
 */
ads_result_t ads_set_config(ads_t *self) {

  assert(self != NULL);

  ads_write_reg(self, ADS_CONFIG1, self->registers.config1.byte);
  ads_write_reg(self, ADS_CONFIG2, self->registers.config2.byte);
  ads_write_reg(self, ADS_CONFIG3, self->registers.config3.byte);
  for (int i = 0; i < self->num_channels; i++) {
    ads_write_reg(self, ADS_CH1SET + i, self->registers.chnset[i].byte);
  }
  ads_write_reg(self, ADS_CONFIG4, self->registers.config4.byte);

  // Verify configuration was written correctly
  if (ads_verify_config(self) == ADS_R_OK) {
    self->status = ADS_STATE_STOPPED;
  } else {
    self->status = ADS_STATE_FAIL;
    return ADS_R_FAIL;
  }

  return ADS_R_OK;
}

/**
 * @brief Verify that all register values were written correctly
 *
 * Reads back all configuration registers and compares them with
 * the expected values stored in the ads_t structure. This ensures
 * that SPI communication was successful and the chip configuration
 * is as expected.
 *
 * @param self Pointer to ads_t structure
 * @return ads_result_t ADS_R_OK if all registers match, ADS_R_FAIL if any
 * mismatch
 *
 * @note Updates the `self->status` field to ADS_STATE_FAIL if verification
 * fails
 */
ads_result_t ads_verify_config(ads_t *self) {

  assert(self != NULL);

  ads_result_t error = ADS_R_OK;
  uint8_t buff[1];

  // Verify CONFIG1 register
  buff[0] = ads_read_reg(self, ADS_CONFIG1);
  if (self->registers.config1.byte != buff[0]) {
    error = ADS_R_FAIL;
  }

  // Verify CONFIG2 register
  buff[0] = ads_read_reg(self, ADS_CONFIG2);
  if (self->registers.config2.byte != buff[0]) {
    error = ADS_R_FAIL;
  }

  // Verify CONFIG3 register
  buff[0] = ads_read_reg(self, ADS_CONFIG3);
  if (self->registers.config3.byte != buff[0]) {
    error = ADS_R_FAIL;
  }

  // Verify CHxSET registers
  for (int i = 0; i < self->num_channels; i++) {
    buff[0] = ads_read_reg(self, (ADS_CH1SET + i));
    if (self->registers.chnset[i].byte != buff[0]) {
      error = ADS_R_FAIL;
    }
  }

  // Verify CONFIG4 register
  buff[0] = ads_read_reg(self, ADS_CONFIG4);
  if (self->registers.config4.byte != buff[0]) {
    error = ADS_R_FAIL;
  }

  if (error == ADS_R_FAIL) {
    self->status = ADS_STATE_FAIL;
  }
  return error;
}

/**
 * @brief Start ADC conversion
 *
 * Initiates continuous ADC conversion on all enabled channels.
 * Uses hardware interface START command.
 *
 * @param self Pointer to ads_t structure
 */
void ads_start(ads_t *self) {

  {
    uint8_t cmd = ADS_CMD_RDATAC;
    ads_interface_spi_tx(self, &cmd, 1);
  }
  {
    uint8_t cmd = ADS_CMD_START;
    ads_interface_spi_tx(self, &cmd, 1);
  }
  ads_interface_start(self);
  self->status = ADS_STATE_ACQUIRING;
}

/**
 * @brief Stop ADC conversion
 *
 * Stops continuous ADC conversion and puts the device in standby mode.
 * Uses hardware interface STOP command.
 *
 * @param self Pointer to ads_t structure
 */
void ads_stop(ads_t *self) {

  ads_interface_stop(self);

  {
    uint8_t cmd = ADS_CMD_SDATAC;
    ads_interface_spi_tx(self, &cmd, 1);
  }
  {
    uint8_t cmd = ADS_CMD_STOP;
    ads_interface_spi_tx(self, &cmd, 1);
  }
  self->status = ADS_STATE_STOPPED;
}

/*******************************************
**** PARAMETER PROGRAMMING ROUTINES  ****
*******************************************/

/**
 * @brief Set data sampling rate for all channels
 *
 * Changes the ADC sampling rate. The device must be stopped before
 * changing the data rate.
 *
 * @param self Pointer to ads_t structure
 * @param data_rate Data rate enumeration value (ADS_DR_16KSPS through
 * ADS_DR_250SPS)
 * @return ads_result_t ADS_R_OK if successful, ADS_R_FAIL otherwise
 *
 * @warning Device must be stopped before calling this function
 */
ads_result_t ads_set_data_rate(ads_t *self, ads_datarate_t data_rate) {

  assert(ADS_DR_MIN <= data_rate && data_rate <= ADS_DR_MAX);
  assert(self != NULL);

  self->registers.config1.bits.DR = data_rate;

  ads_interface_stop(self);

  return ads_set_config(self);
}

/**
 * @brief Set gain for each channel
 *
 * Configures the programmable gain amplifier (PGA) for each channel.
 * Possible gains: 1, 2, 4, 6, 8, 12, 24
 *
 * @param self Pointer to ads_t structure
 * @param gain Array of gain values (one per channel)
 * @return ads_result_t ADS_R_OK if successful, ADS_R_FAIL otherwise
 *
 * @warning Device must be stopped before calling this function
 */
ads_result_t ads_set_ch_gain(ads_t *self, ads_gain_t *gain) {

  assert(self != NULL);
  for (int i = 0; i < self->num_channels; i++) {
    assert(ADS_GAIN_MIN <= gain[i] && gain[i] <= ADS_GAIN_MAX);
  }

  for (int i = 0; i < self->num_channels; i++) {
    self->registers.chnset[i].bits.GAIN = gain[i];
  }

  ads_interface_stop(self);

  return ads_set_config(self);
}

/**
 * @brief Enable or disable individual channels
 *
 * Controls power-down mode for each channel independently.
 * Set to ADS_CHANNEL_ENABLED (0) to enable or ADS_CHANNEL_DISABLED (1) to power
 * down the channel.
 *
 * @param self Pointer to ads_t structure
 * @param channel_enable Array of enable/disable flags (one per channel)
 * @return ads_result_t ADS_R_OK if successful, ADS_R_FAIL otherwise
 *
 * @warning Device must be stopped before calling this function
 */
ads_result_t ads_set_ch_enabled(ads_t *self,
                                ads_channel_enabled_t *channel_enable) {

  assert(self != NULL);
  for (int i = 0; i < self->num_channels; i++) {
    assert(channel_enable[i] == ADS_CHANNEL_ENABLED ||
           channel_enable[i] == ADS_CHANNEL_DISABLED);
  }

  for (int i = 0; i < self->num_channels; i++) {
    self->registers.chnset[i].bits.PD = channel_enable[i];
  }

  ads_interface_stop(self);

  return ads_set_config(self);
}

/**
 * @brief Set input multiplexer mode for each channel
 *
 * Selects what signal is connected to each channel's input:
 * - Normal (0): External electrode
 * - Short (1): Input shorted to ground
 * - Bias (2): Bias signal
 * - VCC (3): Supply voltage
 * - Temperature (4): Internal temperature sensor
 * - Test (5): Internal test signal
 * - BIAS+ (6): Bias drive positive
 * - BIAS- (7): Bias drive negative
 *
 * @param self Pointer to ads_t structure
 * @param channel_mode Array of channel modes (one per channel)
 * @return ads_result_t ADS_R_OK on success, ADS_R_FAIL on error
 *
 * @warning The device will be stopped while applying the new configuration
 */
ads_result_t ads_set_ch_mode(ads_t *self, ads_channel_mode_t *channel_mode) {

  assert(self != NULL);
  for (int i = 0; i < self->num_channels; i++) {
    assert(ADS_CHMOD_MIN <= channel_mode[i] &&
           channel_mode[i] <= ADS_CHMOD_MAX);
  }

  for (int i = 0; i < self->num_channels; i++) {
    self->registers.chnset[i].bits.MUX = channel_mode[i];
  }

  ads_interface_stop(self);

  return ads_set_config(self);
}

/**
 * @brief Read current data sampling rate
 *
 * Reads back the CONFIG1 register to determine the current data rate setting.
 * This queries the actual chip configuration.
 *
 * @param self Pointer to ads_t structure
 * @return ads_datarate_t Current data rate enumeration value
 */
ads_datarate_t ads_get_data_rate(ads_t *self) {

  assert(self != NULL);

  uint8_t config1;

  config1 = ads_read_reg(self, ADS_CONFIG1);

  self->registers.config1.byte = config1;

  return (ads_datarate_t)(self->registers.config1.bits.DR);
}

/**
 * @brief Get input multiplexer mode for all channels
 *
 * Reads the current MUX settings from all CHxSET registers.
 * Returns what signal is connected to each channel's input.
 *
 * @param self Pointer to ads_t structure
 * @param modes Output array for channel modes
 */
void ads_get_ch_mode(ads_t *self, ads_channel_mode_t *modes) {

  assert(self != NULL);
  assert(modes != NULL);

  for (int i = 0; i < self->num_channels; i++) {
    uint8_t chxset;

    chxset = ads_read_reg(self, (ADS_CH1SET + i));

    self->registers.chnset[i].byte = chxset;

    modes[i] = self->registers.chnset[i].bits.MUX;
  }
}

/**
 * @brief Get gain settings for all channels
 *
 * Reads the current GAIN settings from all CHxSET registers.
 * Returns the programmed gain amplifier value for each channel.
 *
 * @param self Pointer to ads_t structure
 * @param gain Output array for gain values
 */
void ads_get_ch_gain(ads_t *self, ads_gain_t *gain) {
  assert(self != NULL);
  assert(gain != NULL);

  for (int i = 0; i < self->num_channels; i++) {
    uint8_t chxset;

    chxset = ads_read_reg(self, (ADS_CH1SET + i));

    self->registers.chnset[i].byte = chxset;

    gain[i] = self->registers.chnset[i].bits.GAIN;
  }
}

/**
 * @brief Read device ID register
 *
 * Reads the ID register from the ADS1299 to verify device presence and
 * obtain information about the number of channels and revision ID.
 *
 * @param self Pointer to ads_t structure
 * @return ADS_ID_reg_t Device ID register value containing:
 *         - DEV_ID: Device identification
 *         - NU_CH: Number of channels
 *         - REV_ID: Revision identification
 */
ADS_ID_reg_t ads_get_ID(ads_t *self) {

  assert(self != NULL);

  uint8_t id;

  id = ads_read_reg(self, ADS_ID);

  self->registers.id.byte = id;

  return self->registers.id;
}

/**
 * @brief Write ADS1299 register
 *
 * Writes a single register on the ADS1299 via SPI using the WREG command.
 *
 * @param self Pointer to the `ads_t` structure
 * @param reg Register enumeration (address) to write
 * @param value Value to write into the register
 * @return void
 *
 * @note This function checks for read-only registers (e.g., ID, LOFF_STATP,
 *       LOFF_STATN) and for channel-address mismatches. If the register is
 *       read-only or invalid for the configured number of channels, the
 *       function returns without performing any write.
 *       It calls `ads_interface_stop` before performing SPI access.
 * @warning This function may interrupt ongoing conversions because it forces
 *          the SPI interface to stop. Ensure the device is stopped or that
 *          interruption is acceptable before calling.
 */
void ads_write_reg(ads_t *self, ads_regs_enum_t reg, uint8_t value) {
  assert(ADS_REG_MIN <= reg && reg <= ADS_REG_MAX);
  assert(self != NULL);

  // If register is read-only
  if (reg == ADS_ID || reg == ADS_LOFF_STATN || reg == ADS_LOFF_STATP) {
    return;
  }
  // If channel number is invalid
  if (self->num_channels == 4 && reg >= ADS_CH5SET && reg <= ADS_CH8SET) {
    return;
  }
  if (self->num_channels == 6 && reg >= ADS_CH7SET && reg <= ADS_CH8SET) {
    return;
  }

  uint8_t buff[3];

  ads_interface_stop(self);

  // Write register
  buff[0] = ADS_CMD_WREG +
            (uint8_t)reg; // Reg number in enum is equal to address in device
  buff[1] = 0x00;
  buff[2] = value;
  ads_interface_spi_tx(self, buff, 3);
  ads_interface_delay(self, 10);
}

/**
 * @brief Read ADS1299 register
 *
 * Reads a single register from the ADS1299 via SPI using the RREG command
 * and returns its value.
 *
 * @param self Pointer to the `ads_t` structure
 * @param reg Register enumeration (address) to read
 * @return uint8_t The value read from the register. For invalid channel
 *         addresses (based on configured channel count) the function
 *         returns 0xFF.
 *
 * @note This function calls `ads_interface_stop` before performing SPI access
 *       and will return 0xFF for out-of-range channel registers.
 * @warning This function may interrupt ongoing conversions because it forces
 *          the SPI interface to stop. Ensure the device is stopped or that
 *          interruption is acceptable before calling.
 */
uint8_t ads_read_reg(ads_t *self, ads_regs_enum_t reg) {
  assert(ADS_REG_MIN <= reg && reg <= ADS_REG_MAX);
  assert(self != NULL);

  // If channel number is invalid
  if (self->num_channels == 4 && reg >= ADS_CH5SET && reg <= ADS_CH8SET) {
    return 0xFF;
  }
  if (self->num_channels == 6 && reg >= ADS_CH7SET && reg <= ADS_CH8SET) {
    return 0xFF;
  }

  ads_stop(self);

  return ads_interface_read_reg(self, reg);
}

///////////////////////////////////////////////////////////////
