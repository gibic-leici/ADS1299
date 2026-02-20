#ifndef _ADS1299LIB_H
#define _ADS1299LIB_H

/**
 * @file ads1299lib.h
 * @brief Public API definitions and types for the ADS1299 driver
 *
 * Library for using the ADS1299 from any microcontroller. Users must
 * implement the platform interface and call `ads_init()` with a valid
 * configuration. See `ads1299lib_config.h` for hardware-specific settings.
 *
 * @author Marcelo Haberman <marcelo.haberman@gmail.com>,
 *         marcelo.haberman@ing.unlp.edu.ar - GIBIC (gibic.ar)
 * @date 2026-02-05
 */
#include "ads1299lib_config.h"
#include "ads1299lib_regs.h"
#include <stdint.h>

#define ADS_MAX_CHANNELS 8

/*******************************
 * RESULT AND STATE ENUMERATIONS*
 *******************************/

typedef enum { ADS_R_OK, ADS_R_FAIL } ads_result_t;
typedef enum {
  ADS_STATE_MIN = 0,
  ADS_STATE_PRE_INIT = 0,
  ADS_STATE_STOPPED,
  ADS_STATE_ACQUIRING,
  ADS_STATE_FAIL,
  ADS_STATE_MAX = ADS_STATE_FAIL
} ads_state_t;
typedef enum ads_channel_mode_e {
  ADS_CHMOD_MIN = 0,
  ADS_CHMOD_NORMAL = 0,
  ADS_CHMOD_SHORT = 1,
  ADS_CHMOD_BIAS = 2,
  ADS_CHMOD_VCC = 3,
  ADS_CHMOD_TEMP = 4,
  ADS_CHMOD_TEST = 5,
  ADS_CHMOD_BIASP = 6,
  ADS_CHMOD_BIASN = 7,
  ADS_CHMOD_MAX = 7
} ads_channel_mode_t;

typedef enum ads_datarate_e {
  ADS_DR_MIN = 0,
  ADS_DR_16KSPS = 0,
  ADS_DR_8KSPS = 1,
  ADS_DR_4KSPS = 2,
  ADS_DR_2KSPS = 3,
  ADS_DR_1KSPS = 4,
  ADS_DR_500SPS = 5,
  ADS_DR_250SPS = 6,
  ADS_DR_MAX = 6
} ads_datarate_t;

typedef enum ads_gain_e {
  ADS_GAIN_MIN = 0,
  ADS_GAIN_1 = 0,
  ADS_GAIN_2 = 1,
  ADS_GAIN_4 = 2,
  ADS_GAIN_6 = 3,
  ADS_GAIN_8 = 4,
  ADS_GAIN_12 = 5,
  ADS_GAIN_24 = 6,
  ADS_GAIN_MAX = 6
} ads_gain_t;

typedef enum ads_channel_state_e {
  ADS_CHANNEL_ENABLED = 0,
  ADS_CHANNEL_DISABLED = 1
} ads_channel_enabled_t;

typedef void *ads_interface_t;

/**
 * @struct ads_t
 * @brief Runtime state structure for an ADS1299 device instance
 */
typedef struct {
  uint8_t num_channels; ///< Number of configured channels (4, 6, or 8)
  ads_state_t status;   ///< Current device state

  // REGISTER COPIES
  ads_regs_t registers; ///< Shadow copy of all device registers

  // HARDWARE LINK
  // The library user should associate this pointer with a structure
  // defined by the user containing hardware-specific details
  ads_interface_t
      interface_Handler; ///< Opaque pointer to the platform interface handler
} ads_t;

/**
 * @struct ads_channel_cfg_t
 * @brief Per-channel configuration used during initialization
 */
typedef struct {
  ads_channel_enabled_t enabled; ///< Channel power state (enabled/disabled)
  ads_channel_mode_t mode;       ///< Input MUX mode (Normal, Short, Test, etc.)
  ads_gain_t gain;               ///< PGA gain setting
} ads_channel_cfg_t;

/**
 * @struct ads_init_t
 * @brief Configuration structure for ADS1299 initialization
 */
typedef struct {
  uint8_t num_channels;     ///< Number of active channels (4, 6, or 8)
  ads_datarate_t data_rate; ///< Data rate for ADC conversions
  ads_channel_cfg_t
      channel_config[ADS_MAX_CHANNELS]; ///< Configuration for each channel
  ads_interface_t interface_Handler;    ///< Hardware interface handler
} ads_init_t;

/* 4 Channels */
#define ADS_N_CH ADS_CONFIG_N_CH

#define ADS_STATUS_BYTES 3
#define ADS_CHANNEL_BYTES 3
#define ADS_BYTES_PER_SAMPLE (ADS_N_CH * ADS_CHANNEL_BYTES + ADS_STATUS_BYTES)

#define ADS_REF 4.5f

#define ADS_WRITE_CONFIG1_RESERVED_43 2
#define ADS_WRITE_CONFIG1_RESERVED_7 1
#define ADS_WRITE_CONFIG2_RESERVED_3 0
#define ADS_WRITE_CONFIG2_RESERVED_75 6
#define ADS_WRITE_CONFIG3_RESERVED_65 3
#define ADS_WRITE_CONFIG4_RESERVED_0 0
#define ADS_WRITE_CONFIG4_RESERVED_2 0
#define ADS_WRITE_CONFIG4_RESERVED_47 0

#define ADS_CLCK_OUTPUT_ENABLED 1
#define ADS_CLCK_OUTPUT_DISABLED 0

#define ADS_DAISY_MODE_ENABLED 0
#define ADS_DAISY_MODE_DISABLED 1

#define ADS_ID_DEVICE_ID 3
#define ADS_ID_NUCH_4 0
#define ADS_ID_NUCH_6 1
#define ADS_ID_NUCH_8 2

/**
 * @struct ads_sample_ch_t
 * @brief Raw 3-byte sample for a single ADS1299 channel
 *
 * The three bytes form a 24-bit two's complement value (MSB first).
 */
typedef union {
  uint8_t bytes[3]; ///< Raw bytes: bytes[0] is MSB, bytes[2] is LSB
} ads_sample_ch_t;

/**
 * @struct ads_sample_pkg_t
 * @brief One complete ADS1299 data frame (status word + channel samples)
 */
typedef struct {
  ads_sample_ch_t status; ///< 3-byte status word (LOFF_STATP, LOFF_STATN, GPIO)
  ads_sample_ch_t ch[ADS_N_CH]; ///< Per-channel sample data (one entry per
                                ///< configured channel)
} ads_sample_pkg_t;

/* Commands */
// System
#define ADS_CMD_WAKEUP 0x02
#define ADS_CMD_STANDBY 0x04
#define ADS_CMD_RESET 0x06
#define ADS_CMD_START 0x08
#define ADS_CMD_STOP 0x0A
// Data read
#define ADS_CMD_RDATAC 0x10
#define ADS_CMD_SDATAC 0x11
#define ADS_CMD_RDATA 0x12
// Register read
#define ADS_CMD_RREG 0x20
#define ADS_CMD_WREG 0x40

/**********************************************************/
/*        PUBLIC FUNCTION DECLARATIONS                   */
/**********************************************************/

/**
 * @brief Initialize the ADS1299 chip with the specified configuration
 *
 * Initializes the hardware interface, configures global and per-channel
 * settings, performs hardware reset and verifies register programming.
 *
 * @param self Pointer to the `ads_t` structure
 * @param init Pointer to the initialization configuration structure
 * @return ads_result_t ADS_R_OK on success, ADS_R_FAIL on failure
 * @note On success `self->status` will be set to ADS_STATE_STOPPED; on
 *       failure it will be set to ADS_STATE_FAIL.
 */
ads_result_t ads_init(ads_t *self, ads_init_t *init);

/**
 * @brief Start continuous ADC conversion
 *
 * Sends the START command to the ADS1299 to begin continuous conversions
 * on enabled channels using the hardware interface.
 *
 * @param self Pointer to the `ads_t` structure
 * @note Uses the hardware START command via the interface handler.
 */
void ads_start(ads_t *self);

/**
 * @brief Stop ADC conversion
 *
 * Sends the STOP command to the ADS1299 to halt conversions and put the
 * device in standby mode.
 *
 * @param self Pointer to the `ads_t` structure
 * @note Uses the hardware STOP command via the interface handler.
 */
void ads_stop(ads_t *self);

/**
 * @brief Perform a software reset of the ADS1299
 *
 * Sends the RESET command (soft reset) to reinitialize the serial interface
 * and reset all device registers to their default values.
 *
 * @param self Pointer to the `ads_t` structure
 * @note Requires a functional SPI interface.
 */
void ads_reset(ads_t *self);

/*******************************
 * PARAMETER CONFIGURATION      *
 *******************************/

/**
 * @brief Set input multiplexer mode for each channel
 *
 * Selects the input signal connected to each channel's input (Normal,
 * Short, Bias, VCC, Temperature, Test, BIAS+, BIAS-).
 *
 * @param self Pointer to the `ads_t` structure
 * @param channel_mode Array of channel modes (one per configured channel)
 * @return ads_result_t ADS_R_OK on success, ADS_R_FAIL on failure
 * @warning This function forces the device into the stopped state when
 *          applying the configuration. Ensure stopping conversions is
 *          acceptable before calling.
 */
ads_result_t ads_set_ch_mode(ads_t *self, ads_channel_mode_t *channel_mode);

/**
 * @brief Enable or disable individual channels
 *
 * Controls the power-down state for each channel independently. Use
 * ADS_CHANNEL_ENABLED (0) to enable or ADS_CHANNEL_DISABLED (1) to power
 * down the channel.
 *
 * @param self Pointer to the `ads_t` structure
 * @param channel_enable Array of enable/disable flags (one per configured
 * channel)
 * @return ads_result_t ADS_R_OK on success, ADS_R_FAIL on failure
 * @warning This function forces the device into the stopped state when
 *          applying the configuration. Ensure stopping conversions is
 *          acceptable before calling.
 */
ads_result_t ads_set_ch_enabled(ads_t *self,
                                ads_channel_enabled_t *channel_enable);

/**
 * @brief Set programmable gain amplifier (PGA) for each channel
 *
 * Possible gains: 1, 2, 4, 6, 8, 12, 24 (encoded as enumeration values).
 *
 * @param self Pointer to the `ads_t` structure
 * @param gain Array of gain values (one per configured channel)
 * @return ads_result_t ADS_R_OK on success, ADS_R_FAIL on failure
 * @warning This function forces the device into the stopped state when
 *          applying the configuration. Ensure stopping conversions is
 *          acceptable before calling.
 */
ads_result_t ads_set_ch_gain(ads_t *self, ads_gain_t *gain);

/**
 * @brief Set data sampling rate for all channels
 *
 * Changes the ADC sampling rate. The device is stopped while the new rate
 * is applied.
 *
 * @param self Pointer to the `ads_t` structure
 * @param data_rate Data rate enumeration value (ADS_DR_16KSPS .. ADS_DR_250SPS)
 * @return ads_result_t ADS_R_OK on success, ADS_R_FAIL on failure
 * @warning This function forces the device into the stopped state when
 *          applying the configuration. Ensure stopping conversions is
 *          acceptable before calling.
 */
ads_result_t ads_set_data_rate(ads_t *self, ads_datarate_t data_rate);

/**
 * @brief Get input multiplexer mode for all channels
 *
 * Reads the current MUX settings from all CHxSET registers and returns the
 * mode for each channel.
 *
 * @param self Pointer to the `ads_t` structure
 * @param modes Output array for channel modes (one per configured channel)
 * @note This function performs SPI reads and may stop ongoing conversions.
 * @warning The function may interrupt conversions since it stops the SPI
 *          interface during register reads. Ensure this is acceptable.
 */
void ads_get_ch_mode(ads_t *self, ads_channel_mode_t *modes);

/**
 * @brief Get current data sampling rate
 *
 * Reads back the CONFIG1 register to determine the device's configured
 * data rate.
 *
 * @param self Pointer to the `ads_t` structure
 * @return ads_datarate_t Current data rate (enumeration value)
 * @note Performs an SPI read; may interrupt conversions.
 */
ads_datarate_t ads_get_data_rate(ads_t *self);

/**
 * @brief Get gain settings for all channels
 *
 * Reads the current GAIN settings from all CHxSET registers and returns the
 * programmed gain for each channel.
 *
 * @param self Pointer to the `ads_t` structure
 * @param gain Output array for gain values (one per configured channel)
 * @note This function performs SPI reads and may stop ongoing conversions.
 */
void ads_get_ch_gain(ads_t *self, ads_gain_t *gain);

/**
 * @brief Read device ID register
 *
 * Reads the ID register from the ADS1299 to verify device presence and to
 * obtain the number of channels and revision ID.
 *
 * @param self Pointer to the `ads_t` structure
 * @return ADS_ID_reg_t Device ID register value containing DEV_ID, NU_CH and
 * REV_ID
 * @note Performs an SPI read; may interrupt conversions.
 */
ADS_ID_reg_t ads_get_ID(ads_t *self);

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
 * @note Checks for read-only registers and for channel-address mismatches.
 *       This function calls `ads_interface_stop` before SPI access.
 * @warning This function may interrupt ongoing conversions because it forces
 *          the SPI interface to stop. Ensure the device is stopped or that
 *          interruption is acceptable before calling.
 */
void ads_write_reg(ads_t *self, ads_regs_enum_t reg, uint8_t value);

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
 * @note Calls `ads_interface_stop` before performing SPI access and will
 *       return 0xFF for out-of-range channel registers.
 * @warning This function may interrupt ongoing conversions because it forces
 *          the SPI interface to stop. Ensure the device is stopped or that
 *          interruption is acceptable before calling.
 */
uint8_t ads_read_reg(ads_t *self, ads_regs_enum_t reg);

#endif
