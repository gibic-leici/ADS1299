#ifndef _ADS1299LIB_H
#define _ADS1299LIB_H

/*****************************************************
*           ADS1299 LIBRARY V1  19/07/2019
*           
* Library for using the ADS1299 from any microcontroller.
* 
* INITIALIZATION STEPS:
*   1. Implement the interface functions:
*      - ads_interface_spi_tx() - SPI transmission
*      - ads_interface_spi_rx() - SPI reception
*      - ads_interface_hard_reset() - RESET/PWDN pin control
*      - ads_interface_delay() - Delay function
*   2. Call ads_init() with proper configuration
*   3. Monitor library state using ads1299 status field
*   4. Use configuration and acquisition control functions
*
* AUTHOR: Usuario
* LAST MODIFICATION: February 4, 2026
*****************************************************/
#include <stdint.h>
#include "ads1299lib_config.h"
#include "ads1299lib_regs.h"


#define ADS_MAX_CHANNELS 8

/*******************************
* RESULT AND STATE ENUMERATIONS*
*******************************/

typedef enum resultado {R_OK, R_FAIL} ads_result_t;
typedef enum  {ADS_STATE_MIN = 0, ADS_STATE_PRE_INIT=0, ADS_STATE_STOPPED, ADS_STATE_ACQUIRING, ADS_STATE_FAIL, ADS_STATE_MAX = ADS_STATE_FAIL}  ads_state_t;
typedef enum canalmodo_e {	ADS_CHMOD_MIN = 0,
							ADS_CHMOD_NORMAL=0,
							ADS_CHMOD_CORTO=1,
							ADS_CHMOD_BIAS=2,
							ADS_CHMOD_VCC=3,
							ADS_CHMOD_TEMP=4,
							ADS_CHMOD_TEST=5,
							ADS_CHMOD_BIASP=6,
							ADS_CHMOD_BIASN=7,
							ADS_CHMOD_MAX = 7}	ads_channel_mode_t;

typedef enum tasa_e {		ADS_DR_MIN = 0,
							ADS_DR_16KSPS=0,
							ADS_DR_8KSPS=1,
							ADS_DR_4KSPS=2,
							ADS_DR_2KSPS=3,
							ADS_DR_1KSPS=4,
							ADS_DR_500SPS=5,
							ADS_DR_250SPS=6,
							ADS_DR_MAX=6}		ads_datarate_t;

typedef enum ganancia_e {	ADS_GAIN_MIN=0,
							ADS_GAIN_1=0,
							ADS_GAIN_2=1,
							ADS_GAIN_4=2,
							ADS_GAIN_6=3,
							ADS_GAIN_8=4,
							ADS_GAIN_12=5,
							ADS_GAIN_24=6,
							ADS_GAIN_MAX=6}		ads_gain_t;

typedef enum canalactivo_e {ADS_CHANNEL_ENABLED=0,
							ADS_CHANNEL_DISABLED=1}	ads_channel_enabled_t;

typedef void* ads_interface_t;


typedef struct {
	uint8_t num_channels;
	ads_state_t status;

	// REGISTER COPIES
	// Read Only ID Registers
	ADS_ID_reg_t id;
	// Global Settings Across Channels
	ADS_CONFIG1_reg_t config1;
	ADS_CONFIG2_reg_t config2;
	ADS_CONFIG3_reg_t config3;
	ADS_LOFF_reg_t loff;
	// Channel-Specific Settings
	ADS_CHnSET_reg_t chnset[ADS_MAX_CHANNELS];
	ADS_BIAS_SENSP_reg_t bias_sensp;
	ADS_BIAS_SENSN_reg_t bias_sensn;
	ADS_LOFF_SENSP_reg_t loff_sensp;
	ADS_LOFF_SENSN_reg_t loff_sensn;
	ADS_LOFF_FLIP_reg_t loff_flip;
	// Lead-Off Status Registers (Read-Only Registers)
	ADS_LOFF_STATP_reg_t loff_statp;
	ADS_LOFF_STATN_reg_t loff_statn;
	// GPIO and OTHER Registers
	ADS_GPIO_reg_t gpio;
	ADS_MISC1_reg_t misc1;
	ADS_MISC2_reg_t misc2;
	ADS_CONFIG4_reg_t config4;

	// HARDWARE LINK
	// The library user should associate this pointer with a structure
	// defined by the user containing hardware-specific details
	ads_interface_t interface_Handler;
} ads_t;

typedef struct{
	ads_channel_enabled_t enabled;
	ads_channel_mode_t mode;
	ads_gain_t gain;
} ads_channel_cfg_t;

/**
 * @struct ads_init_t
 * @brief Configuration structure for ADS1299 initialization
 */
typedef struct {
	uint8_t num_channels;           ///< Number of active channels (4, 6, or 8)
	ads_datarate_t data_rate;       ///< Data rate for ADC conversions
	ads_channel_cfg_t channel_config[ADS_MAX_CHANNELS]; ///< Configuration for each channel
	ads_interface_t interface_Handler; ///< Hardware interface handler
} ads_init_t;





/* 4 Canales */
#define ADS_N_CH ADS_CONFIG_N_CH

#define ADS_STATUS_BYTES 3
#define ADS_CHANNEL_BYTES 3
#define ADS_BYTES_PER_SAMPLE (ADS_N_CH*ADS_CHANNEL_BYTES + ADS_STATUS_BYTES)



#define ADS_REF 4.5f


#define ADS_WRITE_CONFIG1_RESERVED_43	2
#define ADS_WRITE_CONFIG1_RESERVED_7 	1
#define ADS_WRITE_CONFIG2_RESERVED_3	0
#define ADS_WRITE_CONFIG2_RESERVED_75 	6
#define ADS_WRITE_CONFIG3_RESERVED_65 	3
#define ADS_WRITE_CONFIG4_RESERVED_0 	0
#define ADS_WRITE_CONFIG4_RESERVED_2 	0
#define ADS_WRITE_CONFIG4_RESERVED_47 	0

#define ADS_CLCK_OUTPUT_ENABLED				1
#define ADS_CLCK_OUTPUT_DISABLED			0

#define ADS_DAISY_MODE_ENABLED				0
#define ADS_DAISY_MODE_DISABLED				1




#define ADS_NUCH_4 0
#define ADS_NUCH_6 1
#define ADS_NUCH_8 2

typedef union{
	uint8_t bytes[3];
}ads_sample_ch_t;

typedef struct{
	ads_sample_ch_t status;
	ads_sample_ch_t ch[ADS_N_CH];	
}ads_sample_pkg_t;


/* Comandos */
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
 * @brief Initialize the ADS1299 chip with specified configuration
 * @param self Pointer to ads_t structure
 * @param init Pointer to initialization configuration structure
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 */
ads_result_t ads_init(ads_t *self, ads_init_t *init);

/**
 * @brief Start continuous ADC conversion
 * @param self Pointer to ads_t structure
 */
void ads_start(ads_t *self);

/**
 * @brief Stop ADC conversion
 * @param self Pointer to ads_t structure
 */
void ads_stop(ads_t *self);

/**
 * @brief Reset the ADS1299 chip (software reset)
 * @param self Pointer to ads_t structure
 */
void ads_reset(ads_t *self);


/*******************************
* PARAMETER CONFIGURATION      *
*******************************/

/**
 * @brief Set channel input multiplexer mode (Normal, Short, Bias, etc.)
 * @param self Pointer to ads_t structure
 * @param canalmodo Array of channel modes
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 */
ads_result_t ads_set_ch_mode(ads_t *self, ads_channel_mode_t * canalmodo);

/**
 * @brief Enable or disable individual channels
 * @param self Pointer to ads_t structure
 * @param canalactivo Array indicating enabled (0) or disabled (1) state for each channel
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 */
ads_result_t ads_set_ch_enabled(ads_t *self, ads_channel_enabled_t* canalactivo);

/**
 * @brief Set gain for each channel
 * @param self Pointer to ads_t structure
 * @param ganancias Array of gain values for each channel
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 */
ads_result_t ads_set_ch_gain(ads_t *self, ads_gain_t *ganancias);

/**
 * @brief Set data sampling rate
 * @param self Pointer to ads_t structure
 * @param tasa Data rate enumeration value
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 */
ads_result_t ads_set_data_rate(ads_t *self, ads_datarate_t tasa);

/**
 * @brief Get current channel modes
 * @param self Pointer to ads_t structure
 * @param modos Output array for channel modes
 */
void ads_get_ch_mode(ads_t *self, ads_channel_mode_t *modos);

/**
 * @brief Get current sampling data rate
 * @param self Pointer to ads_t structure
 * @return ads_datarate_t Current data rate
 */
ads_datarate_t ads_get_data_rate(ads_t *self);

/**
 * @brief Get gain settings for all channels
 * @param self Pointer to ads_t structure
 * @param ganancias Output array for gain values
 */
void ads_get_ch_gain(ads_t *self, ads_gain_t *ganancias);

/**
 * @brief Read device ID register
 * @param self Pointer to ads_t structure
 * @return ADS_ID_reg_t Device ID register value
 */
ADS_ID_reg_t ads_get_ID(ads_t *self);


#endif
