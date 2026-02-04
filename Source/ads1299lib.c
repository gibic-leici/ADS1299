#include "ads1299lib.h"
#include "ads1299lib_interface.h"
#include "freertos.h"
#include <assert.h>
#include <stddef.h>

// Forward declarations of internal functions
void ads_hard_reset	(ads_t *self);
void ads_reset		(ads_t *self);
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
 * @return ads_result_t R_OK if initialization successful, R_FAIL otherwise
 * 
 * @note Asserts will fail if:
 *   - self or init are NULL
 *   - num_channels is not 4, 6, or 8
 *   - data_rate is out of valid range
 *   - interface_Handler is NULL
 *   - channel configurations are invalid
 */
ads_result_t ads_init(ads_t *self, ads_init_t *init){
	assert(self!=NULL);
	assert( init->num_channels == 4 || init->num_channels == 6 || init->num_channels == 8);
	assert( init->data_rate >= ADS_DR_MIN && init->data_rate <= ADS_DR_MAX);
	assert( init->interface_Handler != NULL);
	for(int i=0;i<init->num_channels;i++){
		assert( init->channel_config[i].enabled == ADS_CHANNEL_ENABLED || init->channel_config[i].enabled == ADS_CHANNEL_DISABLED);
		assert( init->channel_config[i].gain >= ADS_GAIN_MIN  && init->channel_config[i].gain <= ADS_GAIN_MAX);
		assert( init->channel_config[i].mode >= ADS_CHMOD_MIN && init->channel_config[i].mode <= ADS_CHMOD_MAX);
	}

	*self = (ads_t){0};
	self->status = ADS_STATE_MIN;
	self->num_channels = init->num_channels;
	self->interface_Handler = init->interface_Handler;

	if(R_OK != ads_interface_init(self)){
		return R_FAIL;
	}

	self->config1.bits.CLK_EN = ADS_CLCK_OUTPUT_ENABLED;
	self->config1.bits.DAISY_EN = ADS_DAISY_MODE_DISABLED;
	self->config1.bits.DR = init->data_rate;
	self->config1.bits.RESERVED_43 = ADS_WRITE_CONFIG1_RESERVED_43; 		//Always write 2h
	self->config1.bits.RESERVED_7  = ADS_WRITE_CONFIG1_RESERVED_7;  		//Always write 1h

	self->config2.bits.INT_CAL = 1; 		// Test signal generated internally
	self->config2.bits.CAL_FREQ = 0; 		// 00: Pulsed at fCLK / 2^21 (approx. 1Hz)
	self->config2.bits.CAL_AMP = 0; 		// 0: 1 × –(VREFP – VREFN) / 2400 (approx. 1.9mV)
	self->config2.bits.RESERVED_3 = ADS_WRITE_CONFIG2_RESERVED_3;		// Always write 0h
	self->config2.bits.RESERVED_75 = ADS_WRITE_CONFIG2_RESERVED_75;		// Always write 6h

	self->config3.bits.BIAS_STAT = 0;		// Read-only
	self->config3.bits.BIAS_LOFF_SENS = 0; 	// 0: BIAS sense is disabled
	self->config3.bits.PD_BIAS = 1; 		// 1: BIAS buffer is enabled
	self->config3.bits.BIASREF_INT = 1;		// 1: BIASREF signal (AVDD + AVSS) / 2 generated internally
	self->config3.bits.BIAS_MEAS = 0; 		// 0: Open
	self->config3.bits.RESERVED_65 = ADS_WRITE_CONFIG3_RESERVED_65;		// Always write 3h
	self->config3.bits.PD_REFBUF = 1;		// 1: Enable internal reference buffer


	self->config4.bits.RESERVED_0 = ADS_WRITE_CONFIG4_RESERVED_0; 		// Always write 0h
	self->config4.bits.PD_LOFF_COMP = 0; 		// 0: Lead-off comparators disabled
	self->config4.bits.RESERVED_2 = ADS_WRITE_CONFIG4_RESERVED_2; 		// Always write 0h
	self->config4.bits.SINGLE_SHOT = 0;		// 0: Continuous conversion mode
	self->config4.bits.RESERVED_47 = ADS_WRITE_CONFIG4_RESERVED_47;		// Always write 0h

	for(int i=0;i<self->num_channels;i++){
		self->chnset[i].bits.PD = init->channel_config[i].enabled;
		self->chnset[i].bits.GAIN = init->channel_config[i].gain;
		self->chnset[i].bits.MUX = init->channel_config[i].mode;
	}

	ads_interface_hard_reset(self);
	ads_reset(self);
	ads_interface_stop(self);
	if(ads_set_config(self)== R_OK){
		self->status = ADS_STATE_STOPPED;
	}
	else{
		self->status = ADS_STATE_FAIL;
		return R_FAIL;
	}

	// Clear ADS1299 FIFO by sending dummy data
	uint8_t buff[3]={0};
	ads_interface_spi_tx(self,buff,3);
	ads_interface_delay(self, 100);
	return R_OK;

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
void ads_hard_reset(ads_t *self){
	ads_interface_hard_reset(self);
}

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
void ads_reset(ads_t *self){
	uint8_t buff[1];
	buff[0] = ADS_CMD_RESET;

	ads_interface_stop(self);
	ads_interface_spi_tx(self,buff, 1);
	ads_interface_delay(self,100);

	self->status = ADS_STATE_MIN;
}

ads_result_t ads_set_config(ads_t *self){

	assert(self != NULL);

	uint8_t buff[10];

	ads_interface_stop(self);

	// Write CONFIG1 register
	buff[0] = ADS_CMD_WREG+dirCONFIG1;
	buff[1] = 0x00;
	buff[2] = self->config1.byte;
	ads_interface_spi_tx(self,buff,3);
	ads_interface_delay(self,10);

	// Write CONFIG2 register
	buff[0] = ADS_CMD_WREG+dirCONFIG2;
	buff[1] = 0x00;
	buff[2] = self->config2.byte;
	ads_interface_spi_tx(self,buff,3);
	ads_interface_delay(self,10);

	// Write CONFIG3 register
	buff[0] = ADS_CMD_WREG+dirCONFIG3;
	buff[1] = 0x00;
	buff[2] = self->config3.byte;
	ads_interface_spi_tx(self,buff,3);
	ads_interface_delay(self,10);

	// Write CHxSET registers (per channel configuration)
	buff[0] = ADS_CMD_WREG+dirCHxSET;
	buff[1] = self->num_channels-1u;
	for(int i=0;i<self->num_channels;i++){
		buff[i+2] = self->chnset[i].byte;
	}
	ads_interface_spi_tx(self,buff,self->num_channels + 2u);
	ads_interface_delay(self,10);

	// Write CONFIG4 register
	buff[0] = ADS_CMD_WREG+dirCONFIG4;
	buff[1] = 0x00;
	buff[2] = self->config4.byte;
	ads_interface_spi_tx(self,buff,3);
	ads_interface_delay(self,10);

	// Verify configuration was written correctly
	if(ads_verify_config(self) == R_OK){
		self->status = ADS_STATE_STOPPED;
	}
	else{
		self->status = ADS_STATE_FAIL;
		return R_FAIL;
	}

	return R_OK;
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
 * @return ads_result_t R_OK if all registers match, R_FAIL if any mismatch
 * 
 * @note Updates the status field to falla if verification fails
 */
ads_result_t ads_verify_config(ads_t *self){

	assert(self != NULL);

	ads_result_t error = R_OK;
	uint8_t buff[10];

	// Verify CONFIG1 register
	buff[0] = ADS_CMD_RREG+dirCONFIG1;
	buff[1] = 0x00;
	ads_interface_spi_tx(self, buff, 2);
	ads_interface_delay(self, 10);

	buff[0]=0x00;
	ads_interface_spi_rx(self,buff,1);

	if(	self->config1.byte != buff[0] ){
		error = R_FAIL;
	}
	// Verify CONFIG2 register
	buff[0] = ADS_CMD_RREG+dirCONFIG2;
	buff[1] = 0x00;
	ads_interface_spi_tx(self, buff, 2);
	ads_interface_delay(self, 10);

	buff[0]=0x00;
	ads_interface_spi_rx(self,buff,1);

	if(	self->config2.byte != buff[0] ){
		error = R_FAIL;
	}
	// Verify CONFIG3 register
	buff[0] = ADS_CMD_RREG+dirCONFIG3;
	buff[1] = 0x00;
	ads_interface_spi_tx(self, buff, 2);
	ads_interface_delay(self, 10);

	buff[0]=0x00;
	ads_interface_spi_rx(self,buff,1);

	if(	self->config3.byte != buff[0] ){
		error = R_FAIL;
	}
	// Verify CHxSET registers
	for(int i=0;i<self->num_channels;i++){

		buff[0] = ADS_CMD_RREG+dirCHxSET+i;
		buff[1] = 0x00;
		ads_interface_spi_tx(self, buff, 2);
		ads_interface_delay(self, 10);

		buff[0]=0x00;
		ads_interface_spi_rx(self,buff,1);

		if(	self->chnset[i].byte != buff[0] ){
			error = R_FAIL;
		}
	}
	// Verify CONFIG4 register
	buff[0] = ADS_CMD_RREG+dirCONFIG4;
	buff[1] = 0x00;
	ads_interface_spi_tx(self, buff, 2);
	ads_interface_delay(self, 10);

	buff[0]=0x00;
	ads_interface_spi_rx(self,buff,1);

	if(	self->config4.byte != buff[0] ){
		error = R_FAIL;
	}

	if(error==R_FAIL){
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
void ads_start(ads_t *self){ads_interface_start(self);};

/**
 * @brief Stop ADC conversion
 * 
 * Stops continuous ADC conversion and puts the device in standby mode.
 * Uses hardware interface STOP command.
 * 
 * @param self Pointer to ads_t structure
 */
void ads_stop(ads_t *self){ads_interface_stop(self);};


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
 * @param tasa Data rate enumeration value (ADS_DR_16KSPS through ADS_DR_250SPS)
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 * 
 * @warning Device must be stopped before calling this function
 */
ads_result_t ads_set_data_rate(ads_t *self, ads_datarate_t tasa){

	assert(ADS_DR_MIN <= tasa && tasa <= ADS_DR_MAX);
	assert(self != NULL);

	self->config1.bits.DR = tasa;

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
 * @param ganancias Array of gain values (one per channel)
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 * 
 * @warning Device must be stopped before calling this function
 */
ads_result_t ads_set_ch_gain(ads_t *self, ads_gain_t *ganancias){
	
	assert(self != NULL);
	for(int i=0;i<self->num_channels;i++){
		assert(ADS_GAIN_MIN <= ganancias[i]  && ganancias[i] <= ADS_GAIN_MAX);
	}
	
	for(int i=0;i<self->num_channels;i++){
		self->chnset[i].bits.GAIN = ganancias[i];
	}
	
	ads_interface_stop(self);

	return ads_set_config(self);

}

/**
 * @brief Enable or disable individual channels
 * 
 * Controls power-down mode for each channel independently.
 * Set to ADS_CHANNEL_ENABLED (0) to enable or ADS_CHANNEL_DISABLED (1) to power down.
 * 
 * @param self Pointer to ads_t structure
 * @param canalactivo Array of enable/disable flags (one per channel)
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 * 
 * @warning Device must be stopped before calling this function
 */
ads_result_t ads_set_ch_enabled(ads_t *self, ads_channel_enabled_t* canalactivo){

	assert(self != NULL);
	for(int i=0;i<self->num_channels;i++){
		assert(canalactivo[i]==ADS_CHANNEL_ENABLED || canalactivo[i]==ADS_CHANNEL_DISABLED);
	}

	for(int i=0;i<self->num_channels;i++){
		self->chnset[i].bits.PD = canalactivo[i];
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
 * @param canalmodo Array of channel modes (one per channel)
 * @return ads_result_t R_OK if successful, R_FAIL otherwise
 * 
 * @warning Device must be stopped before calling this function
 */
ads_result_t ads_set_ch_mode(ads_t *self, ads_channel_mode_t * canalmodo){

	assert(self != NULL);
	for(int i=0;i<self->num_channels;i++){
		assert(ADS_CHMOD_MIN <= canalmodo[i]  && canalmodo[i] <= ADS_CHMOD_MAX);
	}

	for(int i=0;i<self->num_channels;i++){
		self->chnset[i].bits.MUX = canalmodo[i];
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
ads_datarate_t ads_get_data_rate(ads_t *self){

	assert(self != NULL);

	ads_interface_stop(self);

	uint8_t buff[2];
	buff[0] = ADS_CMD_RREG+dirCONFIG1;
	buff[1] = 0x00;
	ads_interface_spi_tx(self, buff, 2);
	ads_interface_delay(self, 10);

	buff[0]=0x00;
	ads_interface_spi_rx(self,buff,1);
	self->config1.byte = buff[0];

	return (ads_datarate_t)(self->config1.bits.DR);
}


/**
 * @brief Get input multiplexer mode for all channels
 * 
 * Reads the current MUX settings from all CHxSET registers.
 * Returns what signal is connected to each channel's input.
 * 
 * @param self Pointer to ads_t structure
 * @param modos Output array for channel modes
 */
void ads_get_ch_mode(ads_t *self, ads_channel_mode_t *modos)
{

	assert(self != NULL);
	assert(modos != NULL);

	uint8_t buff[2];

	ads_interface_stop(self);
	for(int i=0;i<self->num_channels;i++){
		buff[0] = ADS_CMD_RREG+dirCHxSET+i;
		buff[1] = 0;
		ads_interface_spi_tx(self, buff, 2);
		ads_interface_delay(self, 10);

		buff[0]=0;
		ads_interface_spi_rx(self,buff,1);
		self->chnset[i].byte = buff[0];
		modos[i] = self->chnset[i].bits.MUX;
	}
}

/**
 * @brief Get gain settings for all channels
 * 
 * Reads the current GAIN settings from all CHxSET registers.
 * Returns the programmed gain amplifier value for each channel.
 * 
 * @param self Pointer to ads_t structure
 * @param ganancias Output array for gain values
 */
void ads_get_ch_gain(ads_t *self, ads_gain_t *ganancias)
{
	assert(self != NULL);
	assert(ganancias != NULL);

	uint8_t buff[2];

	ads_interface_stop(self);
	for(int i=0;i<self->num_channels;i++){
		buff[0] = ADS_CMD_RREG+dirCHxSET+i;
		buff[1] = 0;
		ads_interface_spi_tx(self, buff, 2);
		ads_interface_delay(self, 10);

		buff[0]=0;
		ads_interface_spi_rx(self,buff,1);
		self->chnset[i].byte = buff[0];
		ganancias[i] = self->chnset[i].bits.GAIN;
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
ADS_ID_reg_t ads_get_ID(ads_t *self){

	assert(self != NULL);

	ads_interface_stop(self);

	uint8_t buff[2];
	buff[0] = ADS_CMD_RREG+dirID;
	buff[1] = 0x00;
	ads_interface_spi_tx(self, buff, 2);
	ads_interface_delay(self, 10);

	buff[0]=0x00;
	ads_interface_spi_rx(self,buff,1);
	self->id.byte = buff[0];

	return self->id;
}

///////////////////////////////////////////////////////////////








//enum resultado ADS_visualizar_registros(uint8_t * arr){
//
//		int i;
//
//		txBuf[0] = ADS_CMD_RREG+dirID;
//		txBuf[1] = 0x03;
//		ADS_spi_tx(txBuf,2);
//		ADS_Delay_ms(10);
//		rxBuf[0]=rxBuf[1]=rxBuf[2]=rxBuf[3]=0x00;
//		ADS_spi_rx(rxBuf,4);
//
//		*(arr+0) =  rxBuf[0];
//		*(arr+1) =  rxBuf[1];
//		*(arr+2) =  rxBuf[2];
//		*(arr+3) =  rxBuf[3];
//
//		txBuf[0] = ADS_CMD_RREG+dirCHxSET;
//    txBuf[1] = (char)(ADS_N_CH-1);
//    ADS_spi_tx(txBuf,2);
//		ADS_Delay_ms(10);
//		rxBuf[0]=rxBuf[1]=rxBuf[2]=rxBuf[3]=0x00;
//		ADS_spi_rx(rxBuf,ADS_N_CH);
//
//		for(i=0;i<ADS_N_CH;i++){
//			*(arr+4+i)=rxBuf[i];
//		}
//
//		txBuf[0] = ADS_CMD_RREG+dirCONFIG4;
//		txBuf[1] = 0x00;
//		ADS_spi_tx(txBuf,2);
//		ADS_Delay_ms(10);
//		rxBuf[0]=0x00;
//		ADS_spi_rx(rxBuf,1);
//
//		*(arr+8)= rxBuf[0];
//
//
//		//estado_lib_ads = falla;
//		if(ADS_verificar_configuracion() == R_OK){
//			//estado_lib_ads = conf_programada;
//		}
//		else{
//			//estado_lib_ads = falla;
//			return R_FAIL;
//		}
//
//		return R_OK;
//}






