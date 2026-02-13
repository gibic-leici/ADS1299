/**
 * @file ads1299lib_regs.h
 * @brief Register definitions and bitfield types for ADS1299
 *
 * Contains register addresses, typed unions and bitfield structures
 * representing the ADS1299 register map (see datasheet section 9).
 *
 * @author Marcelo Haberman <marcelo.haberman@gmail.com>,
 *         marcelo.haberman@ing.unlp.edu.ar - GIBIC (gibic.ar)
 * @date 2026-02-05
 */

#ifndef INCLUDE_ADS1299LIB_REGS_H_
#define INCLUDE_ADS1299LIB_REGS_H_

#include <stdint.h>
#include "ads1299lib_config.h"


/************************************/
//9.6.1.1 ID: ID Control Register (address = 00h) (reset = xxh)

typedef struct{
	uint8_t NU_CH:2;
	uint8_t DEV_ID:2;
	uint8_t RESERVED:1;
	uint8_t REV_ID:3;
}ADS_ID_bits_t;

typedef union{
	ADS_ID_bits_t bits;
	uint8_t byte;
}ADS_ID_reg_t;


#define ADS_DEV_ID 3
#define ADS_NUCH_4 0
#define ADS_NUCH_6 1
#define ADS_NUCH_8 2

/********************************/
//9.6.1.2 CONFIG1: Configuration Register 1 (address = 01h) (reset = 96h)
//This register configures the DAISY_EN bit, clock, and data rate

typedef struct{
	uint8_t DR:3;
	uint8_t RESERVED_43:2;
	uint8_t CLK_EN:1;
	uint8_t DAISY_EN:1;
	uint8_t RESERVED_7:1;
}ADS_CONFIG1_bits_t;

typedef union{
	ADS_CONFIG1_bits_t bits;
	uint8_t byte;
}ADS_CONFIG1_reg_t;

/********************************/
//9.6.1.3 CONFIG2: Configuration Register 2 (address = 02h) (reset = C0h)
//This register configures the test signal generation. See the Input Multiplexer section for more details.

typedef struct{
	uint8_t CAL_FREQ:2;
	uint8_t CAL_AMP:1;
	uint8_t RESERVED_3:1;
	uint8_t INT_CAL:1;
	uint8_t RESERVED_75:3;
}ADS_CONFIG2_bits_t;

typedef union{
	ADS_CONFIG2_bits_t bits;
	uint8_t byte;
}ADS_CONFIG2_reg_t;

/*******************************/
//9.6.1.4 CONFIG3: Configuration Register 3 (address = 03h) (reset = 60h)
//Configuration register 3 configures either an internal or exteral reference and BIAS operation.

typedef struct{
	uint8_t BIAS_STAT:1;
	uint8_t BIAS_LOFF_SENS:1;
	uint8_t PD_BIAS:1;
	uint8_t BIASREF_INT:1;
	uint8_t BIAS_MEAS:1;
	uint8_t RESERVED_65:2;
	uint8_t PD_REFBUF:1;
}ADS_CONFIG3_bits_t;

typedef union{
	ADS_CONFIG3_bits_t bits;
	uint8_t byte;
}ADS_CONFIG3_reg_t;

/*******************************/
//9.6.1.5 LOFF: Lead-Off Control Register (address = 04h) (reset = 00h)
//The lead-off control register configures the lead-off detection operation.

typedef struct{
	uint8_t FLEAD_OFF:2;
	uint8_t ILEAD_OFF:2;
	uint8_t RESERVED_4:1;
	uint8_t COMP_TH:3;
}ADS_LOFF_bits_t;

typedef union{
	ADS_LOFF_bits_t bits;
	uint8_t byte;
}ADS_LOFF_reg_t;

/*******************************/
//9.6.1.6 CHnSET: Individual Channel Settings (n = 1 to 8) (address = 05h to 0Ch) (reset = 61h)
//The CH[1:8]SET control register configures the power mode, PGA gain, and multiplexer settings channels. See
//the Input Multiplexer section for details. CH[2:8]SET are similar to CH1SET, corresponding to the respective
//channels.
typedef struct{
	uint8_t MUX:3;
	uint8_t SRB2:1;
	uint8_t GAIN:3;
	uint8_t PD:1;
}ADS_CHnSET_bits_t;

typedef union{
	ADS_CHnSET_bits_t bits;
	uint8_t byte;
}ADS_CHnSET_reg_t;

/*******************************/
//9.6.1.7 BIAS_SENSP: Bias Drive Positive Derivation Register (address = 0Dh) (reset = 00h)
//This register controls the selection of the positive signals from each channel for bias voltage (BIAS) derivation.
//See the Bias Drive (DC Bias Circuit) section for details.
//Registers bits[5:4] are not available for the ADS1299-4. Register bits[7:6] are not available for the ADS1299-4, or
//ADS1299-6. Set unavailable bits for the associated device to 0 when writing to the register.

typedef struct{
	uint8_t BIAS1P:1;
	uint8_t BIAS2P:1;
	uint8_t BIAS3P:1;
	uint8_t BIAS4P:1;
	uint8_t BIAS5P:1;
	uint8_t BIAS6P:1;
	uint8_t BIAS7P:1;
	uint8_t BIAS8P:1;
}ADS_BIAS_SENSP_bits_t;

typedef union{
	ADS_BIAS_SENSP_bits_t bits;
	uint8_t byte;
}ADS_BIAS_SENSP_reg_t;

/*******************************/
//9.6.1.8 BIAS_SENSN: Bias Drive Negative Derivation Register (address = 0Eh) (reset = 00h)
//This register controls the selection of the negative signals from each channel for bias voltage (BIAS) derivation.
//See the Bias Drive (DC Bias Circuit) section for details.
//Registers bits[5:4] are not available for the ADS1299-4. Register bits[7:6] are not available for the ADS1299-4, or
//ADS1299-6. Set unavailable bits for the associated device to 0 when writing to the register.

typedef struct{
	uint8_t BIAS1N:1;
	uint8_t BIAS2N:1;
	uint8_t BIAS3N:1;
	uint8_t BIAS4N:1;
	uint8_t BIAS5N:1;
	uint8_t BIAS6N:1;
	uint8_t BIAS7N:1;
	uint8_t BIAS8N:1;
}ADS_BIAS_SENSN_bits_t;

typedef union{
	ADS_BIAS_SENSN_bits_t bits;
	uint8_t byte;
}ADS_BIAS_SENSN_reg_t;

/******************************/
//9.6.1.9 LOFF_SENSP: Positive Signal Lead-Off Detection Register (address = 0Fh) (reset = 00h)
//This register selects the positive side from each channel for lead-off detection. See the Lead-Off Detection
//section for details. The LOFF_STATP register bits are only valid if the corresponding LOFF_SENSP bits are set
//to 1.
//Registers bits[5:4] are not available for the ADS1299-4. Register bits[7:6] are not available for the ADS1299-4, or
//ADS1299-6. Set unavailable bits for the associated device to 0 when writing to the register.

typedef struct{
	uint8_t LOFF1P:1;
	uint8_t LOFF2P:1;
	uint8_t LOFF3P:1;
	uint8_t LOFF4P:1;
	uint8_t LOFF5P:1;
	uint8_t LOFF6P:1;
	uint8_t LOFF7P:1;
	uint8_t LOFF8P:1;
}ADS_LOFF_SENSP_bits_t;

typedef union{
	ADS_LOFF_SENSP_bits_t bits;
	uint8_t byte;
}ADS_LOFF_SENSP_reg_t;

/******************************/
//9.6.1.10 LOFF_SENSN: Negative Signal Lead-Off Detection Register (address = 10h) (reset = 00h)
//This register selects the negative side from each channel for lead-off detection. See the Lead-Off Detection
//section for details. The LOFF_STATN register bits are only valid if the corresponding LOFF_SENSN bits are set
//to 1.
//Registers bits[5:4] are not available for the ADS1299-4. Register bits[7:6] are not available for the ADS1299-4, or
//ADS1299-6. Set unavailable bits for the associated device to 0 when writing to the register.

typedef struct{
	uint8_t LOFFM1:1;
	uint8_t LOFFM2:1;
	uint8_t LOFFM3:1;
	uint8_t LOFFM4:1;
	uint8_t LOFFM5:1;
	uint8_t LOFFM6:1;
	uint8_t LOFFM7:1;
	uint8_t LOFFM8:1;
}ADS_LOFF_SENSN_bits_t;

typedef union{
	ADS_LOFF_SENSN_bits_t bits;
	uint8_t byte;
}ADS_LOFF_SENSN_reg_t;

/******************************/
//9.6.1.11 LOFF_FLIP: Lead-Off Flip Register (address = 11h) (reset = 00h)
//This register controls the direction of the current used for lead-off derivation. See the Lead-Off Detection section
//for details.

typedef struct{
	uint8_t LOFF_FLIP1:1;
	uint8_t LOFF_FLIP2:1;
	uint8_t LOFF_FLIP3:1;
	uint8_t LOFF_FLIP4:1;
	uint8_t LOFF_FLIP5:1;
	uint8_t LOFF_FLIP6:1;
	uint8_t LOFF_FLIP7:1;
	uint8_t LOFF_FLIP8:1;
}ADS_LOFF_FLIP_bits_t;

typedef union{
	ADS_LOFF_FLIP_bits_t bits;
	uint8_t byte;
}ADS_LOFF_FLIP_reg_t;

/******************************/
//9.6.1.12 LOFF_STATP: Lead-Off Positive Signal Status Register (address = 12h) (reset = 00h)
//This register stores the status of whether the positive electrode on each channel is on or off. See the Lead-Off
//Detection section for details. Ignore the LOFF_STATP values if the corresponding LOFF_SENSP bits are not set
//to 1.
//When the LOFF_SENSEP bits are 0, the LOFF_STATP bits should be ignored.

typedef struct{
	uint8_t IN1P_OFF:1;
	uint8_t IN2P_OFF:1;
	uint8_t IN3P_OFF:1;
	uint8_t IN4P_OFF:1;
	uint8_t IN5P_OFF:1;
	uint8_t IN6P_OFF:1;
	uint8_t IN7P_OFF:1;
	uint8_t IN8P_OFF:1;
}ADS_LOFF_STATP_bits_t;

typedef union{
	ADS_LOFF_STATP_bits_t bits;
	uint8_t byte;
}ADS_LOFF_STATP_reg_t;

/******************************/
//9.6.1.13 LOFF_STATN: Lead-Off Negative Signal Status Register (address = 13h) (reset = 00h)
//This register stores the status of whether the negative electrode on each channel is on or off. See the Lead-Off
//Detection section for details. Ignore the LOFF_STATN values if the corresponding LOFF_SENSN bits are not set
//to 1.
//When the LOFF_SENSEN bits are 0, the LOFF_STATP bits should be ignored.

typedef struct{
	uint8_t IN1N_OFF:1;
	uint8_t IN2N_OFF:1;
	uint8_t IN3N_OFF:1;
	uint8_t IN4N_OFF:1;
	uint8_t IN5N_OFF:1;
	uint8_t IN6N_OFF:1;
	uint8_t IN7N_OFF:1;
	uint8_t IN8N_OFF:1;
}ADS_LOFF_STATN_bits_t;

typedef union{
	ADS_LOFF_STATN_bits_t bits;
	uint8_t byte;
}ADS_LOFF_STATN_reg_t;

/******************************/
//9.6.1.14 GPIO: General-Purpose I/O Register (address = 14h) (reset = 0Fh)
//The general-purpose I/O register controls the action of the three GPIO pins. When RESP_CTRL[1:0] is in mode
//01 and 11, the GPIO2, GPIO3, and GPIO4 pins are not available for use.

typedef struct{
	uint8_t ADS_GPIOC:4;
	uint8_t ADS_GPIOD:4;
}ADS_GPIO_bits_t;

typedef union{
	ADS_GPIO_bits_t bits;
	uint8_t byte;
}ADS_GPIO_reg_t;

/******************************/
//9.6.1.15 MISC1: Miscellaneous 1 Register (address = 15h) (reset = 00h)
//This register provides the control to route the SRB1 pin to all inverting inputs of the four, six, or eight channels
//(ADS1299-4, ADS1299-6, or ADS1299).

typedef struct{
	uint8_t RESERVED_40:5;
	uint8_t SRB1:1;
	uint8_t RESERVED_76:2;
}ADS_MISC1_bits_t;

typedef union{
	ADS_MISC1_bits_t bits;
	uint8_t byte;
}ADS_MISC1_reg_t;

/******************************/
//9.6.1.16 MISC2: Miscellaneous 2 (address = 16h) (reset = 00h)
//This register is reserved for future use

typedef struct{
	uint8_t RESERVED:8;
}ADS_MISC2_bits_t;

typedef union{
	ADS_MISC2_bits_t bits;
	uint8_t byte;
}ADS_MISC2_reg_t;

/******************************/
//9.6.1.17 CONFIG4: Configuration Register 4 (address = 17h) (reset = 00h)
//This register configures the conversion mode and enables the lead-off comparators.

typedef struct{
	uint8_t RESERVED_0:1;
	uint8_t PD_LOFF_COMP:1;
	uint8_t RESERVED_2:1;
	uint8_t SINGLE_SHOT:1;
	uint8_t RESERVED_47:4;
}ADS_CONFIG4_bits_t;

typedef union{
	ADS_CONFIG4_bits_t bits;
	uint8_t byte;
}ADS_CONFIG4_reg_t;

typedef struct {
	// Read Only ID Registers
	ADS_ID_reg_t id;
	// Global Settings Across Channels
	ADS_CONFIG1_reg_t config1;
	ADS_CONFIG2_reg_t config2;
	ADS_CONFIG3_reg_t config3;
	ADS_LOFF_reg_t loff;
	// Channel-Specific Settings
	ADS_CHnSET_reg_t chnset[8];
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
}ads_regs_t;

typedef enum {
	ADS_REG_MIN,
	ADS_ID=ADS_REG_MIN,
	ADS_CONFIG1,
	ADS_CONFIG2,
	ADS_CONFIG3,
	ADS_LOFF,
	ADS_CH1SET,
	ADS_CH2SET,
	ADS_CH3SET,
	ADS_CH4SET,
	ADS_CH5SET,
	ADS_CH6SET,
	ADS_CH7SET,
	ADS_CH8SET,
	ADS_BIAS_SENSP,
	ADS_BIAS_SENSN,
	ADS_LOFF_SENSP,
	ADS_LOFF_SENSN,
	ADS_LOFF_FLIP,
	ADS_LOFF_STATP,
	ADS_LOFF_STATN,
	ADS_GPIO,
	ADS_MISC1,
	ADS_MISC2,
	ADS_CONFIG4,
	ADS_REG_MAX=ADS_CONFIG4,
}ads_regs_enum_t;


#endif /* INCLUDE_ADS1299LIB_REGS_H_ */
