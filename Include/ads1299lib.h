#ifndef _ADS1299LIB_H
#define _ADS1299LIB_H

/*****************************************************
*				LIBRERIA ADS1299 V1  19/07/2019
*				
* Librería para el uso del ADS1299 desde cualquier uC.  
*	1. Programar las funciones de ads1299lib_spi.h
*				- Transmisión y recepción SPI
*				- Control de pin de RESET/PWDN
*				- Delay
*	2. Llamar a ADS_init()
* 3. Se puede consultar el estado de la librería con
*		 ADS_leer_estado_lib()
* 4. Utilizar las funciones de programación y control 
*    de adquisición
*****************************************************/

#include "ads1299lib_spi.h"  




/* 4 Canales */
#define ADS_N_CH 4

#define ADS_BYTES_PER_SAMPLE ADS_N_CH*3+3

#define ADS_CLCK_FREQ 2048000

#define ADS_REF 4.5f

/*******************************
* DEFINICIONES DE TIPOS PARA   *
* REGISTROS DEL ADS1299        *
*******************************/

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

typedef struct{
	uint8_t DR:3;
	uint8_t RESERVED_34:2;	
	uint8_t CLK_EN:1;
	uint8_t DAISY_EN:1;
	uint8_t RESERVED_7:1;
}ADS_CONFIG1_bits_t;

typedef union{
	ADS_CONFIG1_bits_t bits;
	uint8_t byte;
}ADS_CONFIG1_reg_t;

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


#define ADS_WRITE_CONFIG1_RESERVED_34	2
#define ADS_WRITE_CONFIG1_RESERVED_7 	1

#define ADS_CLCK_OUTPUT_ENABLED				1
#define ADS_CLCK_OUTPUT_DISABLED			0

#define ADS_DAISY_MODE_ENABLED				0
#define ADS_DAISY_MODE_DISABLED				1

#define ADS_WRITE_CONFIG1_RESERVED_7 	1


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

struct ads1299regs_t{
		unsigned char CONFIG1;
		unsigned char CONFIG2;
		unsigned char CONFIG3;
		unsigned char CHxSET[ADS_N_CH];
		unsigned char CONFIG4;
};



/*******************************************
* COPIA EN MEOMMORIA D EREGISTROS INTERNOS *
*******************************************/
extern struct ads1299regs_t ads1299regs;


/*******************************
* ENUMS RESULTADOS             *
*******************************/

enum resultado {R_OK, R_FAIL};
enum estado_lib_ads_e {sin_iniciar, iniciado_default, conf_programada, adquiriendo, detenido, adquiriendo_test, falla};

/*******************************
* INICIALIZACION DE LIBRERIA   *
*******************************/
/* Invocar siempre primero */
enum resultado ADS_init(void);
/* Chequeo de estados*/
enum estado_lib_ads_e ADS_leer_estado_lib(void);

/*******************************
* CONFIGURACION DE PARAMETROS  *
*******************************/

enum tasa_e {					ADS_DR_MIN = 0,
											ADS_DR_16KSPS=0,
											ADS_DR_8KSPS=1,
											ADS_DR_4KSPS=2,
											ADS_DR_2KSPS=3,
											ADS_DR_1KSPS=4,
											ADS_DR_500SPS=5,
											ADS_DR_250SPS=6,
											ADS_DR_MAX=6};
enum resultado ADS_set_data_rate(enum tasa_e);

enum ganancia_e {			ADS_GAIN_MIN=0,
											ADS_GAIN_1=0,
											ADS_GAIN_2=1,
											ADS_GAIN_4=2,
											ADS_GAIN_6=3,
											ADS_GAIN_8=4,
											ADS_GAIN_12=5,
											ADS_GAIN_24=6,
											ADS_GAIN_MAX=6};

enum resultado ADS_set_ch_gain(enum ganancia_e *);

enum canalactivo_e {	canalactivo_SI=0, 
											canalactivo_NO=1};
enum resultado ADS_set_ch_enabled(enum canalactivo_e*);

enum canalmodo_e {		ADS_CHMOD_MIN = 0,
											ADS_CHMOD_NORMAL=0, 
											ADS_CHMOD_CORTO=1, 
											ADS_CHMOD_BIAS=2,
											ADS_CHMOD_VCC=3,
											ADS_CHMOD_TEMP=4,
											ADS_CHMOD_TEST=5,
											ADS_CHMOD_BIASP=6,
											ADS_CHMOD_BIASN=7,
											ADS_CHMOD_MAX = 7};
enum resultado ADS_set_ch_mode(enum canalmodo_e*);
										
										
/*******************************
* CONTROL DE ADQUISICION       *
*******************************/
void ADS_iniciar_adquisicion(void);
void ADS_detener_adquisicion(void);
										
/*******************************
* AUXILIARES                   *
*******************************/
enum resultado ADS_visualizar_registros(uint8_t * arr);
enum resultado ADS_get_ID(uint8_t * res);
void ADS_RESET(void);
enum tasa_e ADS_get_data_rate(void);
void ADS_get_ch_mode(enum canalmodo_e*);
void ADS_get_ch_gain(enum ganancia_e *);

#endif
