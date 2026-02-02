#include "ads1299lib.h"

extern void ADS_spi_init(void);


enum resultado ADS_verificar_configuracion(void);

/*******************************
* Definiciones hoja de datos   *
*******************************/

/* Comandos */
// System
#define WAKEUP 0x02
#define STANDBY 0x04
#define RESET 0x06
#define START 0x08
#define STOP 0x0A
// Data read
#define RDATAC 0x10
#define SDATAC 0x11
#define RDATA 0x12
// Register read
#define RREG 0x20
#define WREG 0x40

/* Registros */
#define dirID 0x00
#define dirCONFIG1 0x01
#define dirCONFIG2 0x02
#define dirCONFIG3 0x03
#define dirCONFIG4 0x17
#define dirCHxSET 0x05


/*******************************
* Estado de la libreria        *
*******************************/
enum estado_lib_ads_e estado_lib_ads=sin_iniciar;

/* Chequeo de estados*/
enum estado_lib_ads_e ADS_leer_estado_lib(void)
{
	return estado_lib_ads;
}


/*******************************
* Config binaria               *
*******************************/


#define tamREGCONFIG 8



struct ads1299regs_t ads1299regs;

/*******************************
* Buffers de comunicacion      *
*******************************/

uint8_t rxBuf[100] = {0};
uint8_t txBuf[100] = {0};

/************************************
**** RUTINA INICIALIZACI�N ADS1299 **
************************************/

enum resultado ADS_init(){
				
			int i;
			
			ADS_CONFIG1_reg_t config1;
	
			ADS_DRDY_IT_DISABLE();
			ADS_spi_init();
	
			config1.bits.CLK_EN = ADS_CLCK_OUTPUT_ENABLED;
			config1.bits.DAISY_EN = ADS_DAISY_MODE_DISABLED;
			config1.bits.DR = ADS_DR_16KSPS;					
			config1.bits.RESERVED_34 = 2;
			config1.bits.RESERVED_7 = 1;
	
			ads1299regs.CONFIG1 = config1.byte;
			ads1299regs.CONFIG2 = 0xD0;//0x68; // bit 4 en 1 test signal generada internamente
			ads1299regs.CONFIG3 = 0xEC;
			ads1299regs.CONFIG4 = 0x00;

			// 0b00000000 Normal operation - 0b00000001 Input shorted - 0b00000101 Test square wave
			ads1299regs.CHxSET[0] = 0x00; 
			ads1299regs.CHxSET[1] = 0x00;
			ads1299regs.CHxSET[2] = 0x00;
			ads1299regs.CHxSET[3] = 0x00;
				
//			ADS_PWRRST_LOW();
//			ADS_Delay_ms(100);
//			ADS_PWRRST_HIGH();
//			ADS_Delay_ms(100);

//			txBuf[0] = SDATAC;
//			txBuf[1] = STOP;
//			ADS_spi_tx(txBuf,2);
//			
//			
//			txBuf[0] = WREG+dirCONFIG1;
//			txBuf[1] = 0x00;
//			txBuf[2] = 0xD5; //0b11010101; 
//			ADS_spi_tx(txBuf,3);
//			ADS_Delay_ms(100);
	
	
			ADS_DRDY_IT_DISABLE();
			
			ADS_PWRRST_LOW();
			ADS_Delay_ms(100);
			ADS_PWRRST_HIGH();
			ADS_Delay_ms(1000);
			
			ADS_RESET();			

			ADS_detener_adquisicion();
			
			txBuf[0] = WREG+dirCONFIG1;
			txBuf[1] = 0x00;
			txBuf[2] = ads1299regs.CONFIG1;
			ADS_spi_tx(txBuf,3);
			ADS_Delay_ms(100);
			
			txBuf[0] = WREG+dirCONFIG2;
			txBuf[1] = 0x00;
			txBuf[2] = ads1299regs.CONFIG2;
			ADS_spi_tx(txBuf,3);
			
			txBuf[0] = WREG+dirCONFIG3;
			txBuf[1] = 0x00;
			txBuf[2] = ads1299regs.CONFIG3;
			ADS_spi_tx(txBuf,3);
			
			txBuf[0] = WREG+dirCHxSET;
			txBuf[1] = (char)(ADS_N_CH-1); 
			for(i=0;i<ADS_N_CH;i++){
        txBuf[i+2] = ads1299regs.CHxSET[i];    
			}
			ADS_spi_tx(txBuf,ADS_N_CH+2);
			ADS_Delay_ms(100);
			
			txBuf[0] = WREG+dirCONFIG4;
			txBuf[1] = 0x00;
			txBuf[2] = ads1299regs.CONFIG4;
			ADS_spi_tx(txBuf,3);
			ADS_Delay_ms(100);
			
			if(ADS_verificar_configuracion() == R_OK){
				estado_lib_ads = iniciado_default;
			}
			else{
				estado_lib_ads = falla;
				return R_FAIL;
			}
			
			//para sacar porquería del ADS
			txBuf[0]=txBuf[1]=txBuf[2]=0;
			ADS_spi_tx(txBuf,3);
			ADS_Delay_ms(100);

			return R_OK;
}

/************************************
**** RUTINA PROGRAMACION ADS1299   **
************************************/
enum resultado ADS_programar_configuracion(void){
			
			int i;
	
			if(estado_lib_ads==sin_iniciar || estado_lib_ads==falla){
				return R_FAIL;
			}
			
			txBuf[0] = SDATAC;
			txBuf[1] = STOP;
			ADS_spi_tx(txBuf,2);
			ADS_Delay_ms(100);
			
			txBuf[0] = WREG+dirCONFIG1;
			txBuf[1] = 0x00;
			txBuf[2] = ads1299regs.CONFIG1;
			ADS_spi_tx(txBuf,3);
			ADS_Delay_ms(100);
			
			txBuf[0] = WREG+dirCONFIG2;
			txBuf[1] = 0x01;
			txBuf[2] = ads1299regs.CONFIG2;
			txBuf[3] = ads1299regs.CONFIG3;
			ADS_spi_tx(txBuf,4);
			ADS_Delay_ms(100);
			
			txBuf[0] = WREG+dirCHxSET;
			txBuf[1] = (char)(ADS_N_CH-1); 
			for(i=0;i<ADS_N_CH;i++){
				txBuf[i+2] = ads1299regs.CHxSET[i];    
			}
			ADS_spi_tx(txBuf,ADS_N_CH+2);
			ADS_Delay_ms(100);
			
			txBuf[0] = WREG+dirCONFIG4;
			txBuf[1] = 0x00;
			txBuf[2] = ads1299regs.CONFIG4;
			ADS_spi_tx(txBuf,3);
			ADS_Delay_ms(100);
			
			if(ADS_verificar_configuracion() == R_OK){
				estado_lib_ads = conf_programada;
			}
			else{
				estado_lib_ads = falla;
				return R_FAIL;
			}
			
			return R_OK;
}

/*******************************************
**** RUTINA VERIFICACION CONF ADS1299 ******
*******************************************/

enum resultado ADS_verificar_configuracion(void){

		int i;

		enum resultado error = R_OK;
		
	  txBuf[0] = RREG+dirCONFIG1;
		txBuf[1] = 0x00; 
		ADS_spi_tx(txBuf,2);	
		ADS_Delay_ms(10);
	  rxBuf[0]=0x00;
		ADS_spi_rx(rxBuf,1);	
		if(	ads1299regs.CONFIG1 != rxBuf[0] ){
			error = R_FAIL;
		}
		
		txBuf[0] = RREG+dirCONFIG2;
		txBuf[1] = 0x00; 
		ADS_spi_tx(txBuf,2);	
		ADS_Delay_ms(10);
	  rxBuf[0]=0x00;
		ADS_spi_rx(rxBuf,1);	
		if(	ads1299regs.CONFIG2 != rxBuf[0] ){
			error = R_FAIL;
		}
		
		txBuf[0] = RREG+dirCONFIG3;
		txBuf[1] = 0x00; 
		ADS_spi_tx(txBuf,2);	
		ADS_Delay_ms(10);
	  rxBuf[0]=0x00;
		ADS_spi_rx(rxBuf,1);	
		if(	ads1299regs.CONFIG3 != rxBuf[0] ){
			error = R_FAIL;
		}

		for(i=0;i<ADS_N_CH;i++){
			txBuf[0] = RREG+dirCHxSET+i;
			txBuf[1] = 0;
			ADS_spi_tx(txBuf,2);
			ADS_Delay_ms(10);
			
			rxBuf[0]=0x00;
			ADS_spi_rx(rxBuf,1);		
		
			if( ads1299regs.CHxSET[i] != rxBuf[0]){
					error = R_FAIL;
			}
		}
		
		txBuf[0] = RREG+dirCONFIG4;
		txBuf[1] = 0x00; 
		ADS_spi_tx(txBuf,2);
		ADS_Delay_ms(10);
		
		rxBuf[0]=0x00;
		ADS_spi_rx(rxBuf,1);
		
		if(	ads1299regs.CONFIG4 != rxBuf[0] ){
			error = R_FAIL;
		}
		
		if(error==R_FAIL){
			estado_lib_ads = falla;
		}
		return error;

}

/*******************************************
**** RUTINAS DE CONTROL DE ADQUISICION   ***
*******************************************/

void ADS_iniciar_adquisicion(void){
		
		txBuf[0] = START;
		txBuf[1] = RDATAC;
		ADS_spi_tx(txBuf, 2);
		estado_lib_ads = adquiriendo;
		ADS_DRDY_IT_ENABLE();
		
}
void ADS_detener_adquisicion(void){
		
		ADS_DRDY_IT_DISABLE();
	  ADS_Delay_ms(10);
		txBuf[0] = SDATAC;
		txBuf[1] = STOP;
		ADS_spi_tx(txBuf,2);
	  ADS_Delay_ms(100);
		
		estado_lib_ads = detenido;
			
}

void ADS_RESET(void){
		
		//ADS_DRDY_IT_DISABLE();
		ADS_detener_adquisicion();
		txBuf[0] = RESET;
		ADS_spi_tx(txBuf,1);
		ADS_Delay_ms(100);
	
		estado_lib_ads = detenido;
			
}


/*******************************************
**** RUTINAS DE PROGRAMACION PARAMETROS  ***
*******************************************/

enum resultado ADS_set_data_rate(enum tasa_e tasa){
	
	enum resultado res=R_OK;
	ADS_CONFIG1_reg_t config1;
	
	if(ADS_DR_MIN > tasa || tasa > ADS_DR_MAX){		
		return  R_FAIL;
	}
	
	config1.byte = ads1299regs.CONFIG1;
	config1.bits.DR = tasa;		
	ads1299regs.CONFIG1 = config1.byte;
	
	ADS_detener_adquisicion();
				
	return ADS_programar_configuracion();

}

enum tasa_e ADS_get_data_rate(){

	ADS_CONFIG1_reg_t config1;
	rxBuf[0]=0x00;
		
	ADS_detener_adquisicion();
	
	txBuf[0] = RREG+dirCONFIG1;
	txBuf[1] = 0x00; 
	ADS_spi_tx(txBuf,2);		
	ADS_Delay_ms(10);	
	
	ADS_spi_rx(rxBuf,1);	
	config1.byte = rxBuf[0];
	
	return (config1.bits.DR);
}

enum resultado ADS_set_ch_gain(enum ganancia_e *ganancias){
	
	int i;
	ADS_CHnSET_reg_t CHnSET;
	
	for(i=0;i<ADS_N_CH;i++){
		if(ADS_GAIN_MIN > ganancias[i]  || ganancias[i] > ADS_GAIN_MAX){		
			return  R_FAIL;
		}
	}
	
	for(i=0;i<ADS_N_CH;i++){
		CHnSET.byte = ads1299regs.CHxSET[i];
		CHnSET.bits.GAIN = ganancias[i];
		ads1299regs.CHxSET[i] = CHnSET.byte;
	}
	
	ADS_detener_adquisicion();
	
	return ADS_programar_configuracion();
	
}

enum resultado ADS_set_ch_enabled(enum canalactivo_e* canalactivo){
	int i;
	uint8_t canalactivo_bytes;
	
	for(i=0;i<ADS_N_CH;i++){
		
		switch(*(canalactivo+i)){
				case canalactivo_SI:	canalactivo_bytes = 0x00; break;
				case canalactivo_NO:  canalactivo_bytes = 0x80; break;
				default: canalactivo_bytes = 0x00; break;
		}
	
		ads1299regs.CHxSET[i] = (ads1299regs.CHxSET[i] & 0x7F) + canalactivo_bytes;
	}
	
	ADS_programar_configuracion();
	
	if(ADS_verificar_configuracion() == R_OK){
		estado_lib_ads = conf_programada;
	}
	else{
		estado_lib_ads = falla;
		return R_FAIL;
	}
	
	return R_OK;
}

enum resultado ADS_set_ch_mode(enum canalmodo_e * canalmodo){
	
	int i;
		
	ADS_CHnSET_reg_t CHnSET;
	
	for(i=0;i<ADS_N_CH;i++){
		if(ADS_CHMOD_MIN > canalmodo[i]  || canalmodo[i] > ADS_CHMOD_MAX){		
			return  R_FAIL;
		}
	}
	
	for(i=0;i<ADS_N_CH;i++){
		CHnSET.byte = ads1299regs.CHxSET[i];
		CHnSET.bits.MUX = canalmodo[i];
		ads1299regs.CHxSET[i] = CHnSET.byte;
	}
	
	ADS_detener_adquisicion();
	
	return ADS_programar_configuracion();
}


enum resultado ADS_visualizar_registros(uint8_t * arr){
	
		int i;

		txBuf[0] = RREG+dirID;
		txBuf[1] = 0x03;
		ADS_spi_tx(txBuf,2);
		ADS_Delay_ms(10);
		rxBuf[0]=rxBuf[1]=rxBuf[2]=rxBuf[3]=0x00;
		ADS_spi_rx(rxBuf,4);

		*(arr+0) =  rxBuf[0];
		*(arr+1) =  rxBuf[1]; 
		*(arr+2) =  rxBuf[2];
		*(arr+3) =  rxBuf[3]; 	 	

		txBuf[0] = RREG+dirCHxSET;
    txBuf[1] = (char)(ADS_N_CH-1);
    ADS_spi_tx(txBuf,2);
		ADS_Delay_ms(10);
		rxBuf[0]=rxBuf[1]=rxBuf[2]=rxBuf[3]=0x00;
		ADS_spi_rx(rxBuf,ADS_N_CH);
		
		for(i=0;i<ADS_N_CH;i++){
			*(arr+4+i)=rxBuf[i];
		}
		
		txBuf[0] = RREG+dirCONFIG4;
		txBuf[1] = 0x00; 
		ADS_spi_tx(txBuf,2);
		ADS_Delay_ms(10);
		rxBuf[0]=0x00;
		ADS_spi_rx(rxBuf,1);
		
		*(arr+8)= rxBuf[0];
		
		
		estado_lib_ads = falla;
		if(ADS_verificar_configuracion() == R_OK){
			estado_lib_ads = conf_programada;
		}
		else{
			estado_lib_ads = falla;
			return R_FAIL;
		}

		return R_OK;
}	

enum resultado ADS_get_ID(uint8_t * res){
	
		ADS_detener_adquisicion();
		txBuf[0] = RREG+dirID;
		txBuf[1] = 0x00;
		ADS_spi_tx(txBuf,2);
		ADS_Delay_ms(10);
		rxBuf[0]=0x00;
		ADS_spi_rx(rxBuf,1);

		*res =  rxBuf[0];
		
		return R_OK;
}	

void ADS_get_ch_mode(enum canalmodo_e* modos)
{
	int i;
	ADS_CHnSET_reg_t chnset;
	
	ADS_detener_adquisicion();
	for(i=0;i<ADS_N_CH;i++){		
		txBuf[0] = RREG+dirCHxSET+i;
    txBuf[1] = 0;
    ADS_spi_tx(txBuf,2);		
		ADS_Delay_ms(10);
		
		rxBuf[0]=0;
		ADS_spi_rx(rxBuf,1);
		chnset.byte = rxBuf[0];
		modos[i] = chnset.bits.MUX;			
	}
}

void ADS_get_ch_gain(enum ganancia_e* ganancias)
{
	int i;
	ADS_CHnSET_reg_t chnset;
	
	ADS_detener_adquisicion();
	for(i=0;i<ADS_N_CH;i++){		
		txBuf[0] = RREG+dirCHxSET+i;
    txBuf[1] = 0;
    ADS_spi_tx(txBuf,2);		
		ADS_Delay_ms(10);
		
		rxBuf[0]=0;
		ADS_spi_rx(rxBuf,1);
		chnset.byte = rxBuf[0];
		ganancias[i] = chnset.bits.GAIN;			
	}
}


