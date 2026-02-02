#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "cmsis_os.h"
#include "cmsis_gcc.h"
#include "ads1299lib.h"
#include "ads1299lib_gpio.h"
#include "semphr.h"

/********************************************************************************/
/*          ARCHIVO DE CÓDIGO ESPECÍFICO PARA ARQUITECTURAS STM32               */
/********************************************************************************/
/*  DEPENDENCIAS:  # LIBRERÍAS LOW-LEVEL PARA MANEJO DEL SPI y GPIO             */
/*                 # CMSIS-RTOS API V2 (Y FREERTOS SUBYACENTE)                  */
/*                                                                              */
/********************************************************************************/
/*  DESCRIPCIÓN: este módulo contiene las definiciones de las funciones para el */
/*               manejo de la comunicación SPI y los GPIO de control especí-    */
/*               ficas para la arquitectura STM32 y las librerías LL y CMSIS-   */
/*				 RTOS.															*/
/*																				*/
/* 				DEBE SELECCIONARSE LA INSTANCIA DE PUERTO SPI                   */
#define ADS_SPI SPI2
/* 				DEBE SELECCIONARSE LA INSTANCIA DE CONTORLADOR DMA              */
#define ADS_DMA DMA1
/* 				DEBEN SELECCIONARSE LAS INSTANCIAS DE STREAMS DMA               */
#define ADS_STREAM_RX LL_DMA_STREAM_3
#define ADS_STREAM_TX LL_DMA_STREAM_4




/* RUTINAS ESPEC�FICAS PARA ACCESO AL SPI1 */

#define WAIT(spi) while(LL_SPI_IsActiveFlag_BSY(spi) || !LL_SPI_IsActiveFlag_TXE(spi)){;}

//osMutexId_t ADS_spi_mutex = NULL;
uint32_t dummy =0;

/*
const osMutexAttr_t Thread_Mutex_attr = {
		"ADS_SPI_Mutex",                          // human readable mutex name
		NULL,                                     // memory for control block
		0U                                        // size for control block
};*/

SemaphoreHandle_t ADS_spi_mutex=NULL;

void ADS_spi_init(){
	/*if(ADS_spi_mutex == NULL)
		ADS_spi_mutex = osMutexNew(&Thread_Mutex_attr);*/
	if(ADS_spi_mutex == NULL)
			ADS_spi_mutex = xSemaphoreCreateBinary();
	xSemaphoreGive( ADS_spi_mutex );
	LL_SPI_Enable(ADS_SPI);
	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_RX);//RX
	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_TX);//TX
	LL_DMA_SetMemoryIncMode(ADS_DMA, ADS_STREAM_TX, LL_DMA_MEMORY_NOINCREMENT);
}


void ADS_spi_rx(uint8_t * rBuf, int n){
	int j;
	volatile int k;
	//osMutexAcquire(ADS_spi_mutex,osWaitForever);
	xSemaphoreTake( ADS_spi_mutex,
	                 portMAX_DELAY);
	LL_GPIO_ResetOutputPin(CS_GPIO_Port,CS_Pin);
	//LL_SPI_Enable(ADS_SPI);
	for(j=0;j<n;j++){

		WAIT(ADS_SPI);
		LL_SPI_TransmitData8(ADS_SPI,0x00);
		WAIT(ADS_SPI);

		*(rBuf+j) = LL_SPI_ReceiveData8(ADS_SPI);
	}
	//seg�n hoja de datos hay que dejar pasar al menos 4 Tclk para subir CS
	for(k=100;k>0;k--)__NOP(); //aprox. 7us
	//LL_SPI_Disable(ADS_SPI);
	LL_GPIO_SetOutputPin(CS_GPIO_Port,CS_Pin);
	//osMutexRelease(ADS_spi_mutex);
	xSemaphoreGive( ADS_spi_mutex );

}


void ADS_spi_tx(uint8_t * tBuf, int n){
	//volatile int k;
	int j;
	//osMutexAcquire(ADS_spi_mutex,osWaitForever);
	xSemaphoreTake( ADS_spi_mutex,
		                 portMAX_DELAY);
	LL_GPIO_ResetOutputPin(CS_GPIO_Port,CS_Pin);
	//LL_SPI_Enable(ADS_SPI);
	for(j=0;j<n;j++){
		WAIT(ADS_SPI);
		LL_SPI_TransmitData8(ADS_SPI,*(tBuf+j));
		WAIT(ADS_SPI);

		LL_SPI_ReceiveData8(ADS_SPI);
		ADS_Delay_ms(1);
		//for(k=150;k>0;k--)__NOP();//aprox. 7us
		//esto es para que pueda leer los multybyte commands [9.5.3.1 Sending Multi-Byte Commands]
	}
	//LL_SPI_Disable(ADS_SPI);
	LL_GPIO_SetOutputPin(CS_GPIO_Port,CS_Pin);
	//osMutexRelease(ADS_spi_mutex);
	xSemaphoreGive( ADS_spi_mutex );

}

void ADS_spi_rx_DMA_start(volatile uint8_t * tBuf, int n){





	//osStatus_t stat = osMutexAcquire(ADS_spi_mutex,0);
	BaseType_t stat =  xSemaphoreTake( ADS_spi_mutex,portMAX_DELAY );
	//if(stat == osOK){
	if(stat == pdPASS){
		LL_GPIO_ResetOutputPin(CS_GPIO_Port,CS_Pin);
		//LL_SPI_Enable(ADS_SPI);

		// Custom configuration of DMA (after calling function MX_SPI1_INIT()
		// Configure address of the buffer for receiving data
		LL_DMA_ConfigAddresses(ADS_DMA, ADS_STREAM_RX, LL_SPI_DMA_GetRegAddr(ADS_SPI), (uint32_t)tBuf,LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
		LL_DMA_ConfigAddresses(ADS_DMA, ADS_STREAM_TX,  (uint32_t)&dummy, LL_SPI_DMA_GetRegAddr(ADS_SPI),LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		// Configure data length
		LL_DMA_SetDataLength(ADS_DMA, ADS_STREAM_RX,n);
		LL_DMA_SetDataLength(ADS_DMA, ADS_STREAM_TX,n);
		// Enable DMA Transfer complete interrupt
		LL_DMA_EnableIT_TC(ADS_DMA, ADS_STREAM_RX);

		LL_SPI_EnableDMAReq_RX(ADS_SPI);
		LL_SPI_EnableDMAReq_TX(ADS_SPI);

		LL_DMA_ClearFlag_TC3(ADS_DMA);//esto quedó poco generico (depende del stream 3)
		LL_DMA_ClearFlag_TC4(ADS_DMA);//esto quedó poco generico (depende del stream 4)

		LL_DMA_EnableStream(ADS_DMA, ADS_STREAM_RX);//RX
		LL_DMA_EnableStream(ADS_DMA, ADS_STREAM_TX);//TX
	}

}

void ADS_spi_rx_DMA_start_FromISR(volatile uint8_t * tBuf, int n/*,BaseType_t *pxHigherPriorityTaskWoken*/){

	//BaseType_t stat =  xSemaphoreTakeFromISR(ADS_spi_mutex,pxHigherPriorityTaskWoken);

	//if(stat == pdPASS)
	{
		LL_GPIO_ResetOutputPin(CS_GPIO_Port,CS_Pin);
		//LL_SPI_Enable(ADS_SPI);

		// Custom configuration of DMA (after calling function MX_SPI1_INIT()
		// Configure address of the buffer for receiving data
		LL_DMA_ConfigAddresses(ADS_DMA, ADS_STREAM_RX, LL_SPI_DMA_GetRegAddr(ADS_SPI), (uint32_t)tBuf,LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
		LL_DMA_ConfigAddresses(ADS_DMA, ADS_STREAM_TX,  (uint32_t)&dummy, LL_SPI_DMA_GetRegAddr(ADS_SPI),LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		// Configure data length
		LL_DMA_SetDataLength(ADS_DMA, ADS_STREAM_RX,n);
		LL_DMA_SetDataLength(ADS_DMA, ADS_STREAM_TX,n);
		// Enable DMA Transfer complete interrupt
		LL_DMA_EnableIT_TC(ADS_DMA, ADS_STREAM_RX);

		LL_SPI_EnableDMAReq_RX(ADS_SPI);
		LL_SPI_EnableDMAReq_TX(ADS_SPI);

		LL_DMA_ClearFlag_TC3(ADS_DMA);//esto quedó poco generico (depende del stream 3)
		LL_DMA_ClearFlag_TC4(ADS_DMA);//esto quedó poco generico (depende del stream 4)

		LL_DMA_EnableStream(ADS_DMA, ADS_STREAM_RX);//RX
		LL_DMA_EnableStream(ADS_DMA, ADS_STREAM_TX);//TX
	}

}

void ADS_spi_rx_DMA_stop(){	

	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_RX);//RX
	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_TX);//TX

	LL_DMA_ClearFlag_TC3(ADS_DMA);//esto quedó poco generico (depende del stream 3)
	LL_DMA_ClearFlag_TC4(ADS_DMA);//esto quedó poco generico (depende del stream 4)
	//LL_SPI_DisableDMAReq_RX(ADS_SPI);
	//LL_SPI_DisableDMAReq_TX(ADS_SPI);

	//LL_SPI_Disable(ADS_SPI);

	LL_GPIO_SetOutputPin(CS_GPIO_Port,CS_Pin);

	//osMutexRelease(ADS_spi_mutex);
	xSemaphoreGive( ADS_spi_mutex );

}

void ADS_spi_rx_DMA_stop_FromISR(/*BaseType_t *pxHigherPriorityTaskWoken*/){

	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_RX);//RX
	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_TX);//TX

	LL_DMA_ClearFlag_TC3(ADS_DMA);//esto quedó poco generico (depende del stream 3)
	LL_DMA_ClearFlag_TC4(ADS_DMA);//esto quedó poco generico (depende del stream 4)
	LL_DMA_DisableIT_TC(ADS_DMA, ADS_STREAM_RX);
	//LL_SPI_DisableDMAReq_RX(ADS_SPI);
	//LL_SPI_DisableDMAReq_TX(ADS_SPI);

	//LL_SPI_Disable(ADS_SPI);

	LL_GPIO_SetOutputPin(CS_GPIO_Port,CS_Pin);


	//xSemaphoreGiveFromISR( ADS_spi_mutex, pxHigherPriorityTaskWoken);

}

void ADS_spi_rx_DMA_stop_and_prepare(volatile uint8_t * tBuf, int n)
{
	LL_GPIO_SetOutputPin(CS_GPIO_Port,CS_Pin);

	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_RX);//RX
	LL_DMA_DisableStream(ADS_DMA, ADS_STREAM_TX);//TX


	LL_DMA_ConfigAddresses(ADS_DMA, ADS_STREAM_RX, LL_SPI_DMA_GetRegAddr(ADS_SPI), (uint32_t)tBuf,LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_ConfigAddresses(ADS_DMA, ADS_STREAM_TX,  (uint32_t)&dummy, LL_SPI_DMA_GetRegAddr(ADS_SPI),LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	// Configure data length
	LL_DMA_SetDataLength(ADS_DMA, ADS_STREAM_RX,n);
	LL_DMA_SetDataLength(ADS_DMA, ADS_STREAM_TX,n);


	LL_SPI_EnableDMAReq_RX(ADS_SPI);
	LL_SPI_EnableDMAReq_TX(ADS_SPI);

	LL_DMA_ClearFlag_TC3(ADS_DMA);//esto quedó poco generico (depende del stream 3)
	LL_DMA_ClearFlag_TC4(ADS_DMA);//esto quedó poco generico (depende del stream 4)
	// Enable DMA Transfer complete interrupt
	LL_DMA_EnableIT_TC(ADS_DMA, ADS_STREAM_RX);

	//xSemaphoreGiveFromISR( ADS_spi_mutex, pxHigherPriorityTaskWoken);

}

void ADS_spi_rx_DMA_start_fast(void)
{
	LL_GPIO_ResetOutputPin(CS_GPIO_Port,CS_Pin);


	LL_DMA_EnableStream(ADS_DMA, ADS_STREAM_RX);//RX
	LL_DMA_EnableStream(ADS_DMA, ADS_STREAM_TX);//TX

}


void ADS_Delay_ms(int t_ms){
	HAL_Delay(t_ms);
}

void ADS_PWRRST_LOW(void){
	LL_GPIO_ResetOutputPin(PRST_GPIO_Port,PRST_Pin);
	//HAL_GPIO_WritePin( PRST_GPIO_Port, PRST_Pin, GPIO_PIN_RESET );
}
void ADS_PWRRST_HIGH(void){
	LL_GPIO_SetOutputPin(PRST_GPIO_Port,PRST_Pin);
	//HAL_GPIO_WritePin( PRST_GPIO_Port, PRST_Pin, GPIO_PIN_SET );
}

void ADS_DRDY_IT_DISABLE(){
	LL_EXTI_DisableIT_0_31(DRDY_EXTI_NUM);
	NVIC_DisableIRQ(DRDY_EXTI_IRQn);
}
void ADS_DRDY_IT_ENABLE(){
	LL_EXTI_ClearFlag_0_31(DRDY_EXTI_NUM);
	LL_EXTI_EnableIT_0_31(DRDY_EXTI_NUM);
	NVIC_EnableIRQ(DRDY_EXTI_IRQn);

}
