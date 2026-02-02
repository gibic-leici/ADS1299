/*
 * ads1299lib_gpio.h
 *
 *  Created on: Oct 30, 2021
 *      Author: Lenovo
 */

#ifndef INC_ADS1299_4_ADS1299LIB_GPIO_H_
#define INC_ADS1299_4_ADS1299LIB_GPIO_H_




/****************************************************************************
 *  DARTA READY
 ****************************************************************************/


/* 				DEBE SELECCIONARSE LA INSTANCIA DE EXT IRQ                      */
#define DRDY_EXTI_IRQn EXTI4_IRQn //EXTI15_10_IRQn
#define DRDY_EXTI_NUM LL_EXTI_LINE_4//LL_EXTI_LINE_15
/* 				TANTO EL PUERTO COMO SUS STREAMS DMA DEBEN ESTAR CONFIGURADOS   */
/*				PREVIAMENTE MEDIANTE LA HERRAMIENTA DE CUBE                     */

/****************************************************************************
 *  CHIP SELECT
 ****************************************************************************/
#define CS_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_5


/****************************************************************************
 *  POWER DOWN/RESET
 ****************************************************************************/
#define PRST_GPIO_Port  GPIOB
#define PRST_Pin GPIO_PIN_0



#endif /* INC_ADS1299_4_ADS1299LIB_GPIO_H_ */
