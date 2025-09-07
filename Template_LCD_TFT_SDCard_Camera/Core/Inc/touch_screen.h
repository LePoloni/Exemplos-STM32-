/*
 * @file touch_screen.h
 * @date 16 de nov de 2022
 * @author Leandro Poloni Dantas
 */

#ifndef INC_TOUCH_SCREEN_H_
#define INC_TOUCH_SCREEN_H_

//Arquivos
#include "stm32f4xx_hal.h"

//Definições e constantes
//LCD TFT paralelo 8 bits
#define RS_PORT GPIOA						//RS PORT
#define RS_PIN  GPIO_PIN_4			//RS PIN (Arduino A2)
#define CS_PORT GPIOB          	//CS PORT
#define CS_PIN  GPIO_PIN_0     	//CS PIN	(Arduino A3)

#define D1_PORT GPIOC						//D1 PORT
#define D1_PIN 	GPIO_PIN_7			//D1 PIN	(Arduino D9)
#define D0_PORT GPIOA						//D0 PORT
#define D0_PIN 	GPIO_PIN_9			//D0 PIN	(Arduino D8)

//Touch screen 4 fios (medidas: Rx = 630 ohms, Ry = 373 ohms)
#define YP_PORT GPIOA						//RS PORT
#define YP_PIN  GPIO_PIN_4			//RS PIN (Arduino A2)
#define XM_PORT GPIOB         	//CS PORT
#define XM_PIN  GPIO_PIN_0    	//CS PIN (Arduino A3)

#define XP_PORT GPIOC						//D1 PORT
#define XP_PIN 	GPIO_PIN_7			//D1 PIN (Arduino D9)
#define YM_PORT GPIOA						//D0 PORT
#define YM_PIN 	GPIO_PIN_9			//D0 PIN (Arduino D8)

#define YP_CHANNEL	ADC_CHANNEL_4
#define XM_CHANNEL	ADC_CHANNEL_8

#define RXPLATE		600		//Colocar aqui o valor aproximada da resistência Rx
#define AMOSTRAS 	2

#define MODO_NORMAL 		0
#define MODO_INVERTIDO 	1

#define TS_SUCCESS			0
#define TS_FAIL					-1

#ifndef WIDTH
	#define WIDTH 	320		//Largura da tela
#endif
#ifndef HEIGHT
	#define HEIGHT 	240		//Altura da tela
#endif

#define DIF_MAX 	16		//Diferença máxima entre amostras sucessivas para validação

//Protótipos de funções
void ts_tft_gpio_init(void);
void ts_gpio_init(void);
void ts_PIN_LOW (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void ts_PIN_HIGH (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void ts_PIN_INPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void ts_PIN_OUTPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void ts_PIN_ANALOG (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t channel);
void ts_read_point (uint32_t* x, uint32_t* y, uint32_t* z);
void ts_read_point (uint32_t* x, uint32_t* y, uint32_t* z);
uint32_t ts_read_x();
uint32_t ts_read_y();
uint32_t ts_read_z();
uint32_t ts_top(uint32_t modo);
uint32_t ts_bottom(uint32_t modo);
uint32_t ts_left(uint32_t modo);
uint32_t ts_right(uint32_t modo);
int32_t ts_ajusta_limites(uint32_t left, uint32_t right, uint32_t top, uint32_t bottom);
int32_t ts_read_pixel(uint32_t* px, uint32_t* py, uint32_t* pz);

#endif /* INC_TOUCH_SCREEN_H_ */
