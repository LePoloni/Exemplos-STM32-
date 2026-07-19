/**
 ******************************************************************************
 * @file        touch_spi.h
 * @brief       Biblioteca controle do touch screen de LCD TFT SPI via
 * 				kit ST NUCLEO-F446RE
 ******************************************************************************
 * @attention
 *
 * <h2><center>Desenvolvida para fins acadêmicos</center></h2>
 *
 * @author      Leandro Poloni Dantas
 * @details     Compatível com controlador de touch XPT2046 presente em displays
 * 				do tipo TFT com driver ILI9341 e ST7789
 * @version     1.0
 * @date        Julho de 2026
 * @details     Comentada de acordo com o padrão Doxygen.
 *
 * ***************************************************************************
 * @details	Notas:
 * 			É recomendado que o clock do touch seja menor do que 1 MHz,
 * 			por limitações do CAD do chip controlador do touch XPT2046.
 * 			Na configuração dos pinos para interface SPI, defina os pinos
 * 			para as funções abaixo com os nomes indicados:
 * 			Chip select: 	TOUCH_CS
 * 			IRQ: 			TOUCH_IRQ
 * 			A porta 2 é padrão para SPI. Caso deseje outra porta, crie uma
 * 			definição como indicado abaixo no seu arquivo main.h:
 * 			#define TOUCH_SPI_PORT hspi<número>
 * ***************************************************************************
 * @details	Modificações:
 * @date	_________
 * @details	_________
 *
 *****************************************************************************
 */

/* Define para prevenir inclusão recursiva ----------------------------------*/
#ifndef TOUCH_SPI_H_
#define TOUCH_SPI_H_

/* Includes -----------------------------------------------------------------*/
#include <stdbool.h>
//#include "stm32f1xx_hal.h"
#include "stm32f4xx_hal.h"
#include "main.h"

/* Constantes (redefina quando necessário -----------------------------------*/
//Atenção! Use o barramento SPI com velocidade < 1,3 Mbit, mais seguro ~650 kbit
#ifndef TOUCH_SPI_PORT
#define TOUCH_SPI_PORT hspi2
#endif
extern SPI_HandleTypeDef TOUCH_SPI_PORT;

#ifndef TOUCH_IRQ_Pin
//#define TOUCH_IRQ_Pin       GPIO_PIN_4 // Arduino D5 e CN10.27
//#define TOUCH_IRQ_GPIO_Port GPIOB
#define TOUCH_IRQ_Pin       GPIO_PIN_1 // Arduino A4 e CN7.36
#define TOUCH_IRQ_GPIO_Port GPIOC
#endif

#ifndef TOUCH_CS_Pin
//#define TOUCH_CS_Pin        GPIO_PIN_10 // Arduino D2 e CN10.33
//#define TOUCH_CS_GPIO_Port  GPIOA
#define TOUCH_CS_Pin        GPIO_PIN_0 // Arduino A3 e CN7.34
#define TOUCH_CS_GPIO_Port  GPIOB
#endif

//Mude de acordo com a orientação da tela
#define TOUCH_SCALE_X 240
#define TOUCH_SCALE_Y 320
 
//Para calibrar use as funções de calibração presentes em touch_spi.c
#define TOUCH_MIN_RAW_X 1853
#define TOUCH_MAX_RAW_X 30080
#define TOUCH_MIN_RAW_Y 1435
#define TOUCH_MAX_RAW_Y 31255

/* Protótipos de funções ---------------------------------------------------*/
//Chamar antes de inicializar qualquer outro dispositivo SPI
void touch_unselect();
 
bool touch_pressed();
bool touch_getCoordinates(uint16_t* x, uint16_t* y);
bool touch_getRawCoordinates(uint16_t* raw_x, uint16_t* raw_y);
void touch_getRawMinMaxCoordenadtes(uint16_t* raw_x_min, uint16_t* raw_x_max,
		uint16_t* raw_y_min, uint16_t* raw_y_max);

#endif /* TOUCH_SPI_H_ */
