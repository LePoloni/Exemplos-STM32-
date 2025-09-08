//Criada por: Leandro Poloni Dantas
//Data: 23/10/23
//Notas: 
//	Compatibilizada com o a placa Nucleo-F446RE rodando a 84 MHz e timer 2 a 84 MHz
//	Leds usados WS2812B.
//	Configurar o timer 2 canal 1 como PWM: clock source internal clock,
//	counter period 105-1, pino PA15 (CN7.17).
//	Adicionar DMA1 ao timer 2 canal 1: modo normal, increment address memory,
//	datawidth word.
//  Configurar o tipo de leds e a quantidade no arquivo NeoPixel_STM32.h.
//Alterações:
//  06/11/23 - Corrigido bug do led verde extra na tela. Foram acrecentados dois
//             dummy pixels para parar o PWM antes do atendimento à interrupção.
//	03/11/24 - Correção na intensidade de cor I na função neopixel_fill e acréscimo
//			   de novos comentários.

#ifndef _NeoPixel_STM32_
#define _NeoPixel_STM32_

#include "stm32f4xx_hal.h"
#include <stdint.h>

//Parâmetros dos leds =========================================================
//Tipo de led (escolher um dos dois)
#define NUM_BPP (3) // WS2812B
//#define NUM_BPP (4) // SK6812

//Número de leds (pixels) (definir o valor)
#define NUM_PIXELS (9)

//Múmero de bytes para armazenamento de cores
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)

//Largura do pulso (duty cycle para um período de 105)
#define PWM_HI (75)
#define PWM_LO (29)

//Ajuste dos arrays
#define G 	0
#define R 	1
#define B 	2
#define I 	3
#define SG 	8
#define SR 	16
#define SB 	0
#define SI 	24

//Tabela de cores no padrão RGB de 24 bits
//https://www.cin.ufpe.br/~js/tabeladecores.htm
#define	BLACK   0x000000
#define	BLUE    0x0000FF
#define	RED     0xFF0000
#define	GREEN   0x00FF00
#define CYAN    0x00FFFF
#define MAGENTA 0xFF00FF
#define YELLOW  0xFFFF00
#define WHITE   0xFFFFFF
#define ORANGE	0xFF7F00
#define BROWN		0xA62A2A
#define GRAY		0xC0C0C0

//Protótipos de funções
uint32_t neopixel_blanck_all(void);
uint32_t neopixel_fill(uint32_t cor_rgb, uint32_t primeiro, uint32_t quantidade);
HAL_StatusTypeDef neopixel_show(void);
uint8_t neopixel_free_to_send(void);
void neopixel_demo(void);

#endif
