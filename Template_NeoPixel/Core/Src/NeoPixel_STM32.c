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

#include <NeoPixel_STM32.h>

//Periféricos usados
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch1;

//Buffer de escrita com dutycycle de cada bit de cor de cada led
uint32_t wr_dc_buf[(NUM_PIXELS+2)*NUM_BPP*8];
//Sinalização de fim de transmissão do DMA
uint8_t datasentflag = 1;

//Apaga todos os leds
uint32_t neopixel_blanck_all(void)
{
	return neopixel_fill(0, 0, NUM_PIXELS);
}

//Define a cor de 1 ou mais leds
uint32_t neopixel_fill(uint32_t cor_rgb, uint32_t primeiro, uint32_t quantidade)
{
	//Array de cores
#if (NUM_BPP == 4) // SK6812
	uint8_t grb_arr[4];
#else // WS2812B
	uint8_t grb_arr[3];
#endif

	//Último pixel
	uint32_t last_pixel = NUM_PIXELS - 1;

	//Separa as cores
	grb_arr[R] = (cor_rgb >> SR) & 0xFF;
	grb_arr[G] = (cor_rgb >> SG) & 0xFF;
	grb_arr[B] = (cor_rgb >> SB) & 0xFF;
#if (NUM_BPP == 4) // SK6812
	grb_arr[I] = (cor_rgb >> SI) & 0xFF;
#endif

	//Testa os limites
	if(primeiro > last_pixel)
	{
		return 1;
	}
	else if(quantidade < 1)
	{
		return 2;
	}
	else if((primeiro + quantidade) > NUM_PIXELS)
	{
		last_pixel = NUM_PIXELS - 1;
	}
	else
	{
		last_pixel = primeiro + quantidade - 1;
	}

	//Transforma cada cor de cada pixel em um array de larguras do duty cycle
	for(int i = primeiro; i <= last_pixel; i++)
	{
		for(uint_fast8_t j = 0; j < 8; ++j)
		{
			//Grava um byte de duty cycle para cada bit, começa pelo MSb e termina no LSb
			//Bit 1: grava PWM_LO * 2, Bit 0: grava PWM_LO
			#if (NUM_BPP == 4) // SK6812
			wr_dc_buf[i*32 + j + G*8] = PWM_LO << (((grb_arr[0] << j) & 0x80) > 0); //Pixel i - G
			wr_dc_buf[i*32 + j + R*8] = PWM_LO << (((grb_arr[1] << j) & 0x80) > 0);	//Pixel i - R
			wr_dc_buf[i*32 + j + B*8] = PWM_LO << (((grb_arr[2] << j) & 0x80) > 0);	//Pixel i - B
			wr_dc_buf[i*32 + j + I*8] = PWM_LO << (((grb_arr[3] << j) & 0x80) > 0); //Pixel i - I
			#else // WS2812B
			wr_dc_buf[i*24 + j + G*8] = PWM_LO << (((grb_arr[0] << j) & 0x80) > 0); //Pixel i - G
			wr_dc_buf[i*24 + j + R*8] = PWM_LO << (((grb_arr[1] << j) & 0x80) > 0);	//Pixel i - R
			wr_dc_buf[i*24 + j + B*8] = PWM_LO << (((grb_arr[2] << j) & 0x80) > 0);	//Pixel i - B
			#endif
		}
	}
	return 0;
}

//Atualiza o estado dos leds
HAL_StatusTypeDef neopixel_show(void)
{
	HAL_StatusTypeDef status;

	//Insere dois dummy pixels para parar o PWM antes da interrupção.
	//Isso evita que o último pixel se repita algumas vezes antes da pausa do PWM
	for(int i = (NUM_PIXELS)*NUM_BPP*8; i < (NUM_PIXELS+2)*NUM_BPP*8; i++)
	{
		wr_dc_buf[i] = 0;
	}
	//Inicia o PWM no pino PA15 (CN7.17)
	status = HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)wr_dc_buf, (NUM_PIXELS+2)*NUM_BPP*8);
	if(status == HAL_OK)
	{
		datasentflag = 0;
	}

	return status;
}

//Retorna se o DMA está livre para enviar novas cores aos leds
uint8_t neopixel_free_to_send(void)
{
	return datasentflag;
}

//Função de demonstração e teste
void neopixel_demo(void)
{
	while(1)
	{
		for(int i = 0; i < NUM_PIXELS; i++)
		{
			if(((i+1)%3) == 1)
			{
				neopixel_fill(RED, i, 1);
			}
			else if((((i+1)%3) == 2))
			{
				neopixel_fill(GREEN, i, 1);
			}
			else
			{
				neopixel_fill(BLUE, i, 1);
			}
		}
		while(!neopixel_free_to_send());
		neopixel_show();
		HAL_Delay(300);

		neopixel_blanck_all();
		while(!neopixel_free_to_send());
		neopixel_show();
		HAL_Delay(300);
	}
}

//Função de callbak de metade do buffer transmitida
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	//Disponível para uso futuro
}

//Função de callbak de conclusão da transmissão do buffer
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//Para o DMA
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	//Sinaliza a conclusão do envio
	datasentflag = 1;
}
