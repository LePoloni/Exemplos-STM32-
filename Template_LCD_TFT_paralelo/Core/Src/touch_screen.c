/*
 * @file touch_screen.c
 * @date 16 de nov de 2022
 * @author Leandro Poloni Dantas
 */

//Arquivos
#include "touch_screen.h"

//Variáveis globais
enum posicao {TOP, BOTTOM, LEFT, RIGHT};
ADC_HandleTypeDef hadc1_t;
uint32_t edge[4] = {128,128,128,128};

//Funções
/*
 * @brief Inicialização de GPIOs para LCD TFT
 */
void ts_tft_gpio_init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	ts_PIN_OUTPUT(RS_PORT, RS_PIN);
	ts_PIN_OUTPUT(CS_PORT, CS_PIN);

	ts_PIN_OUTPUT(D1_PORT, D1_PIN);
	ts_PIN_OUTPUT(D0_PORT, D0_PIN);

	//Define valores das saídas
	ts_PIN_HIGH(RS_PORT, RS_PIN);
	ts_PIN_HIGH(CS_PORT, CS_PIN);
	ts_PIN_LOW(D1_PORT, D1_PIN);
	ts_PIN_LOW(D0_PORT, D0_PIN);
}

/*
 * @brief Inicialização de GPIOs para touch screen
 */
void ts_gpio_init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();

	// Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	hadc1_t.Instance = ADC1;
	hadc1_t.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1_t.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1_t.Init.ScanConvMode = DISABLE;
	hadc1_t.Init.ContinuousConvMode = DISABLE;
	hadc1_t.Init.DiscontinuousConvMode = DISABLE;
	hadc1_t.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1_t.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1_t.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1_t.Init.NbrOfConversion = 1;
	hadc1_t.Init.DMAContinuousRequests = DISABLE;
	hadc1_t.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1_t) != HAL_OK)
	{
		//TODO: Definir tratamento para erro
	}
	//Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	sConfig.Channel = YP_CHANNEL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1_t, &sConfig) != HAL_OK)
	{
		//TODO: Definir tratamento para erro
	}

	ts_PIN_ANALOG(YP_PORT, YP_PIN, YP_CHANNEL);
	ts_PIN_INPUT(YM_PORT, YM_PIN);

	ts_PIN_OUTPUT(XP_PORT, XP_PIN);
	ts_PIN_OUTPUT(XM_PORT, XM_PIN);
	ts_PIN_HIGH(XP_PORT, XP_PIN);
	ts_PIN_LOW(XM_PORT, XM_PIN);
}

/*
 * @brief Ajusta GPIO para nível baixo
 * @paran [in] *GPIOx Port
 * @paran [in] GPIO_Pin Pino
 */
void ts_PIN_LOW(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

/*
 * @brief Ajusta GPIO para nível alto
 * @paran [in] *GPIOx Port
 * @paran [in] GPIO_Pin Pino
 */
void ts_PIN_HIGH(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/*
 * @brief Ajusta sentido do GPIO como entrada digital
 * @paran [in] *GPIOx Port
 * @paran [in] GPIO_Pin Pino
 */
void ts_PIN_INPUT(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*
 * @brief Ajusta sentido do GPIO como saída digital
 * @paran [in] *GPIOx Port
 * @paran [in] GPIO_Pin Pino
 */
void ts_PIN_OUTPUT(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	 GPIO_InitStruct.Pin = GPIO_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	 HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*
 * @brief Ajusta sentido do GPIO como entrada analógica
 * @paran [in] *GPIOx Port
 * @paran [in] GPIO_Pin Pino
 * @param [in] channel Canal do CAD
 */
void ts_PIN_ANALOG(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t channel)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

	//Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1_t, &sConfig) != HAL_OK)
	{
		//TODO: Definir tratamento para erro
	}
}

/*
 * @brief Lê um ponto na tela
 * @param [out] *x Posição no eixo x (saída do CAD)
 * @param [out] *y Posição no eixo y (saída do CAD)
 * @param [out] *z Pressão no toque (valor normalizado)
 */
void ts_read_point(uint32_t* x, uint32_t* y, uint32_t* z)
{
	*x = ts_read_x();
	*y = ts_read_y();
	//Se houver falha em alguma das direções, retorna 0
	if((*x != -1) && (*y != -1))
		*z = ts_read_z();
	else
		*z = 0;
}

/*
 * @brief Lê um ponto X na tela
 * @return Posição no eixo x (saída do CAD)
 */
uint32_t ts_read_x()
{
	uint32_t i;
	uint32_t delay = 535;
	int32_t am[AMOSTRAS];
	uint32_t x;

	ts_PIN_ANALOG(YP_PORT, YP_PIN, YP_CHANNEL);
	ts_PIN_INPUT(YM_PORT, YM_PIN);

	ts_PIN_OUTPUT(XP_PORT, XP_PIN);
	ts_PIN_OUTPUT(XM_PORT, XM_PIN);

	ts_PIN_HIGH(XP_PORT, XP_PIN);
	ts_PIN_LOW(XM_PORT, XM_PIN);

	while(--delay);	//~20 us a 84 MHz

	//Leitura seca (teste)
//	HAL_ADC_Start(&hadc1_t);
//	if(HAL_ADC_PollForConversion(&hadc1_t, 5) == HAL_OK)
//	{
//		return HAL_ADC_GetValue(&hadc1_t);
//	}
//	return 1010;

	//Faz as amostras
	for(i=0; i<AMOSTRAS; i++)
	{
		HAL_ADC_Start(&hadc1_t);
		if(HAL_ADC_PollForConversion(&hadc1_t, 5) == HAL_OK)
		{
		   am[i] = HAL_ADC_GetValue(&hadc1_t)>>2;
		}
		else
		{
			return -1;
		}
	}

	//Valida as amostras
	x = am[0];
	for(i=0; i<(AMOSTRAS-1); i++)
	{
		if( ((am[i] - am[i+1]) < -DIF_MAX) || ((am[i] - am[i+1]) > DIF_MAX) )
		{
			return -1;
		}
		else
		{
			x += am[i+1];
		}
	}

	//Média
	x /= AMOSTRAS;

	//Inverte (valor proporcional a resistência XP)
	x = (4095>>2) - x;

	return x;
}

/*
 * @brief Lê um ponto Y na tela
 * @return Posição no eixo y (saída do CAD)
 */
uint32_t ts_read_y()
{
	uint32_t i;
	uint32_t delay = 535;
	int32_t am[AMOSTRAS];
	uint32_t y;

	ts_PIN_ANALOG(XM_PORT, XM_PIN, XM_CHANNEL);
	ts_PIN_INPUT(XP_PORT, XP_PIN);

	ts_PIN_OUTPUT(YP_PORT, YP_PIN);
	ts_PIN_OUTPUT(YM_PORT, YM_PIN);

	ts_PIN_HIGH(YP_PORT, YP_PIN);
	ts_PIN_LOW(YM_PORT, YM_PIN);

	while(--delay);	//~20 us a 84 MHz

	//Leitura seca (teste)
//	HAL_ADC_Start(&hadc1_t);
//	if(HAL_ADC_PollForConversion(&hadc1_t, 5) == HAL_OK)
//	{
//		return HAL_ADC_GetValue(&hadc1_t);
//	}
//	return 1010;

	//Faz as amostras
	for(i=0; i<AMOSTRAS; i++)
	{
		HAL_ADC_Start(&hadc1_t);
		if(HAL_ADC_PollForConversion(&hadc1_t, 5) == HAL_OK)
		{
			am[i] = HAL_ADC_GetValue(&hadc1_t)>>2;
		}
		else
		{
			return -1;
		}
	}

	//Valida as amostras
	y = am[0];
	for(i=0; i<(AMOSTRAS-1); i++)
	{
		if( ((am[i] - am[i+1]) < -DIF_MAX) || ((am[i] - am[i+1]) > DIF_MAX) )
		{
			return -1;
		}
		else
		{
			y += am[i+1];
		}
	}

	//Média
	y /= AMOSTRAS;

	//Inverte (valor proporcional a resistência YP)
	y = (4095>>2) - y;

	return y;
}

/*
 * @brief Lê a pressão do toque na tela
 * @return pressão (valor normalizado)
 */
uint32_t ts_read_z()
{
	uint32_t delay = 535;
	int32_t z1, z2;
	float rtouch;

	ts_PIN_ANALOG(XM_PORT, XM_PIN, XM_CHANNEL);
	ts_PIN_INPUT(YP_PORT, YP_PIN);

	ts_PIN_OUTPUT(XP_PORT, XP_PIN);
	ts_PIN_OUTPUT(YM_PORT, YM_PIN);

	ts_PIN_LOW(XP_PORT, XP_PIN);
	ts_PIN_HIGH(YM_PORT, YM_PIN);

	/*
	 * 												Vy=Z2
	 * Open ou EA	YP -----------.----Ry--- YM Vcc
	 * 													|
	 * 													Rt (resistência do toque)
	 * 													|
	 * 													 /
	 * 													|
	 * 				GND	XP ----Rx-----.--------- XM Open ou EA
	 * 												Vx=Z1
	 */

	while(--delay);	//~20 us a 84 MHz

	//Leitura seca (teste)
//	HAL_ADC_Start(&hadc1_t);
//	if(HAL_ADC_PollForConversion(&hadc1_t, 5) == HAL_OK)
//	{
//		return HAL_ADC_GetValue(&hadc1_t);
//	}
//	return 1010;

	//Lê XM
	HAL_ADC_Start(&hadc1_t);
	if(HAL_ADC_PollForConversion(&hadc1_t, 5) == HAL_OK)
	{
		z1 = HAL_ADC_GetValue(&hadc1_t)>>2;
	}

	//Inverte os canais
	ts_PIN_ANALOG(YP_PORT, YP_PIN, YP_CHANNEL);
	ts_PIN_INPUT(XM_PORT, XM_PIN);

	//Lê YP
	HAL_ADC_Start(&hadc1_t);
	if(HAL_ADC_PollForConversion(&hadc1_t, 5) == HAL_OK)
	{
		z2 = HAL_ADC_GetValue(&hadc1_t)>>2;
	}

	//Para identificar a pressão do toque...
	//Esse cálculo é deduzível a partir da análise de circuitos
	//Sempre z2 > z1
	if (RXPLATE != 0)
	{
		//Acredite, essa conta faz sentido (fiz a dedução)
		rtouch = z2;
		rtouch /= z1;
		rtouch -= 1;
		rtouch *= ts_read_x();			//Lê o X
		rtouch *= RXPLATE;
		rtouch /= (4096>>2);

		return (uint32_t)rtouch;		//Resistência do toque Rt
	}
	else
	{
		return ((4095>>2)-(z2-z1));	//Proporção do toque em relação a resistência total
	}
}

/*
 * @brief  Lê, atualiza e retorna o valor do topo da tela
 * @param [in] modo Posição da tela (MODO_NORMAL - cima para baixo ou MODO_INVERTIDO - baixo para cima)
 * @return Valor do topo da tela
 */
uint32_t ts_top(uint32_t modo)
{
	uint32_t y = ts_read_y();

	//Se houver erro na leitura (limites do CAD)
	if( (y == -1) || (y == (4095>>2)) )
	{
		return edge[TOP];
	}

	if(modo == MODO_NORMAL)
	{
		if(y < edge[TOP])
		{
			edge[TOP] = y;	//Atualiza o limite
		}
	}
	else
	{
		if(y > edge[TOP])
		{
			edge[TOP] = y;	//Atualiza o limite
		}
	}

	return edge[TOP];
}

/*
 * @brief  Lê, atualiza e retorna o valor do pé da tela
 * @param [in] modo Posição da tela (MODO_NORMAL - cima para baixo ou MODO_INVERTIDO - baixo para cima)
 * @return Valor do pé da tela
 */
 uint32_t ts_bottom(uint32_t modo)
{
	uint32_t y = ts_read_y();

	//Se houver erro na leitura (limites do CAD)
	if( (y == -1) || (y == (4095>>2)) )
	{
		return edge[BOTTOM];
	}

	if(modo == MODO_NORMAL)
	{
		if(y > edge[BOTTOM])
		{
			edge[BOTTOM] = y;	//Atualiza o limite
		}
	}
	else
	{
		if(y < edge[BOTTOM])
		{
			edge[BOTTOM] = y;	//Atualiza o limite
		}
	}

	return edge[BOTTOM];
}

/*
 * @brief  Lê, atualiza e retorna o valor da margem esquerda da tela
 * @param [in] modo Posição da tela (MODO_NORMAL - cima para baixo ou MODO_INVERTIDO - baixo para cima)
 * @return Valor da margem esquerda da tela
 */
uint32_t ts_left(uint32_t modo)
{
	uint32_t x = ts_read_x();

	//Se houver erro na leitura (limites do CAD)
	if( (x == -1) || (x == (4095>>2)) )
	{
		return edge[LEFT];
	}

	if(modo == MODO_NORMAL)
	{
		if(x < edge[LEFT])
		{
			edge[LEFT] = x;	//Atualiza o limite
		}
	}
	else
	{
		if(x > edge[LEFT])
		{
			edge[LEFT] = x;	//Atualiza o limite
		}
	}

	return edge[LEFT];
}

/*
 * @brief  Lê, atualiza e retorna o valor da margem direita da tela
 * @param [in] modo Posição da tela (MODO_NORMAL - cima para baixo ou MODO_INVERTIDO - baixo para cima)
 * @return Valor da margem direita da tela
 */
uint32_t ts_right(uint32_t modo)
{
	uint32_t x = ts_read_x();

	//Se houver erro na leitura (limites do CAD)
	if( (x == -1) || (x == (4095>>2)) )
	{
		return edge[RIGHT];
	}

	if(modo == MODO_NORMAL)
	{
		if(x > edge[RIGHT])
		{
			edge[RIGHT] = x;	//Atualiza o limite
		}
	}
	else
	{
		if(x < edge[RIGHT])
		{
			edge[RIGHT] = x;	//Atualiza o limite
		}
	}

	return edge[RIGHT];
}

/*
 * @brief  Atualiza os valores das margens da tela
 * @param [in] left Margem esquerda da tela
 * @param [in] right Margem direita da tela
 * @param [in] top Topo da tela
 * @param [in] bottom Pé da tela
 * @return 0 para sucesso, -1 para erro nas margens
 */
int32_t ts_ajusta_limites(uint32_t left, uint32_t right, uint32_t top, uint32_t bottom)
{
	if((left < (4095>>2)) && (right < (4095>>2)) &&
		 (top < (4095>>2)) && (bottom < (4095>>2)))
	{
		edge[LEFT] = left;
		edge[RIGHT] = right;
		edge[TOP] = top;
		edge[BOTTOM] = bottom;

		return 0;
	}
	else
	{
		return -1;
	}
}

/*
 * @brief  Lê a coordenada na tela em pixels
 * @details	Após a leitura reconfigura os pinos para o LCD TFT
 * @param [out] *px Pixel no eixo x
 * @param [out] *py Pixel no eixo y
 * @param [out] *pz Pressão no toque
 * @return	0 para sucesso, -1 para erro nas medidas ou falta de toque
 */
int32_t ts_read_pixel(uint32_t* px, uint32_t* py, uint32_t* pz)
{
	uint32_t x, y, z;

	//Lê o ponto
	ts_read_point (&x, &y, &z);
	//Retorna a configuração dos GPIOs para LCD TFT
	ts_tft_gpio_init();

	//Se houver erro na leitura (limites do CAD)
	if( (x == -1) || (x == (4095>>2)) ||
			(y == -1) || (y == (4095>>2)) ||
			(z == 0) )
	{
		*px = 0;
		*py = 0;
		*pz = 0;

		return -1;
	}

	//Normaliza x e y
	if(edge[LEFT] > edge[RIGHT])
	{
		*px = (edge[LEFT]-x)*WIDTH/(edge[LEFT]-edge[RIGHT]);
	}
	else
	{
		*px = (x-edge[LEFT])*WIDTH/(edge[RIGHT]-edge[LEFT]);
	}

	if(edge[TOP] > edge[BOTTOM])
	{
		*py = (edge[TOP]-y)*HEIGHT/(edge[TOP]-edge[BOTTOM]);
	}
	else
	{
		*py = (y-edge[TOP])*HEIGHT/(edge[BOTTOM]-edge[TOP]);
	}

	*pz = z;

	return 0;
}


