/**
 ******************************************************************************
 * @file        touch_spi.c
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

/* Includes -----------------------------------------------------------------*/
#include "touch_spi.h"

/* Constantes ---------------------------------------------------------------*/
#define READ_X 0xD0
#define READ_Y 0x90

/* Variáveis globais --------------------------------------------------------*/
static uint16_t rxm = 0xFFFF, rxM = 0, rym = 0xFFFF, ryM = 0;

/* Funções privadas ---------------------------------------------------------*/
/*
 * @brief Habilita a SPI do dispositivo
 */
static void touch_select()
{
    HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);
}

/* Funções públicas ---------------------------------------------------------*/
/*
 * @brief Desabilita a SPI do dispositivo
 */
void touch_unselect()
{
    HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);
}

/*
 * @brief Detecta o toque na tela
 * @return 1 para toque, 0 para não toque
 */
bool touch_pressed()
{
    return HAL_GPIO_ReadPin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin) == GPIO_PIN_RESET;
}

/*
 * @brief Retorna as cooredenadas x e y normalizadas
 * @param [out] x valor de x (normalizado a partir da média de 16 leituras)
 * @param [out] y valor de y (normalizado a partir da média de 16 leituras)
 */
bool touch_getCoordinates(uint16_t* x, uint16_t* y)
{
    static const uint8_t cmd_read_x[] = { READ_X };
    static const uint8_t cmd_read_y[] = { READ_Y };
    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };
 
    touch_select();
 
    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;
    for(uint8_t i = 0; i < 16; i++) {
        if(!touch_pressed())
            break;
 
        nsamples++;
 
        HAL_SPI_Transmit(&TOUCH_SPI_PORT, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), HAL_MAX_DELAY);
        uint8_t y_raw[2];
        HAL_SPI_TransmitReceive(&TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), HAL_MAX_DELAY);
 
        HAL_SPI_Transmit(&TOUCH_SPI_PORT, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), HAL_MAX_DELAY);
        uint8_t x_raw[2];
        HAL_SPI_TransmitReceive(&TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), HAL_MAX_DELAY);
 
        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
    }
 
    touch_unselect();
 
    if(nsamples < 16)
        return false;
 
    uint32_t raw_x = (avg_x / 16);
    if(raw_x < TOUCH_MIN_RAW_X) raw_x = TOUCH_MIN_RAW_X;
    if(raw_x > TOUCH_MAX_RAW_X) raw_x = TOUCH_MAX_RAW_X;
 
    uint32_t raw_y = (avg_y / 16);
    if(raw_y < TOUCH_MIN_RAW_X) raw_y = TOUCH_MIN_RAW_Y;
    if(raw_y > TOUCH_MAX_RAW_Y) raw_y = TOUCH_MAX_RAW_Y;
 
    *x = (raw_x - TOUCH_MIN_RAW_X) * TOUCH_SCALE_X / (TOUCH_MAX_RAW_X - TOUCH_MIN_RAW_X);
    *y = (raw_y - TOUCH_MIN_RAW_Y) * TOUCH_SCALE_Y / (TOUCH_MAX_RAW_Y - TOUCH_MIN_RAW_Y);
 
    return true;
}

/*
 * @brief Retorna as cooredenadas x e y ajuste de coordenadas
 * @param [out] raw_x valor cru de x (média de 16 leituras)
 * @param [out] raw_y valor cru de y (média de 16 leituras)
 */
bool touch_getRawCoordinates(uint16_t* raw_x, uint16_t* raw_y)
{
    static const uint8_t cmd_read_x[] = { READ_X };
    static const uint8_t cmd_read_y[] = { READ_Y };
    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };

    touch_select();

    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;
    for(uint8_t i = 0; i < 16; i++) {
        if(!touch_pressed())
            break;

        nsamples++;

        HAL_SPI_Transmit(&TOUCH_SPI_PORT, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), HAL_MAX_DELAY);
        uint8_t y_raw[2];
        HAL_SPI_TransmitReceive(&TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), HAL_MAX_DELAY);

        HAL_SPI_Transmit(&TOUCH_SPI_PORT, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), HAL_MAX_DELAY);
        uint8_t x_raw[2];
        HAL_SPI_TransmitReceive(&TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), HAL_MAX_DELAY);

        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
    }

    touch_unselect();

    if(nsamples < 16)
        return false;

    *raw_x = (avg_x / 16);

    *raw_y = (avg_y / 16);

    //Atualização dos mínimos e máximos
    rxm = (rxm > *raw_x) ? *raw_x : rxm;
    rxM = (rxM < *raw_x) ? *raw_x : rxM;
    rym = (rym > *raw_y) ? *raw_y : rym;
    ryM = (ryM < *raw_y) ? *raw_y : ryM;

    return true;
}

/*
 * @brief Retorna os valores minimos e máximos de x e y
 * @details Não realiza normalização apresenta os valores memorizados
 * após o uso repetido da função touch_getRawCoordinates.
 * Essa função deve ser usada para calibração da touch.
 * Atualizar as constantes abaixo com os valores obtidos:
 * TOUCH_MIN_RAW_X, TOUCH_MAX_RAW_X,
 * TOUCH_MIN_RAW_Y e TOUCH_MAX_RAW_Y
 */
void touch_getRawMinMaxCoordenadtes(uint16_t* raw_x_min, uint16_t* raw_x_max,
		uint16_t* raw_y_min, uint16_t* raw_y_max)
{
	*raw_x_min = rxm;
	*raw_x_max = rxM;
	*raw_y_min = rym;
	*raw_y_max = ryM;
}
