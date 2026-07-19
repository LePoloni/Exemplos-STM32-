/**
 ******************************************************************************
 * @file        tft_spi_dual_driver.c
 * @brief       Biblioteca controle de LCD TFT SPI via kit ST NUCLEO-F446RE
 ******************************************************************************
 * @attention
 *
 * <h2><center>Desenvolvida para fins acadêmicos</center></h2>
 *
 * @author      Leandro Poloni Dantas
 * @details     Compatível com os drivers ILI9341 e ST7789
 * @date        Abril de 2026
 * @details     Compatível com imagens RGB565.
 *
 * ***************************************************************************
 * @details	Notas:
 * 			Na configuração dos pinos para interface SPI, defina os pinos
 * 			para as funções abaixo com os nomes indicados:
 * 			Chip select: 	TFT_CS
 * 			Reset: 			TFT_RES
 * 			Data/command: 	TFT_DC
 * 			A porta 2 é padrão para SPI. Caso deseje outra porta, crie uma
 * 			definição como indicado abaixo no seu arquivo main.h:
 * 			#define TFT_SPI_PORT hspi<número>
 * 			Para habilitar a leitura de comandos via MISO, crie uma
 * 			definição como indicado abaixo no seu arquivo main.h:
 * 			#define TFT_ENABLE_READ_COMMANDS
 * 			Para definição do tipo de driver utilizado, crie uma definição
 * 			como inidicado abaixo no seu arquivo main.h:
 * 			#define TFT_DRIVER_DEFAULT TFT_DRIVER_ILI9341
 * 			ou
 * 			#define TFT_DRIVER_DEFAULT TFT_DRIVER_ST7789
 *
 * ***************************************************************************
 * @details	Modificações:
 *
 * @date	01/04/2026
 * @version	1.0
 * @details	Funções básicas para driver ILI9341 e ST7789
 *
 * @date	11/07/2026
 * @verion	2.0
 * @details	Foi totalmente revisado e compatibilizado com a biblioteca
 * 			paralela equivalente.
 * 			Passou a trabalhar com fontes do tipo GFX.
 * 			Todas as funções começam com tft_<letra minúscula>...
 * 			Testes feitos com display com driver ILI9341.
 * 			Comentários compatibilizados com o padrão Doxygen.
 *
 *****************************************************************************
 */

/* Includes -----------------------------------------------------------------*/
#include <tft_spi_dual_driver.h>

/* Constantes e macros ------------------------------------------------------*/
//Comandos comuns
#define LCD_SWRESET 0x01
#define LCD_RDDID   0x04
#define LCD_SLPIN   0x10
#define LCD_SLPOUT  0x11
#define LCD_NORON   0x13
#define LCD_INVOFF  0x20
#define LCD_INVON   0x21
#define LCD_DISPOFF 0x28
#define LCD_DISPON  0x29
#define LCD_CASET   0x2A
#define LCD_RASET   0x2B
#define LCD_RAMWR   0x2C
#define LCD_MADCTL  0x36
#define LCD_COLMOD  0x3A

//Específicos do ILI9341
#define ILI9341_POWERA   0xCB
#define ILI9341_POWERB   0xCF
#define ILI9341_DTCA     0xE8
#define ILI9341_DTCB     0xEA
#define ILI9341_POWER_SEQ 0xED
#define ILI9341_PRC      0xF7
#define ILI9341_PWCTR1   0xC0
#define ILI9341_PWCTR2   0xC1
#define ILI9341_VMCTR1   0xC5
#define ILI9341_VMCTR2   0xC7
#define ILI9341_FRMCTR1  0xB1
#define ILI9341_DFUNCTR  0xB6
#define ILI9341_3GAMMA_EN 0xF2
#define ILI9341_GAMMASET 0x26
#define ILI9341_GMCTRP1  0xE0
#define ILI9341_GMCTRN1  0xE1

//Específicos do ST7789
#define ST7789_PORCTRL   0xB2
#define ST7789_GCTRL     0xB7
#define ST7789_VCOMS     0xBB
#define ST7789_LCMCTRL   0xC0
#define ST7789_VDVVRHEN  0xC2
#define ST7789_VRHS      0xC3
#define ST7789_VDVS      0xC4
#define ST7789_FRCTRL2   0xC6
#define ST7789_PWCTRL1   0xD0
#define ST7789_PVGAMCTRL 0xE0
#define ST7789_NVGAMCTRL 0xE1

/* Variáveis globais --------------------------------------------------------*/
static TFT_DriverType g_driver = TFT_DRIVER_UNKNOWN;

uint16_t _width    = TFT_WIDTH;
uint16_t _height   = TFT_HEIGHT;

/**
 * @brief Lê a largura da leta
 * @return largura da tela em pixels
 */
uint16_t width(void)
{ return _width; }

/**
 * @brief Lê a altura da tela
 * @return altura da tela em pixels
 */
uint16_t height(void)
{ return _height; }

uint16_t cursor_y = 0;
uint16_t cursor_x = 0;
uint8_t textsize  = 1;
uint16_t textcolor = 0xFFFF;
uint16_t textbgcolor = 0xFFFF;
uint8_t wrap      = true;
uint8_t _cp437    = false;
uint8_t rotation  = 0;

uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;

/* Funções privadas (no geral) ----------------------------------------------*/
static void tft_select(void)
{
    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
}

void tft_unselect(void)
{
    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
}

static void tft_reset(void)
{
    HAL_GPIO_WritePin(TFT_RES_GPIO_Port, TFT_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(TFT_RES_GPIO_Port, TFT_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(TFT_RES_GPIO_Port, TFT_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
}

static void tft_writeCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&TFT_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
}

static void tft_writeData(const uint8_t* buff, size_t buff_size)
{
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);

    while (buff_size > 0U)
    {
        uint16_t chunk_size = (buff_size > 65535U) ? 65535U : (uint16_t)buff_size;
        HAL_SPI_Transmit(&TFT_SPI_PORT, (uint8_t*)buff, chunk_size, HAL_MAX_DELAY);
        buff += chunk_size;
        buff_size -= chunk_size;
    }
}

static void tft_writeData2(const uint16_t* buff, size_t buff_size)
{
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);

    while (buff_size > 0U)
    {
    	uint16_t chunk_size = (buff_size > 32768U) ? 32768U : (uint16_t)buff_size;
        HAL_SPI_Transmit(&TFT_SPI_PORT, (uint8_t*)buff, chunk_size, HAL_MAX_DELAY);
        buff += chunk_size;
        buff_size -= chunk_size;
    }
}

static void tft_writeDataByte(uint8_t value)
{
    tft_writeData(&value, 1U);
}

static void tft_writeCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
	tft_writeCommand((uint8_t)cmd);
	tft_writeData2((uint16_t*)block, N);
}

static void tft_writeColor(uint16_t color)
{
    uint8_t data[2];
    data[0] = (uint8_t)(color >> 8);
    data[1] = (uint8_t)(color & 0xFF);
    tft_writeData(data, sizeof(data));
}

static uint16_t tft_getXOffset(void)
{
    return (g_driver == TFT_DRIVER_ST7789) ? (uint16_t)TFT_ST7789_X_OFFSET : 0U;
}

static uint16_t tft_getYOffset(void)
{
    return (g_driver == TFT_DRIVER_ST7789) ? (uint16_t)TFT_ST7789_Y_OFFSET : 0U;
}

/**
 * @brief Ajusta uma janela no display
 * @param x0 coordena x inicial
 * @param y0 coordena y inicial
 * @param x1 coordena x final
 * @param y1 coordena y final
 */
static void tft_setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    const uint16_t xs = x0 + tft_getXOffset();
    const uint16_t xe = x1 + tft_getXOffset();
    const uint16_t ys = y0 + tft_getYOffset();
    const uint16_t ye = y1 + tft_getYOffset();
    uint8_t data[4];

    tft_writeCommand(LCD_CASET);
    data[0] = (uint8_t)(xs >> 8);
    data[1] = (uint8_t)(xs & 0xFF);
    data[2] = (uint8_t)(xe >> 8);
    data[3] = (uint8_t)(xe & 0xFF);
    tft_writeData(data, sizeof(data));

    tft_writeCommand(LCD_RASET);
    data[0] = (uint8_t)(ys >> 8);
    data[1] = (uint8_t)(ys & 0xFF);
    data[2] = (uint8_t)(ye >> 8);
    data[3] = (uint8_t)(ye & 0xFF);
    tft_writeData(data, sizeof(data));

    tft_writeCommand(LCD_RAMWR);
}

static void tft_writeColorBurst(uint16_t color, uint32_t count)
{
	//Opção 1 - SPI byte a byte: 242 ms por frame @ SPI 5,25 Mbps
//    uint8_t buffer[TFT_BULK_PIXELS * 2U];
//    uint32_t i;
//
//    for (i = 0U; i < TFT_BULK_PIXELS; i++)
//    {
//        buffer[(i * 2U) + 0U] = (uint8_t)(color >> 8);
//        buffer[(i * 2U) + 1U] = (uint8_t)(color & 0xFF);
//    }
//
//    while (count > 0U)
//    {
//        uint32_t pixels = (count > TFT_BULK_PIXELS) ? TFT_BULK_PIXELS : count;
//        tft_writeData(buffer, pixels * 2U);
//        count -= pixels;
//    }

    //Opção 2 - SPI 2 em 2 bytes: 241 ms por frame @ SPI 5,25 Mbps
    uint16_t buffer[TFT_BULK_PIXELS];
    uint32_t i;

    for (i = 0U; i < TFT_BULK_PIXELS; i++)
    {
    	buffer[i] = color;
    }

    //Ajusta SPI para 16 bits
    TFT_SPI_PORT.Init.DataSize = SPI_DATASIZE_16BIT;
    if (HAL_SPI_Init(&TFT_SPI_PORT) != HAL_OK)
    	return;

    while (count > 0U)
    {
    	uint32_t pixels = (count > TFT_BULK_PIXELS) ? TFT_BULK_PIXELS : count;
    	tft_writeData2(buffer, pixels);
    	count -= pixels;
    }

    //Volta SPI para 8 bits
    TFT_SPI_PORT.Init.DataSize = SPI_DATASIZE_8BIT;
    if (HAL_SPI_Init(&TFT_SPI_PORT) != HAL_OK)
    	return;
}

static void tft_writePixels(const uint16_t* data, uint32_t count)
{
	//Versão 1 - obsoleta
//    uint8_t buffer[TFT_BULK_PIXELS * 2U];
//
//    while (count > 0U)
//    {
//        uint32_t pixels = (count > TFT_BULK_PIXELS) ? TFT_BULK_PIXELS : count;
//        uint32_t i;
//
//        for (i = 0U; i < pixels; i++)
//        {
//            uint16_t color = data[i];
//            buffer[(i * 2U) + 0U] = (uint8_t)(color >> 8);
//            buffer[(i * 2U) + 1U] = (uint8_t)(color & 0xFF);
//        }
//
//        tft_WriteData(buffer, pixels * 2U);
//        data += pixels;
//        count -= pixels;
//    }

	//Versão 2 - trabalha com envios pela 16 bits na SPI
	//Ajusta SPI para 16 bits
	TFT_SPI_PORT.Init.DataSize = SPI_DATASIZE_16BIT;
	if (HAL_SPI_Init(&TFT_SPI_PORT) != HAL_OK)
		return;

	tft_writeData2(data, count);

	//Volta SPI para 8 bits
	TFT_SPI_PORT.Init.DataSize = SPI_DATASIZE_8BIT;
	if (HAL_SPI_Init(&TFT_SPI_PORT) != HAL_OK)
		return;
}

//static void tft_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
//{
//    uint32_t i;
//    uint32_t j;
//    uint32_t b;
//
//    if ((ch < 32) || (ch > 126))
//    {
//        ch = '?';
//    }
//
//    tft_SetAddressWindow(x, y, x + font.width - 1U, y + font.height - 1U);
//
//    for (i = 0U; i < font.height; i++)
//    {
//        b = font.data[(ch - 32) * font.height + i];
//        for (j = 0U; j < font.width; j++)
//        {
//            tft_WriteColor(((b << j) & 0x8000U) ? color : bgcolor);
//        }
//    }
//}

static void tft_drawPixelSigned(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
    {
        return;
    }

    tft_select();
    tft_setAddressWindow((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);
    tft_writeColor(color);
    tft_unselect();
}

static void tft_fillRectangleSigned(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int32_t end;
    uint32_t total_pixels;

    if ((w == 0) || (h == 0))
    {
        return;
    }

    if (w < 0)
    {
        w = (int16_t)(-w);
        x -= w;
    }
    end = (int32_t)x + (int32_t)w;
    if (x < 0)
    {
        x = 0;
    }
    if (end > (int32_t)_width)
    {
        end = _width;
    }
    w = (int16_t)(end - x);

    if (h < 0)
    {
        h = (int16_t)(-h);
        y -= h;
    }
    end = (int32_t)y + (int32_t)h;
    if (y < 0)
    {
        y = 0;
    }
    if (end > (int32_t)_height)
    {
        end = _height;
    }
    h = (int16_t)(end - y);

    if ((w <= 0) || (h <= 0))
    {
        return;
    }

    total_pixels = (uint32_t)w * (uint32_t)h;

    tft_select();
    tft_setAddressWindow((uint16_t)x, (uint16_t)y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
    tft_writeColorBurst(color, total_pixels);
    tft_unselect();
}

static void tft_drawFastVLineSigned(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    tft_fillRectangleSigned(x, y, 1, h, color);
}

static void tft_drawFastHLineSigned(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    tft_fillRectangleSigned(x, y, w, 1, color);
}

/* Funções de inicialização e setup -----------------------------------------*/

static void tft_initSequence_ILI9341(void)
{
    tft_writeCommand(LCD_SWRESET);
    HAL_Delay(150);

    tft_writeCommand(ILI9341_POWERA);
    {
        const uint8_t data[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_POWERB);
    {
        const uint8_t data[] = {0x00, 0xC1, 0x30};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_DTCA);
    {
        const uint8_t data[] = {0x85, 0x00, 0x78};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_DTCB);
    {
        const uint8_t data[] = {0x00, 0x00};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_POWER_SEQ);
    {
        const uint8_t data[] = {0x64, 0x03, 0x12, 0x81};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_PRC);
    tft_writeDataByte(0x20);

    tft_writeCommand(ILI9341_PWCTR1);
    tft_writeDataByte(0x23);

    tft_writeCommand(ILI9341_PWCTR2);
    tft_writeDataByte(0x10);

    tft_writeCommand(ILI9341_VMCTR1);
    {
        const uint8_t data[] = {0x3E, 0x28};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_VMCTR2);
    tft_writeDataByte(0x86);

    tft_writeCommand(LCD_MADCTL);
    tft_writeDataByte(TFT_ROTATION);

    tft_writeCommand(LCD_COLMOD);
    tft_writeDataByte(0x55);

    tft_writeCommand(ILI9341_FRMCTR1);
    {
        const uint8_t data[] = {0x00, 0x18};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_DFUNCTR);
    {
        const uint8_t data[] = {0x08, 0x82, 0x27};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_3GAMMA_EN);
    tft_writeDataByte(0x00);

    tft_writeCommand(ILI9341_GAMMASET);
    tft_writeDataByte(0x01);

    tft_writeCommand(ILI9341_GMCTRP1);
    {
        const uint8_t data[] = {
            0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
            0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00
        };
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ILI9341_GMCTRN1);
    {
        const uint8_t data[] = {
            0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
            0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F
        };
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(LCD_SLPOUT);
    HAL_Delay(120);
    tft_writeCommand(LCD_DISPON);
    HAL_Delay(20);
}

static void tft_initSequence_ST7789(void)
{
    tft_writeCommand(LCD_SWRESET);
    HAL_Delay(150);

    tft_writeCommand(LCD_SLPOUT);
    HAL_Delay(120);

    tft_writeCommand(LCD_COLMOD);
    tft_writeDataByte(0x55);
    HAL_Delay(10);

    tft_writeCommand(LCD_MADCTL);
    tft_writeDataByte(TFT_ROTATION);

    tft_writeCommand(ST7789_PORCTRL);
    {
        const uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ST7789_GCTRL);
    tft_writeDataByte(0x35);

    tft_writeCommand(ST7789_VCOMS);
    tft_writeDataByte(0x19);

    tft_writeCommand(ST7789_LCMCTRL);
    tft_writeDataByte(0x2C);

    tft_writeCommand(ST7789_VDVVRHEN);
    tft_writeDataByte(0x01);

    tft_writeCommand(ST7789_VRHS);
    tft_writeDataByte(0x12);

    tft_writeCommand(ST7789_VDVS);
    tft_writeDataByte(0x20);

    tft_writeCommand(ST7789_FRCTRL2);
    tft_writeDataByte(0x0F);

    tft_writeCommand(ST7789_PWCTRL1);
    {
        const uint8_t data[] = {0xA4, 0xA1};
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ST7789_PVGAMCTRL);
    {
        const uint8_t data[] = {
            0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,
            0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23
        };
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(ST7789_NVGAMCTRL);
    {
        const uint8_t data[] = {
            0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,
            0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23
        };
        tft_writeData(data, sizeof(data));
    }

    tft_writeCommand(TFT_ST7789_INVERSION ? LCD_INVON : LCD_INVOFF);
    HAL_Delay(10);
    tft_writeCommand(LCD_NORON);
    HAL_Delay(10);
    tft_writeCommand(LCD_DISPON);
    HAL_Delay(100);
}

__attribute__((weak)) TFT_DriverType tft_detectDriverHook(void)
{
    return TFT_DRIVER_UNKNOWN;
}

#if TFT_ENABLE_READ_COMMANDS
static bool tft_ReadCommandBytes(uint8_t cmd, uint8_t* data, size_t len)
{
    uint8_t dummy = 0x00;

    if ((data == NULL) || (len == 0U))
    {
        return false;
    }

    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&TFT_SPI_PORT, &cmd, 1U, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
    for (size_t i = 0U; i < len; i++)
    {
        if (HAL_SPI_TransmitReceive(&TFT_SPI_PORT, &dummy, &data[i], 1U, HAL_MAX_DELAY) != HAL_OK)
        {
            return false;
        }
    }
    return true;
}
#endif

TFT_DriverType tft_detectDriver(void)
{
    TFT_DriverType hook_result = tft_detectDriverHook();
    if (hook_result == TFT_DRIVER_ILI9341 || hook_result == TFT_DRIVER_ST7789)
    {
        return hook_result;
    }

#if TFT_ENABLE_READ_COMMANDS
    {
        uint8_t id[4] = {0};

        tft_select();
        if (tft_readCommandBytes(LCD_RDDID, id, sizeof(id)))
        {
            tft_unselect();
            if ((id[1] == 0x93U) && ((id[2] == 0x41U) || (id[3] == 0x41U)))
            {
                return TFT_DRIVER_ILI9341;
            }

            /* A lot of ST7789 modules either do not return a useful ID or return zeros.
             * Do not force a false positive here.
             */
            return TFT_DRIVER_UNKNOWN;
        }
        tft_unselect();
    }
#endif

    return TFT_DRIVER_UNKNOWN;
}

TFT_DriverType tft_getDriver(void)
{
    return g_driver;
}

const char* tft_getDriverName(void)
{
    switch (g_driver)
    {
        case TFT_DRIVER_ILI9341: return "ILI9341";
        case TFT_DRIVER_ST7789:  return "ST7789";
        case TFT_DRIVER_AUTO:    return "AUTO";
        default:                     return "UNKNOWN";
    }
}

void tft_initEx(TFT_DriverType driver)
{
    tft_select();
    tft_reset();

    if (driver == TFT_DRIVER_AUTO)
    {
        driver = tft_detectDriver();
        if (driver == TFT_DRIVER_UNKNOWN)
        {
            driver = TFT_DRIVER_DEFAULT;
        }
    }

    if (driver == TFT_DRIVER_UNKNOWN)
    {
        driver = TFT_DRIVER_DEFAULT;
    }

    g_driver = driver;

    if (g_driver == TFT_DRIVER_ST7789)
    {
        tft_initSequence_ST7789();
    }
    else
    {
        g_driver = TFT_DRIVER_ILI9341;
        tft_initSequence_ILI9341();
    }

    tft_unselect();
}

void tft_init(void)
{
#if TFT_DETECT_ON_INIT
    tft_InitEx(TFT_DRIVER_AUTO);
#else
    tft_initEx((TFT_DriverType)TFT_DRIVER_DEFAULT);
#endif
}

/**
 * @brief Muda a orientação da tela (giro anti-horário)
 * @param r 0 para 0 graus, 1 ou 90 para 90 graus,
 * 2 ou 180 para 180 graus e 3 ou 270 para 270 graus,
 * default 0 graus
 */
void tft_setRotation(uint16_t r)
{
	rotation = (r / 90) & 3;

    switch (rotation)
    {
        case 0:	//Retrato (0 graus)
            _width  = TFT_WIDTH;
            _height = TFT_HEIGHT;
            _lcd_madctl = 0x48;   // MY=0, MX=0, MV=0, BGR=1
            break;

        case 1:	//Paisagem (90 graus)
            _width  = TFT_HEIGHT;
            _height = TFT_WIDTH;
            _lcd_madctl = 0x28;   // MV=1, BGR=1
            break;

        case 2:	//Retrato (180 graus)
            _width  = TFT_WIDTH;
            _height = TFT_HEIGHT;
            _lcd_madctl = 0x88;   // MX=1, BGR=1
            break;

        case 3:	//Paisagem (270 graus)
            _width  = TFT_HEIGHT;
            _height = TFT_WIDTH;
            _lcd_madctl = 0xE8;   // MY=1, MX=1, MV=1, BGR=1
            break;

        default://Retrato (0 graus)
        	_width  = TFT_WIDTH;
        	_height = TFT_HEIGHT;
        	_lcd_madctl = 0x48;   // MY=0, MX=0, MV=0, BGR=1
        	break;
    }

    /* Se o seu painel for RGB em vez de BGR, remova o bit 0x08 */
    tft_select();
    tft_writeCommand(LCD_MADCTL);
    tft_writeDataByte(_lcd_madctl);

    tft_setAddressWindow(0, 0, _width - 1, _height - 1);
    tft_unselect();
}

/**
 * @brief Retorna a orientação da tela (giro horário)
 * @return 0 para 0 graus, 1 para 90 graus,
 * 2 para 180 graus e 3 para 270 graus
 */
uint8_t tft_getRotation(void)
{
    return rotation;
}

/* Funções de formas geométricas --------------------------------------------*/

/**
 * @brief Envia um pixel para o display
 * @param x coordena x
 * @param y coordena y
 * @param color cor no formato RGB565
 */
void tft_drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    tft_drawPixelSigned((int16_t)x, (int16_t)y, color);
}

void tft_vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa;    // bottom fixed area
    int16_t vsp;    // vertical scroll start position
    uint8_t d[6];

    if (scrollines <= 0) return;

    /* Limita offset para a faixa válida */
    if (offset <= -scrollines || offset >= scrollines) {
        offset = 0;
    }

    /* Área fixa inferior */
    bfa = (int16_t)height() - top - scrollines;
    if (bfa < 0) bfa = 0;

    /* ST7789 geralmente usa offsets físicos em alguns módulos */
#if defined(TFT_USE_ST7789_OFFSETS)
    top += TFT_ST7789_TOP_OFFSET;
#endif

    /* Mantém o start dentro da faixa válida */
    vsp = top + offset;
    if (offset < 0) {
        vsp += scrollines;
    }

    tft_select();

    /* Set Vertical Scrolling Area (TFA, VSA, BFA) */
    d[0] = (uint8_t)(top >> 8);
    d[1] = (uint8_t)(top & 0xFF);
    d[2] = (uint8_t)(scrollines >> 8);
    d[3] = (uint8_t)(scrollines & 0xFF);
    d[4] = (uint8_t)(bfa >> 8);
    d[5] = (uint8_t)(bfa & 0xFF);
    tft_writeCmdParamN(0x33, 6, d);

    /* Set Vertical Scroll Start Address */
    d[0] = (uint8_t)(vsp >> 8);
    d[1] = (uint8_t)(vsp & 0xFF);
    tft_writeCmdParamN(0x37, 2, d);

    /* 0x13 = Normal Display Mode On */
    if (offset == 0) {
    	tft_writeCmdParamN(0x13, 0, NULL);
    }

    tft_unselect();
}

/**
 * @brief Desenha um retângulo preenchido
 * @param x coordenada x inicial
 * @param y coordenada y inicial
 * @param w largura em pixels
 * @param h altura em pixels
 * @param color cor RGB565
 */
void tft_fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    tft_fillRectangleSigned((int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, color);
}

/**
 * @brief Preenche a tela
 * @param color cor RGB565
 */
void tft_fillScreen(uint16_t color)
{
    tft_fillRectangleSigned(0, 0, (int16_t)_width, (int16_t)_height, color);
}

/**
 * @brief Displays a 16-bit color BMP image
 * @details A bitmap file that is created
 * by a PC image processing program has a header and may be padded
 * with dummy columns so the data have four byte alignment.  This
 * function assumes that all of that has been stripped out, and the
 * array image[] has one 16-bit halfword for each pixel to be
 * displayed on the screen (encoded in reverse order, which is
 * standard for bitmap files).  An array can be created in this
 * format from a 24-bit-per-pixel .bmp file using the associated
 * converter program.
 * (x,y) is the screen location of the lower left corner of BMP image.
 * Requires (11 + 2*w*h) bytes of transmission (assuming image fully on screen).
 * Must be less than or equal to 320 pixels wide by 240 pixels high.
 * @param x     horizontal position of the bottom left corner of the image, columns from the left edge
 * @param y     vertical position of the bottom left corner of the image, rows from the top edge
 * @param w     number of pixels wide
 * @param h     number of pixels tall
 * @param data pointer to a 16-bit color BMP image
 */
void tft_drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    uint32_t total_pixels;

    if ((data == NULL) || (w == 0U) || (h == 0U))
    {
        return;
    }

    if ((x >= _width) || (y >= _height))
    {
        return;
    }

    if ((x + w) > _width)
    {
        return;
    }

    if ((y + h) > _height)
    {
        return;
    }

    total_pixels = (uint32_t)w * (uint32_t)h;

    tft_select();
    tft_setAddressWindow(x, y, x + w - 1U, y + h - 1U);
    tft_writePixels(data, total_pixels);
    tft_unselect();
}

void tft_drawImageLittleEndian(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    tft_drawImage(x, y, w, h, data);
}

/**
 * @brief Desenha uma linha vertical no modo rápido
 * @param x coordenada x inicial
 * @param y coordenada y inicial
 * @param h altura em pixels
 * @param color cor RGB565
 */
void tft_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    tft_drawFastVLineSigned(x, y, h, color);
}

/**
 * @brief Desenha uma linha horizontal no modo rápido
 * @param x coordenada x inicial
 * @param y coordenada y inicial
 * @param w largura em pixels
 * @param color cor RGB565
 */
void tft_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    tft_drawFastHLineSigned(x, y, w, color);
}

/**
 * @brief Desenha uma linha no display
 * @param x0 coordena x0 inicial
 * @param y0 coordena y0 inicial
 * @param x1 coordena x1 final
 * @param y1 coordena y1 final
 * @param color cor no formato RGB565
 */
void tft_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t steep;
    int16_t dx;
    int16_t dy;
    int16_t err;
    int16_t ystep;

    steep = (int16_t)(abs(y1 - y0) > abs(x1 - x0));

    if (steep)
    {
        int16_t t;
        t = x0; x0 = y0; y0 = t;
        t = x1; x1 = y1; y1 = t;
    }

    if (x0 > x1)
    {
        int16_t t;
        t = x0; x0 = x1; x1 = t;
        t = y0; y0 = y1; y1 = t;
    }

    dx = x1 - x0;
    dy = abs(y1 - y0);
    err = dx / 2;
    ystep = (y0 < y1) ? 1 : -1;

    for (; x0 <= x1; x0++)
    {
        if (steep)
        {
            tft_drawPixelSigned(y0, x0, color);
        }
        else
        {
            tft_drawPixelSigned(x0, y0, color);
        }
        err -= dy;
        if (err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}

/**
 * @brief Desenha uma círculo
 * @param x0 cordenada central x
 * @param y0 cordenada central y
 * @param r raio
 * @param color cor RGB565
 */
void tft_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    tft_drawPixelSigned(x0, y0 + r, color);
    tft_drawPixelSigned(x0, y0 - r, color);
    tft_drawPixelSigned(x0 + r, y0, color);
    tft_drawPixelSigned(x0 - r, y0, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        tft_drawPixelSigned(x0 + x, y0 + y, color);
        tft_drawPixelSigned(x0 - x, y0 + y, color);
        tft_drawPixelSigned(x0 + x, y0 - y, color);
        tft_drawPixelSigned(x0 - x, y0 - y, color);
        tft_drawPixelSigned(x0 + y, y0 + x, color);
        tft_drawPixelSigned(x0 - y, y0 + x, color);
        tft_drawPixelSigned(x0 + y, y0 - x, color);
        tft_drawPixelSigned(x0 - y, y0 - x, color);
    }
}

/**
 * @brief Desenha uma círculo de ajuda
 * @param x0 cordenada central x
 * @param y0 cordenada central y
 * @param r raio
 * @param cornername (?)
 * @param color cor RGB565
 */
void tft_drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x4)
        {
            tft_drawPixelSigned(x0 + x, y0 + y, color);
            tft_drawPixelSigned(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2)
        {
            tft_drawPixelSigned(x0 + x, y0 - y, color);
            tft_drawPixelSigned(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8)
        {
            tft_drawPixelSigned(x0 - y, y0 + x, color);
            tft_drawPixelSigned(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1)
        {
            tft_drawPixelSigned(x0 - y, y0 - x, color);
            tft_drawPixelSigned(x0 - x, y0 - y, color);
        }
    }
}

/**
 * @brief Desenha uma círculo preenchido
 * @param x0 cordenada central x
 * @param y0 cordenada central y
 * @param r raio
 * @param color cor RGB565
 */
void tft_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    tft_drawFastVLineSigned(x0, y0 - r, (int16_t)(2 * r + 1), color);
    tft_fillCircleHelper(x0, y0, r, 3, 0, color);
}

/**
 * @brief Desenha uma círculo de ajuda preenchido
 * @param x0 cordenada central x
 * @param y0 cordenada central y
 * @param r raio
 * @param corners (?)
 * @param delda (?)
 * @param color cor RGB565
 */
void tft_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    int16_t px = x;
    int16_t py = y;

    delta++;

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (x < (y + 1))
        {
            if (corners & 1)
            {
                tft_drawFastVLineSigned(x0 + x, y0 - y, (int16_t)(2 * y + delta), color);
            }
            if (corners & 2)
            {
                tft_drawFastVLineSigned(x0 - x, y0 - y, (int16_t)(2 * y + delta), color);
            }
        }
        if (y != py)
        {
            if (corners & 1)
            {
                tft_drawFastVLineSigned(x0 + py, y0 - px, (int16_t)(2 * px + delta), color);
            }
            if (corners & 2)
            {
                tft_drawFastVLineSigned(x0 - py, y0 - px, (int16_t)(2 * px + delta), color);
            }
            py = y;
        }
        px = x;
    }
}

/**
 * @brief Desenha um retângulo
 * @param x coordenada x inicial
 * @param y coordenada y inicial
 * @param w largura em pixels
 * @param h altura em pixels
 * @param color cor RGB565
 */
void tft_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    tft_drawFastHLineSigned(x, y, w, color);
    tft_drawFastHLineSigned(x, (int16_t)(y + h - 1), w, color);
    tft_drawFastVLineSigned(x, y, h, color);
    tft_drawFastVLineSigned((int16_t)(x + w - 1), y, h, color);
}

/**
 * @brief Desenha um retângulo com cantos arredondados
 * @param x coordenada x inicial
 * @param y coordenada y inicial
 * @param w largura em pixels
 * @param h altura em pixels
 * @param r raio
 * @param color cor RGB565
 */
void tft_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2;
    if (r > max_radius)
    {
        r = max_radius;
    }

    tft_drawFastHLineSigned((int16_t)(x + r), y, (int16_t)(w - 2 * r), color);
    tft_drawFastHLineSigned((int16_t)(x + r), (int16_t)(y + h - 1), (int16_t)(w - 2 * r), color);
    tft_drawFastVLineSigned(x, (int16_t)(y + r), (int16_t)(h - 2 * r), color);
    tft_drawFastVLineSigned((int16_t)(x + w - 1), (int16_t)(y + r), (int16_t)(h - 2 * r), color);

    tft_drawCircleHelper((int16_t)(x + r), (int16_t)(y + r), r, 1, color);
    tft_drawCircleHelper((int16_t)(x + w - r - 1), (int16_t)(y + r), r, 2, color);
    tft_drawCircleHelper((int16_t)(x + w - r - 1), (int16_t)(y + h - r - 1), r, 4, color);
    tft_drawCircleHelper((int16_t)(x + r), (int16_t)(y + h - r - 1), r, 8, color);
}

/**
 * @brief Desenha um retângulo preenchido com cantos arredondados
 * @param x coordenada x inicial
 * @param y coordenada y inicial
 * @param w largura em pixels
 * @param h altura em pixels
 * @param r raio
 * @param color cor RGB565
 */
void tft_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2;
    if (r > max_radius)
    {
        r = max_radius;
    }

    tft_fillRectangleSigned((int16_t)(x + r), y, (int16_t)(w - 2 * r), h, color);
    tft_fillCircleHelper((int16_t)(x + w - r - 1), (int16_t)(y + r), r, 1, (int16_t)(h - 2 * r - 1), color);
    tft_fillCircleHelper((int16_t)(x + r), (int16_t)(y + r), r, 2, (int16_t)(h - 2 * r - 1), color);
}

/**
 * @brief Desenha um triângulo
 * @param x0 coordena x do ponto 0
 * @param y0 coordena y do ponto 0
 * @param x1 coordena x do ponto 1
 * @param y1 coordena y do ponto 1
 * @param x2 coordena x do ponto 2
 * @param y2 coordena y do ponto 2
 * @param color cor RGB565
 */
void tft_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    tft_drawLine(x0, y0, x1, y1, color);
    tft_drawLine(x1, y1, x2, y2, color);
    tft_drawLine(x2, y2, x0, y0, color);
}

/**
 * @brief Desenha um triângulo preenchido
 * @param x0 coordena x do ponto 0
 * @param y0 coordena y do ponto 0
 * @param x1 coordena x do ponto 1
 * @param y1 coordena y do ponto 1
 * @param x2 coordena x do ponto 2
 * @param y2 coordena y do ponto 2
 * @param color cor RGB565
 */
void tft_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a, b, y, last;
    int16_t dx01, dy01;
    int16_t dx02, dy02;
    int16_t dx12, dy12;
    int32_t sa, sb;

    if (y0 > y1)
    {
        int16_t t;
        t = y0; y0 = y1; y1 = t;
        t = x0; x0 = x1; x1 = t;
    }
    if (y1 > y2)
    {
        int16_t t;
        t = y2; y2 = y1; y1 = t;
        t = x2; x2 = x1; x1 = t;
    }
    if (y0 > y1)
    {
        int16_t t;
        t = y0; y0 = y1; y1 = t;
        t = x0; x0 = x1; x1 = t;
    }

    if (y0 == y2)
    {
        a = b = x0;
        if (x1 < a)
        {
            a = x1;
        }
        else if (x1 > b)
        {
            b = x1;
        }
        if (x2 < a)
        {
            a = x2;
        }
        else if (x2 > b)
        {
            b = x2;
        }
        tft_drawFastHLineSigned(a, y0, (int16_t)(b - a + 1), color);
        return;
    }

    dx01 = x1 - x0;
    dy01 = y1 - y0;
    dx02 = x2 - x0;
    dy02 = y2 - y0;
    dx12 = x2 - x1;
    dy12 = y2 - y1;
    sa = 0;
    sb = 0;

    if (y1 == y2)
    {
        last = y1;
    }
    else
    {
        last = (int16_t)(y1 - 1);
    }

    for (y = y0; y <= last; y++)
    {
        a = (int16_t)(x0 + sa / dy01);
        b = (int16_t)(x0 + sb / dy02);
        sa += dx01;
        sb += dx02;
        if (a > b)
        {
            int16_t t = a; a = b; b = t;
        }
        tft_drawFastHLineSigned(a, y, (int16_t)(b - a + 1), color);
    }

    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for (; y <= y2; y++)
    {
        a = (int16_t)(x1 + sa / dy12);
        b = (int16_t)(x0 + sb / dy02);
        sa += dx12;
        sb += dx02;
        if (a > b)
        {
            int16_t t = a; a = b; b = t;
        }
        tft_drawFastHLineSigned(a, y, (int16_t)(b - a + 1), color);
    }
}

/**
 * @brief Inverte as cores do display (efeito negativo_)
 * @param i 0 desliga o a inversão e 1 liga
 */
void tft_invertDisplay(bool invert)
{
    tft_select();
    tft_writeCommand(invert ? LCD_INVON : LCD_INVOFF);
    tft_unselect();
}

/* Funções de texto ---------------------------------------------------------*/

/* Se ainda não existir no seu tft.c, adicione esta global */
//static const GFXfont *gfxFont = NULL;

/* Se já existir em outro lugar, mantenha só uma definição */
int8_t yOffsetLargest;

/**
 * @brief Set the font to display when print()ing, either custom or default
 * @param  f  The GFXfont object, if NULL use built in 6x8 font
 */
void tft_setFont(const GFXfont *f)
{
    if (f) {
        if (!gfxFont) {
            cursor_y += 6;
        }
    } else if (gfxFont) {
        cursor_y -= 6;
    }

    gfxFont = f;

    if (gfxFont) {
        uint8_t first = pgm_read_byte(&gfxFont->first);
        uint8_t last  = pgm_read_byte(&gfxFont->last);

        GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[0]);
        uint16_t i = 0;

        yOffsetLargest = 0;
        while (i++ <= (last - first)) {
            if (yOffsetLargest > glyph->yOffset) {
                yOffsetLargest = glyph->yOffset;
            }
            glyph++;
        }
    } else {
        yOffsetLargest = 0;
    }
}

/**
 * @brief    Helper to determine size of a character with current font/size.
 * 			 Broke this out as it's used by both the - and RAM-resident tft_getTextBounds() functions.
 * @param    c     The ascii character in question
 * @param    x     Pointer to x location of character
 * @param    y     Pointer to y location of character
 * @param    minx  Minimum clipping value for X
 * @param    miny  Minimum clipping value for Y
 * @param    maxx  Maximum clipping value for X
 * @param    maxy  Maximum clipping value for Y
 */
void tft_charBounds(char c, int16_t *x, int16_t *y,
                    int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy)
{
    if (!gfxFont) {
        if (c == '\n') {
            *x = 0;
            *y += textsize * 8;
        } else if (c != '\r') {
            if (wrap && ((*x + textsize * 6) > _width)) {
                *x = 0;
                *y += textsize * 8;
            }

            int16_t x2 = *x + textsize * 6 - 1;
            int16_t y2 = *y + textsize * 8 - 1;

            if (*x < *minx) *minx = *x;
            if (*y < *miny) *miny = *y;
            if (x2 > *maxx) *maxx = x2;
            if (y2 > *maxy) *maxy = y2;

            *x += textsize * 6;
        }
        return;
    }

    if (c == '\n') {
        *x = 0;
        *y += textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
    } else if (c != '\r') {
        uint8_t first = pgm_read_byte(&gfxFont->first);
        uint8_t last  = pgm_read_byte(&gfxFont->last);

        if ((c >= first) && (c <= last)) {
            GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c - first]);
            uint8_t gw = pgm_read_byte(&glyph->width);
            uint8_t gh = pgm_read_byte(&glyph->height);
            uint8_t xa = pgm_read_byte(&glyph->xAdvance);
            int8_t xo = pgm_read_byte(&glyph->xOffset);
            int8_t yo = pgm_read_byte(&glyph->yOffset);

            if (wrap && ((*x + (((int16_t)xo + gw) * textsize)) > _width)) {
                *x = 0;
                *y += textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
            }

            int16_t ts = (int16_t)textsize;
            int16_t x1 = *x + xo * ts;
            int16_t y1 = *y + yo * ts;
            int16_t x2 = x1 + gw * ts - 1;
            int16_t y2 = y1 + gh * ts - 1;

            if (x1 < *minx) *minx = x1;
            if (y1 < *miny) *miny = y1;
            if (x2 > *maxx) *maxx = x2;
            if (y2 > *maxy) *maxy = y2;

            *x += xa * ts;
        }
    }
}

/**
 * @brief    Helper to determine size of a string with current font/size. Pass string and a cursor position, returns UL corner and W,H.
 * @param    str     The ascii string to measure
 * @param    x       The current cursor X
 * @param    y       The current cursor Y
 * @param    x1      The boundary X coordinate, set by function
 * @param    y1      The boundary Y coordinate, set by function
 * @param    w       The boundary width, set by function
 * @param    h       The boundary height, set by function
 */
void tft_getTextBounds(const char *str, int16_t x, int16_t y,
                       int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h)
{
    uint8_t c;

    *x1 = x;
    *y1 = y;
    *w = *h = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while ((c = *str++)) {
        tft_charBounds((char)c, &x, &y, &minx, &miny, &maxx, &maxy);
    }

    if (maxx >= minx) {
        *x1 = minx;
        *w  = maxx - minx + 1;
    }
    if (maxy >= miny) {
        *y1 = miny;
        *h  = maxy - miny + 1;
    }
}

/**
 * @brief Desenho de caractere
 * @param x coordenada x
 * @param y coordenada y
 * @param c caractere
 * @param color cor RGB565
 * @param bg cor de fundo RGB565
 * @param size escala da fonte
 */
void tft_drawChar(int16_t x, int16_t y, unsigned char c,
                  uint16_t color, uint16_t bg, uint8_t size)
{
    if (!gfxFont) {
        return;
    }

    if ((c < pgm_read_byte(&gfxFont->first)) || (c > pgm_read_byte(&gfxFont->last))) {
        return;
    }

    c -= (uint8_t)pgm_read_byte(&gfxFont->first);
    GFXglyph *glyph  = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
    uint8_t  *bitmap = (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);

    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t w   = pgm_read_byte(&glyph->width);
    uint8_t h   = pgm_read_byte(&glyph->height);
    int8_t xo   = pgm_read_byte(&glyph->xOffset);
    int8_t yo   = pgm_read_byte(&glyph->yOffset);

    uint8_t xx, yy, bits = 0, bit = 0;
    int16_t xo16 = (int16_t)xo;
    int16_t yo16 = (int16_t)yo;

    for (yy = 0; yy < h; yy++) {
        for (xx = 0; xx < w; xx++) {
            if (!(bit++ & 7)) {
                bits = pgm_read_byte(&bitmap[bo++]);
            }

            if (bits & 0x80) {
                if (size == 1) {
                    tft_drawPixel(x + xo + xx, y + yo + yy, color);
                } else {
                    tft_fillRect(x + (xo16 + xx) * size, y + (yo16 + yy) * size,
                                 size, size, color);
                }
            } else if (bg != color) {
                if (size == 1) {
                    tft_drawPixel(x + xo + xx, y + yo + yy, bg);
                } else {
                    tft_fillRect(x + (xo16 + xx) * size, y + (yo16 + yy) * size,
                                 size, size, bg);
                }
            }

            bits <<= 1;
        }
    }
}

/**
 * @brief  Print one byte/character of data, used to support print()
 * @param  c  The 8-bit ascii character to write
 * @return sempre 1
 */
size_t tft_write(uint8_t c)
{
    if (!gfxFont) {
        if (c == '\n') {
            cursor_x = 0;
            cursor_y += textsize * 8;
        } else if (c != '\r') {
            if (wrap && ((cursor_x + textsize * 6) > _width)) {
                cursor_x = 0;
                cursor_y += textsize * 8;
            }

            /* Aqui você pode trocar por uma fonte clássica 5x7 se quiser.
               Nesta versão, a escrita funcional fica focada nas fontes GFX. */
            cursor_x += textsize * 6;
        }
        return 1;
    }

    if (c == '\n') {
        cursor_x = 0;
        cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
    } else if (c != '\r') {
        uint8_t first = pgm_read_byte(&gfxFont->first);

        if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
            GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c - first]);
            uint8_t gw = pgm_read_byte(&glyph->width);
            uint8_t gh = pgm_read_byte(&glyph->height);

            if ((gw > 0) && (gh > 0)) {
                int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset);
                if (wrap && ((cursor_x + textsize * (xo + gw)) > _width)) {
                    cursor_x = 0;
                    cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                }

                tft_drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
            }

            cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
        }
    }

    return 1;
}

/**
 * @brief  Print one byte/character of data, used to support print()
 * 		   print the background first with the textbgcolor
 * @param  c  The 8-bit ascii character to write
 * @return sempre 1
 */
size_t tft_write_fillbackground(uint8_t c)
{
    if (!gfxFont) {
        return tft_write(c);
    }

    if (c == '\n') {
        cursor_x = 0;
        cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        return 1;
    } else if (c == '\r') {
        return 1;
    }

    uint8_t first = pgm_read_byte(&gfxFont->first);
    uint8_t last  = pgm_read_byte(&gfxFont->last);

    if ((c < first) || (c > last)) {
        return 1;
    }

    GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c - first]);

    int16_t xx = cursor_x;
    int8_t yo = yOffsetLargest - (pgm_read_byte(&gfxFont->yAdvance) + yOffsetLargest) / 2 - 1;
    int16_t yy = (int16_t)cursor_y + yo * textsize;
    int16_t ww = (int16_t)pgm_read_byte(&glyph->xAdvance) * textsize;
    int16_t hh = ((int16_t)pgm_read_byte(&gfxFont->yAdvance) + 2) * textsize;

    tft_fillRect(xx, yy, ww, hh, textbgcolor);

    uint8_t w = pgm_read_byte(&glyph->width);
    uint8_t h = pgm_read_byte(&glyph->height);

    if ((w > 0) && (h > 0)) {
        int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset);

        if (wrap && ((cursor_x + textsize * (xo + w)) > _width)) {
            cursor_x = 0;
            cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);

            xx = cursor_x;
            yy = (int16_t)cursor_y + yo * textsize;
            tft_fillRect(xx, yy, ww, hh, textbgcolor);
        }

        tft_drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
    }

    cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
    return 1;
}

/**
 * @brief Print a new string on the TFT LCD
 * @param  row	The vertical pixel of botton left (BL) cursor
 * @param  txtcolor	The color of the text
 * @param  txtcolor	The color of the text
 * @param	txtsize	The size of de text
 * @param	str	A point to the string
 */
void tft_printNewStr(int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str)
{
    tft_setFont(f);
    textcolor = txtcolor;
    textsize = (txtsize > 0) ? txtsize : 1;
    tft_setCursor(0, row);
    while (*str) {
        tft_write(*str++);
    }
}

/**
 * @brief Print a new string on the TFT LCD using the background color
 * @param  row	The vertical pixel of botton left (BL) cursor
 * @param  txtcolor	The color of the text
 * @param	txtbackcolor	The text background color
 * @param  txtcolor	The color of the text
 * @param	txtsize	The size of de text
 * @param	str	A point to the string
 */
void tft_printNewStr_bc(int row, uint16_t txtcolor, uint16_t txtbackcolor,
                         const GFXfont *f, uint8_t txtsize, uint8_t *str)
{
    tft_setFont(f);
    textcolor = txtcolor;
    textbgcolor = txtbackcolor;
    textsize = (txtsize > 0) ? txtsize : 1;
    tft_setCursor(0, row);
    while (*str) {
        tft_write_fillbackground(*str++);
    }
}

/**
 * @brief Print a string on the TFT LCD
 * @param	str	A point to the string
 */
void tft_printStr(uint8_t *str)
{
    while (*str) {
        tft_write(*str++);
    }
}

/**
 * @brief Print a string on the TFT LCD using the background color
 * @param	str	A point to the string
 */
void tft_printStr_bc(uint8_t *str)
{
    while (*str) {
        tft_write_fillbackground(*str++);
    }
}

/**
 * @brief Enable the wrap text (quebra automática de texto)
 * @param	w	Boolean to enable text wrapping
 */
void tft_setTextWrap(uint8_t w)
{
    wrap = w;
}

/**
 * @brief Set the text color
 * @param	color Value of a RGB565 color to the text
 */
void tft_setTextColor(uint16_t color)
{
    textcolor = color;
}

/**
 * @brief Set the background text color
 * @param	color Value of a RGB565 color to the text
 */
void tft_setTextBackColor(uint16_t color)
{
    textbgcolor = color;
}

/**
 * @brief Ajusta da escala da fonte
 * @param size escala (1=100%, 2=200%, ...)
 */
void tft_setTextSize(uint8_t size)
{
    textsize = (size > 0) ? size : 1;
}

/**
 * @brief Ajusta da posição do cursor
 * @param x coordena x
 * @param y coordena y
 */
void tft_setCursor(int16_t x, int16_t y)
{
    cursor_x = x;
    cursor_y = y;
}

/**
 * @brief Ajuste de velocidade de scroll up
 * @param speed velocidade
 */
void tft_scrollup(uint16_t speed)
{
    uint16_t maxscroll = (tft_getRotation() & 1) ? width() : height();

    for (uint16_t i = 1; i <= maxscroll; i++) {
        tft_vertScroll(0, maxscroll, i);
        if (speed < 655) {
            HAL_Delay(speed);
        } else {
            HAL_Delay(speed);
        }
    }
}

/**
 * @brief Ajuste de velocidade de scroll down
 * @param speed velocidade
 */
void tft_scrolldown(uint16_t speed)
{
    uint16_t maxscroll = (tft_getRotation() & 1) ? width() : height();

    for (uint16_t i = 1; i <= maxscroll; i++) {
        tft_vertScroll(0, maxscroll, 0 - (int16_t)i);
        if (speed < 655) {
            HAL_Delay(speed);
        } else {
            HAL_Delay(speed);
        }
    }
}

/* Funções de teste ---------------------------------------------------------*/

/**
 * @brief Teste de preenchimento de tela
 */
void tft_testfillScreen()
{
	tft_fillScreen(BLACK);
	tft_fillScreen(RED);
	tft_fillScreen(GREEN);
	tft_fillScreen(BLUE);
	tft_fillScreen(BLACK);
}

/**
 * @brief Teste de desenho de linhas
 * @param color cor RGB565
 */
void tft_testLines(uint16_t color)
{
	int           x1, y1, x2, y2,
	w = width(),
	h = height();

	tft_fillScreen(BLACK);

	x1 = y1 = 0;
	y2    = h - 1;
	for (x2 = 0; x2 < w; x2 += 6) tft_drawLine(x1, y1, x2, y2, color);
	x2    = w - 1;
	for (y2 = 0; y2 < h; y2 += 6) tft_drawLine(x1, y1, x2, y2, color);

	tft_fillScreen(BLACK);

	x1    = w - 1;
	y1    = 0;
	y2    = h - 1;
	for (x2 = 0; x2 < w; x2 += 6) tft_drawLine(x1, y1, x2, y2, color);
	x2    = 0;
	for (y2 = 0; y2 < h; y2 += 6) tft_drawLine(x1, y1, x2, y2, color);

	tft_fillScreen(BLACK);

	x1    = 0;
	y1    = h - 1;
	y2    = 0;
	for (x2 = 0; x2 < w; x2 += 6) tft_drawLine(x1, y1, x2, y2, color);
	x2    = w - 1;
	for (y2 = 0; y2 < h; y2 += 6) tft_drawLine(x1, y1, x2, y2, color);

	tft_fillScreen(BLACK);

	x1    = w - 1;
	y1    = h - 1;
	y2    = 0;
	for (x2 = 0; x2 < w; x2 += 6) tft_drawLine(x1, y1, x2, y2, color);
	x2    = 0;
	for (y2 = 0; y2 < h; y2 += 6) tft_drawLine(x1, y1, x2, y2, color);

}

/**
 * @brief Teste de desenho de linhas rapidas
 * @param color1 cor RGB565
 * @param color2 cor RGB565
 */
void tft_testFastLines(uint16_t color1, uint16_t color2)
{
	int           x, y, w = width(), h = height();

	tft_fillScreen(BLACK);
	for (y = 0; y < h; y += 5) tft_drawFastHLine(0, y, w, color1);
	for (x = 0; x < w; x += 5) tft_drawFastVLine(x, 0, h, color2);
}

/**
 * @brief Teste de desenho de retângulos
 * @param color cor RGB565
 */
void tft_testRects(uint16_t color)
{
	int           n, i, i2,
	cx = width()  / 2,
	cy = height() / 2;

	tft_fillScreen(BLACK);
	n     = min(width(), height());
	for (i = 2; i < n; i += 6) {
		i2 = i / 2;
		tft_drawRect(cx - i2, cy - i2, i, i, color);
	}

}

/**
 * @brief Teste de desenho de retângulos preenchidos
 * @param color1 cor RGB565
 * @param color2 cor RGB565
 */
void tft_testFilledRects(uint16_t color1, uint16_t color2)
{
	int           n, i, i2,
	cx = width()  / 2 - 1,
	cy = height() / 2 - 1;

	tft_fillScreen(BLACK);
	n = min(width(), height());
	for (i = n; i > 0; i -= 6) {
		i2    = i / 2;

		tft_fillRect(cx - i2, cy - i2, i, i, color1);

		tft_drawRect(cx - i2, cy - i2, i, i, color2);
	}
}

/**
 * @brief Teste de desenho de círculos preenchidos
 * @param radius raio
 * @param color cor RGB565
 */
 void tft_testFilledCircles(uint8_t radius, uint16_t color)
{
	int x, y, w = width(), h = height(), r2 = radius * 2;

	tft_fillScreen(BLACK);
	for (x = radius; x < w; x += r2) {
		for (y = radius; y < h; y += r2) {
			tft_fillCircle(x, y, radius, color);
		}
	}
}

/**
 * @brief Teste de desenho de círculos
 * @param radius raio
 * @param color cor RGB565
 */
void tft_testCircles(uint8_t radius, uint16_t color)
{
	int           x, y, r2 = radius * 2,
			w = width()  + radius,
			h = height() + radius;

	// Screen is not cleared for this one -- this is
	// intentional and does not affect the reported time.
	for (x = 0; x < w; x += r2) {
		for (y = 0; y < h; y += r2) {
			tft_drawCircle(x, y, radius, color);
		}
	}

}

/**
 * @brief Teste de desenho de triângulos
 */
void tft_testTriangles()
{
	int n, i, cx = width()  / 2 - 1,
			  cy = height() / 2 - 1;

	tft_fillScreen(BLACK);
	n     = min(cx, cy);
	for (i = 0; i < n; i += 5) {
		tft_drawTriangle(
				cx    , cy - i, // peak
				cx - i, cy + i, // bottom left
				cx + i, cy + i, // bottom right
				TFT_COLOR565(0, 0, i));
	}
}

/**
 * @brief Teste de desenho de triângulos preenchidos
 */
void tft_testFilledTriangles()
{
	int i, cx = width()  / 2 - 1,
		   cy = height() / 2 - 1;

	tft_fillScreen(BLACK);
	for (i = min(cx, cy); i > 10; i -= 5) {
		tft_fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
				TFT_COLOR565(0, i, i));
		tft_drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
				TFT_COLOR565(i, i, 0));
	}
}

/**
 * @brief Teste de desenho de retângulos com cantos arredondados
 */
void tft_testRoundRects()
{
	int           w, i, i2, red, step,
	cx = width()  / 2 - 1,
	cy = height() / 2 - 1;

	tft_fillScreen(BLACK);
	w     = min(width(), height());
	red = 0;
	step = (256 * 6) / w;
	for (i = 0; i < w; i += 6) {
		i2 = i / 2;
		red += step;
		tft_drawRoundRect(cx - i2, cy - i2, i, i, i / 8, TFT_COLOR565(red, 0, 0));
	}
}

/**
 * @brief Teste de desenho de retângulos peenchidos com cantos arredondados
 */
void tft_testFilledRoundRects()
{
	int           i, i2, green, step,
	cx = width()  / 2 - 1,
	cy = height() / 2 - 1;

	tft_fillScreen(BLACK);
	green = 256;
	step = (256 * 6) / min(width(), height());
	for (i = min(width(), height()); i > 20; i -= 6) {
		i2 = i / 2;
		green -= step;
		tft_fillRoundRect(cx - i2, cy - i2, i, i, i / 8, TFT_COLOR565(0, green, 0));
	}
}

/* Funções de interação com a câmera e SD Card ------------------------------*/

/**
 * @brief Desenha um pixel no display
 * @details Otimizada para gerar melhor desempenho
 * @param pixel cor no formato RGB565
 */
void tft_desenhaPixel(uint16_t pixel)
{
	//Opção 1 - mais lento
//	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
//	uint8_t data[2] = { pixel >> 8, pixel & 0xFF };
//	HAL_SPI_Transmit(&TFT_SPI_PORT, data, 2, HAL_MAX_DELAY);

	//Opção 2 - mais rápido
	tft_writePixels(&pixel, 1);
}

/**
 * @brief Desenha um conjunto de pixels no display
 * @details Otimizada para gerar melhor desempenho
 * @param pixel cor no formato RGB565
 * @param size quantidade de pixels
 */
void tft_enviaPixels(uint16_t* pixel, uint16_t size)
{
	//Opção 1 - Tempo médio para um frame: 708 ms @ SPI 5,25 Mbps
//	uint8_t data[2];
//	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
//	for(uint16_t i = 0; i < size; i++)
//	{
//		data[0] = pixel[i] >> 8;
//		data[1] = pixel[i] & 0xFF;
//		HAL_SPI_Transmit(&TFT_SPI_PORT, data, 2, HAL_MAX_DELAY);
//	}

	//Opção 2 - Tempo médio para um frame: 240 ms @ SPI 5,25 Mbps
	tft_writePixels(pixel, size);
}

/**
 * @brief Inicia a transmissão de dados
 */
void tft_inicioDados(void)
{
	tft_select();
	tft_writeCommand(LCD_RAMWR);
}

/**
 * @brief Marca o fim da transmissão de dados
 */
void tft_fimDados(void)
{
	tft_unselect();
}

/**
 * @brief Ajusta uma janela no display
 * @param x coordena x inicial
 * @param y coordena y inicial
 * @param x1 coordena x final
 * @param y1 coordena y final
 */
void tft_setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
	tft_select();
	tft_setAddressWindow(x, y, x1, y1);
}
