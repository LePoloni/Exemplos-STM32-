/**
 ******************************************************************************
 * @file        tft_spi_dual_driver.h
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

/* Define para prevenir inclusão recursiva ----------------------------------*/
#ifndef tft_spi_dual_driver_H_
#define tft_spi_dual_driver_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes -----------------------------------------------------------------*/
#include <fonts.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "main.h"

/* Constantes e macros ------------------------------------------------------*/
#define TFT_MADCTL_MY  0x80
#define TFT_MADCTL_MX  0x40
#define TFT_MADCTL_MV  0x20
#define TFT_MADCTL_ML  0x10
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08
#define TFT_MADCTL_MH  0x04

#ifndef TFT_SPI_PORT
#define TFT_SPI_PORT hspi2
#endif
extern SPI_HandleTypeDef TFT_SPI_PORT;

#ifndef TFT_RES_Pin
#define TFT_RES_Pin       GPIO_PIN_0
#define TFT_RES_GPIO_Port GPIOA
#endif

#ifndef TFT_CS_Pin
#define TFT_CS_Pin        GPIO_PIN_1
#define TFT_CS_GPIO_Port  GPIOA
#endif

#ifndef TFT_DC_Pin
#define TFT_DC_Pin        GPIO_PIN_4
#define TFT_DC_GPIO_Port  GPIOA
#endif

/* Optional readback pin definitions. Many SPI TFT modules do not connect MISO. */
#ifndef TFT_ENABLE_READ_COMMANDS
#define TFT_ENABLE_READ_COMMANDS 0
#endif

/* Logical panel size used by drawing functions. */
#ifndef TFT_WIDTH
#define TFT_WIDTH 240
#endif

#ifndef TFT_HEIGHT
#define TFT_HEIGHT 320
#endif

/* Default rotation. Change as needed for your panel orientation. */
#ifndef TFT_ROTATION
#define TFT_ROTATION (TFT_MADCTL_MX | TFT_MADCTL_BGR)
#endif

/* ST7789 panel offsets. Common examples:
 * 240x320 -> 0,0
 * 240x240 -> 0,80 (some modules)
 * 135x240 -> 52,40 (some modules)
 */
#ifndef TFT_ST7789_X_OFFSET
#define TFT_ST7789_X_OFFSET 0
#endif

#ifndef TFT_ST7789_Y_OFFSET
#define TFT_ST7789_Y_OFFSET 0
#endif

#ifndef TFT_ST7789_INVERSION
#define TFT_ST7789_INVERSION 1
#endif

#ifndef TFT_BULK_PIXELS
#define TFT_BULK_PIXELS 64
#endif

/* Driver selection */
typedef enum
{
    TFT_DRIVER_UNKNOWN = 0,
	TFT_DRIVER_ILI9341 = 1,
    TFT_DRIVER_ST7789 = 2,
    TFT_DRIVER_AUTO = 3
} TFT_DriverType;

/* Compile-time default.
 * Set to TFT_DRIVER_tft, TFT_DRIVER_ST7789 or TFT_DRIVER_AUTO.
 */
#ifndef TFT_DRIVER_DEFAULT
#define TFT_DRIVER_DEFAULT TFT_DRIVER_ILI9341
#endif

/* If enabled, tft_Init() calls detection logic first. */
#ifndef TFT_DETECT_ON_INIT
#define TFT_DETECT_ON_INIT 0
#endif

#define TFT_BLACK   0x0000
#define TFT_BLUE    0x001F
#define TFT_RED     0xF800
#define TFT_GREEN   0x07E0
#define TFT_CYAN    0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW  0xFFE0
#define TFT_WHITE   0xFFFF

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define TFT_COLOR565(r, g, b) \
    ((((uint16_t)((r) & 0xF8)) << 8) | (((uint16_t)((g) & 0xFC)) << 3) | (((uint16_t)(b) & 0xF8) >> 3))

#define min(a, b) (((a) < (b)) ? (a) : (b))

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
//#define pgm_read_word(addr) (*(const unsigned short *)(addr))	//Leandro: Essa linha limitava o endereço em 16 bits
#define pgm_read_word(addr) (*(const unsigned long *)(addr))	//com essa correção passa a ler toda a memória
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))

/* Protótipos de funções ---------------------------------------------------*/
void tft_unselect(void);
/* Simple init using compile-time default or auto-detect, depending on macros. */
void tft_init(void);
/* Explicit init choosing the controller at runtime. */
void tft_initEx(TFT_DriverType driver);
/* Detection hook: override in user code if you can distinguish the panel on your board.
 * Example: inspect an ID GPIO, EEPROM, or board revision.
 */
TFT_DriverType tft_detectDriverHook(void);
/* Tries hook first, optional SPI readback second, then returns UNKNOWN if not possible. */
TFT_DriverType tft_detectDriver(void);
TFT_DriverType tft_getDriver(void);
const char* tft_getDriverName(void);
void tft_setRotation(uint16_t r);
uint8_t tft_getRotation(void);
void tft_drawPixel(uint16_t x, uint16_t y, uint16_t color);
void tft_vertScroll(int16_t top, int16_t scrollines, int16_t offset);
void tft_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void tft_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void tft_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void tft_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void tft_drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void tft_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void tft_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void tft_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void tft_fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void tft_drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
void tft_fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
void tft_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void tft_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void tft_fillScreen(uint16_t color);
//void tft_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void tft_drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void tft_drawImageLittleEndian(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void tft_invertDisplay(bool invert);
/* Compatibility aliases with the parallel driver naming style. */
#define tft_invertColors(i)					tft_invertDisplay(i)
#define tft_drawRGBBitmap(x, y, data, w, h) tft_drawImage(x, y, w, h, data)

/* Funções de texto ---------------------------------------------------------*/
void tft_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
size_t tft_write(uint8_t c);
size_t tft_write_fillbackground(uint8_t c);
void tft_setFont(const GFXfont *f);
void tft_charBounds(char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy);
void tft_getTextBounds(const char *str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
void tft_printNewStr(int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str);
void tft_printNewStr_bc(int row, uint16_t txtcolor, uint16_t txtbackcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str);
void tft_printStr(uint8_t *str);
void tft_printStr_bc(uint8_t *str);
void tft_setTextWrap(uint8_t w);
void tft_setTextColor(uint16_t color);
void tft_setTextBackColor(uint16_t color);
void tft_setTextSize(uint8_t size);
void tft_setCursor(int16_t x, int16_t y);
void tft_scrollup(uint16_t speed);
void tft_scrolldown(uint16_t speed);

/* Funções de teste ---------------------------------------------------------*/
void tft_testfillScreen();
void tft_testLines(uint16_t color);
void tft_testFastLines(uint16_t color1, uint16_t color2);
void tft_testRects(uint16_t color);
void tft_testFilledRects(uint16_t color1, uint16_t color2);
void tft_testFilledCircles(uint8_t radius, uint16_t color) ;
void tft_testCircles(uint8_t radius, uint16_t color);
void tft_testTriangles();
void tft_testFilledTriangles();
void tft_testRoundRects();
void tft_testFilledRoundRects();

/* Funções de interação com a câmera e SD Card ------------------------------*/
void tft_desenhaPixel(uint16_t pixel);
void tft_enviaPixels(uint16_t* pixel, uint16_t size);
void tft_inicioDados(void);
void tft_fimDados(void);
void tft_setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);

#ifdef __cplusplus
}
#endif

#endif /* TFT_SPI_DUAL_DRIVER_H_ */
