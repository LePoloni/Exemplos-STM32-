/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>	//sprintf
#include <tft_spi_dual_driver.hxxx>
#include "touch_spi.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Definição da porta SPI do display TFT

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Imagens BMP convertidas para RGB565
extern const unsigned short bean_157x157[24649];
extern const unsigned short eletriciade_200x151[30200];
extern const unsigned short red_100x50[5000];
extern const unsigned short green_100x50[5000];
extern const unsigned short blue_100x50[5000];
extern const unsigned short RGB_100x50[5000];
extern const uint8_t playmobil_240x240[];
extern const uint16_t test_img_240x240[];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void tft_loop(void);
void touch_loop(void);
void colors_loop(void);
void tft_touch_testeToque(void);
void tft_testeDriver(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init()
{
	//Atenção: neste exemplo o display TFT e o touch screen compartilham
	//a mesma porta SPI (SPI3: CLK-Arduino D3, MISO-Ard. D5 e MOSI-Ard. D4).
	//É recomendado que o clock do touch seja menor do que 1 MHz, porque ele
	//é usado para o CAD do chip controlador do touch XPT2046.
	//Isso implica na velocidade máxima do clock para o display.
	//Caso o touch não seja usado, o clock da interface SPI pode ser aumentado
	//até 10 MHz para escrita e 6,7 MHz para leitura considerando o driver
	//de LCD TFT ILI9341.
	//SP3 prescaler = 4 -> 10,5 MHz e 64 -> 656,25 kHz

	tft_unselect();				//Desativa CS do TFT
	touch_unselect();			//Desativa CS do touch
	tft_init();					//Inicializa o display TFT
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  init();	//Inicializa do TFT e o touch
//  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
  HAL_Delay(2000);
  tft_fillScreen(TFT_BLUE);
  tft_printNewStr(30, WHITE, &mono12x7, 1, "BLUE");
  HAL_Delay(500);
  tft_drawImage(20,100,200,151,eletriciade_200x151);
  HAL_Delay(2000);
  //tft_drawImage(0,0,240,240,(const uint16_t*)playmobil_240x240);
  //HAL_Delay(2000);
  //tft_fillScreen(TFT_RED);
  //tft_drawImageLittleEndian(0,0,240,240,(const uint16_t*)playmobil_240x240);
  HAL_Delay(2000);
  tft_testeDriver();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//Escolha o teste desejado
		//tft_loop();
		//touch_loop();
		//colors_loop();
		tft_touch_testeToque();	//Fica em looping infinito
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TFT_RES_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_IRQ_Pin */
  GPIO_InitStruct.Pin = TOUCH_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RES_Pin LD2_Pin */
  GPIO_InitStruct.Pin = TFT_RES_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TFT_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_CS_Pin TFT_DC_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin|TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void tft_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

void tft_loop(void)
{
	//Check border
	tft_fillScreen(TFT_BLACK);

	for(int x = 0; x < TFT_WIDTH; x++)
	{
		tft_drawPixel(x, 0, TFT_RED);
		tft_drawPixel(x, TFT_HEIGHT-1, TFT_RED);
	}

	for(int y = 0; y < TFT_HEIGHT; y++)
	{
		tft_drawPixel(0, y, TFT_RED);
		tft_drawPixel(TFT_WIDTH-1, y, TFT_RED);
	}

	HAL_Delay(1000);

	//Check font
	tft_fillScreen(TFT_BLACK);
	tft_printNewStr(20, TFT_RED, &mono12x7, 1, "Oi mundo!");
	tft_printNewStr(40, TFT_GREEN, &mono12x7, 1, "Oi mundo!");
	tft_printNewStr(60, TFT_BLUE, &mono12x7, 1, "Oi mundo!");

	HAL_Delay(1000);
	tft_invertDisplay(true);
	HAL_Delay(1000);
	tft_invertDisplay(false);

	HAL_Delay(2000);

	//Check colors
	tft_fillScreen(TFT_WHITE);
	tft_printNewStr(20, TFT_RED, &mono12x7, 1, "WHITE");
	HAL_Delay(500);

	tft_fillScreen(TFT_BLUE);
	tft_printNewStr(20, TFT_WHITE, &mono12x7, 1, "BLUE");
	HAL_Delay(500);

	tft_fillScreen(TFT_RED);
	tft_printNewStr(20, TFT_WHITE, &mono12x7, 1, "RED");
	HAL_Delay(500);

	tft_fillScreen(TFT_GREEN);
	tft_printNewStr(20, TFT_WHITE, &mono12x7, 1, "GREEN");
	HAL_Delay(500);

	tft_fillScreen(TFT_CYAN);
	tft_printNewStr(20, TFT_BLACK, &mono12x7, 1, "CYAN");
	HAL_Delay(500);

	tft_fillScreen(TFT_MAGENTA);
	tft_printNewStr(20, TFT_BLACK, &mono12x7, 1, "MAGENTA");
	HAL_Delay(500);

	tft_fillScreen(TFT_YELLOW);
	tft_printNewStr(20, TFT_BLACK, &mono12x7, 1, "YELLOW");
	HAL_Delay(500);

	tft_fillScreen(TFT_BLACK);
	tft_printNewStr(20, TFT_WHITE, &mono12x7, 1, "BLACK");
	HAL_Delay(500);

	tft_drawImage(0,0,200,151,eletriciade_200x151);
	HAL_Delay(2000);
}

void touch_loop(void)
{
  uint16_t x, y, xm, xM, ym, yM;
  char str[50];
  uint16_t size;

  if(touch_getCoordinates(&x, &y))
//  if(touch_getRawCoordinates(&x, &y))	//Usar para calibração
  {
  	size = sprintf(str, "x: %i     ", x);
  	HAL_UART_Transmit(&huart2, str, size, 100);

  	size = sprintf(str, "y: %i\r\n", y);
  	HAL_UART_Transmit(&huart2, str, size, 100);

  	//Usar para calibração
//  	touch_getRawMinMaxCoordenadtes(&xm, &xM, &ym, &yM);
//  	size = sprintf(str, "xm: %i\txM: %i\t ym: %i\t yM: %i\r\n", xm, xM, ym, yM);
//  	HAL_UART_Transmit(&huart2, str, size, 100);
  }
  else
  {
  	HAL_UART_Transmit(&huart2, "Not pressed\r\n", 13, 100);
  }

  HAL_Delay(250);
}

void colors_loop(void)
{
	tft_fillScreen(TFT_BLACK);
	HAL_Delay(500);
	tft_drawImageLittleEndian(0,50-1,100,50,(const uint16_t*)red_100x50);
	HAL_Delay(500);
	tft_drawImageLittleEndian(0,100-1,100,50,(const uint16_t*)green_100x50);
	HAL_Delay(500);
	tft_drawImageLittleEndian(0,150-1,100,50,(const uint16_t*)blue_100x50);
	HAL_Delay(500);
	tft_drawImageLittleEndian(0,200-1,100,50,(const uint16_t*)RGB_100x50);
	HAL_Delay(500);
}

void tft_touch_testeToque(void)
{
	uint16_t x, y;
	char str[50];
	uint16_t size;

	tft_setRotation(0);
	tft_fillScreen(TFT_WHITE);
	tft_setFont(&mono12x7bold);
	tft_setTextBackColor(TFT_WHITE);
	tft_setTextColor(TFT_BLACK);

	while(1)
	{
		if(touch_getCoordinates(&x, &y))
		{
			//Como as coordenas do touch crescem da esquerda para direita,
			//de baixo para cima, as coordenadas y devem ser invertidas para
			//se adequarem a orientação do eixo y do display
			y = 320 - y;
			tft_drawCircle(x, y, 4, TFT_RED);
			tft_setCursor(15, 15);
			size = sprintf(str, "X = %i\t Y = %i    ", x, y);
			tft_printStr_bc(str);
		}
		HAL_Delay(250);
	}
}

void tft_testeDriver(void)
{
    uint16_t w = 0;
    uint16_t h = 0;

    //Funções de teste do LCD (tirar para teste do touch)
    tft_testfillScreen();
    tft_testLines(CYAN);
    tft_testFastLines(RED, BLUE);
    tft_testFilledCircles(10, MAGENTA);
    tft_testCircles(10, WHITE);
    tft_fillScreen(BLACK);

    /* Fundo */
    tft_fillScreen(BLACK);
    tft_delay(300);

    /* Identificação visual */
    tft_setCursor(5, 20);
    tft_setTextColor(WHITE);
    tft_setTextBackColor(BLACK);
    tft_setTextSize(1);
    tft_printStr((uint8_t *)"TFT DRIVER TEST");

    tft_setCursor(5, 40);
    tft_setTextColor(CYAN);
    tft_printStr((uint8_t *)"Testing shapes and colors");

    tft_delay(300);

    /* Linhas coloridas em diagonal */
    tft_fillScreen(BLACK);
    for (int16_t i = 0; i < 240; i += 20) {
        tft_drawLine(0, 0, i, 319, RED);
        tft_drawLine(0, 319, i, 0, GREEN);
        tft_drawLine(0, i, 239, 319 - i, BLUE);
        tft_drawLine(239, i, 0, 319 - i, YELLOW);
    }
    tft_delay(600);

    /* Linhas horizontais e verticais */
    tft_fillScreen(BLACK);
    for (int16_t y = 0; y < 320; y += 20) {
        tft_drawFastHLine(0, y, 240, CYAN);
    }
    for (int16_t x = 0; x < 240; x += 20) {
        tft_drawFastVLine(x, 0, 320, MAGENTA);
    }
    tft_delay(600);

    /* Retângulos */
    tft_fillScreen(BLACK);
    for (int16_t i = 0; i < 100; i += 12) {
        tft_drawRect(i, i, 240 - 2 * i, 320 - 2 * i, RED);
        tft_drawRect(i + 2, i + 2, 240 - 2 * i - 4, 320 - 2 * i - 4, GREEN);
        tft_drawRect(i + 4, i + 4, 240 - 2 * i - 8, 320 - 2 * i - 8, BLUE);
    }
    tft_delay(600);

    /* Retângulos preenchidos */
    tft_fillScreen(BLACK);
    tft_fillRect(10, 10, 60, 40, RED);
    tft_fillRect(80, 20, 100, 50, GREEN);
    tft_fillRect(30, 80, 150, 60, BLUE);
    tft_fillRect(60, 160, 120, 80, CYAN);
    tft_fillRect(20, 250, 180, 50, YELLOW);
    tft_delay(600);

    /* Cantos arredondados */
    tft_fillScreen(BLACK);
    tft_drawRoundRect(10, 10, 220, 80, 15, RED);
    tft_drawRoundRect(20, 100, 200, 90, 20, GREEN);
    tft_drawRoundRect(30, 200, 180, 100, 25, BLUE);
    tft_fillRoundRect(35, 215, 170, 70, 18, MAGENTA);
    tft_delay(600);

    /* Círculos */
    tft_fillScreen(BLACK);
    tft_drawCircle(60, 60, 30, RED);
    tft_drawCircle(180, 60, 40, GREEN);
    tft_fillCircle(60, 160, 35, BLUE);
    tft_fillCircle(180, 170, 45, YELLOW);
    tft_drawCircleHelper(120, 250, 35, 0x0F, CYAN);
    tft_fillCircleHelper(120, 250, 25, 0x03, 10, MAGENTA);
    tft_delay(600);

    /* Triângulos */
    tft_fillScreen(BLACK);
    tft_drawTriangle(20, 20, 120, 20, 70, 100, RED);
    tft_drawTriangle(140, 20, 220, 100, 100, 120, GREEN);
    tft_fillTriangle(30, 150, 100, 250, 10, 260, BLUE);
    tft_fillTriangle(140, 150, 220, 250, 180, 300, CYAN);
    tft_delay(600);

    /* Mistura de tudo */
    tft_fillScreen(BLACK);
    tft_drawRect(5, 5, 230, 310, WHITE);
    tft_drawLine(0, 0, 239, 319, RED);
    tft_drawLine(239, 0, 0, 319, GREEN);
    tft_drawCircle(120, 160, 70, BLUE);
    tft_fillCircle(120, 160, 35, YELLOW);
    tft_drawRoundRect(20, 40, 200, 240, 20, CYAN);
    tft_fillTriangle(40, 280, 200, 280, 120, 180, MAGENTA);

    tft_setCursor(10, 290);
    tft_setTextColor(WHITE);
    tft_setTextBackColor(BLACK);
    tft_setTextSize(1);
    tft_printStr((uint8_t *)"END OF TEST");
    tft_delay(1000);
    tft_scrollup(10);
    tft_delay(1000);
    tft_scrolldown(10);
    tft_delay(1000);
    tft_invertDisplay(1);
    tft_setRotation(0);
    tft_fillRect(100, 0, 100, 50, WHITE);
    tft_delay(500);
    tft_setRotation(90);
    tft_fillRect(100, 0, 100, 50, RED);
    tft_delay(500);
    tft_setRotation(180);
    tft_fillRect(100, 0, 100, 50, GREEN);
    tft_delay(500);
    tft_setRotation(270);
    tft_fillRect(100, 0, 100, 50, BLUE);
    tft_delay(500);
    tft_invertDisplay(0);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	 /* User can add his own implementation to report the HAL error return state */
	 __disable_irq();
	 while (1)
	 {
	 }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	 /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
