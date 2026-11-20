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
#include <tft_spi_dual_driver.h>
#include "touch_spi.h"
#include "fonts.h"
#include "MPU6050_light_STM32.h"
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Imagens BMP convertidas para RGB565
extern const unsigned short eletriciade_200x151[30200];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void tft_loop(void);
void touch_loop(void);
void tft_touch_testeToque(void);
void tft_testeDriver(void);
void tft_draw_mpu_dashboard_layout(void);
void tft_update_mpu_data_display(void);
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

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker-
>Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t timer = 0;
	char str[30];
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  init();	//Inicializa do TFT e o touch
  tft_setRotation(90);
  tft_fillScreen(TFT_BLUE);

  // 1. Inicialização do MPU6050 com o barramento I2C2
  printf("Inicializando MPU6050...\r\n");
  tft_printNewStr(30, WHITE, &mono12x7, 1, " Inicializando MPU6050");
  MPU6050_init(&hi2c2);

  // 2. Verificação da conexão básica
  uint8_t check = readMPU6050(MPU6050_WHO_AM_I);
  printf("MPU6050 WHO_AM_I: 0x%02X\r\n", check);
  sprintf(str, " WHO_AM_I: 0x%02X", check);
  tft_printNewStr(60, WHITE, &mono12x7, 1, str);
  // Travamento da execução caso o sensor não responda corretamente
  // (WHO_AM_I esperado: 0x68 ou 0x98 (sem explicação))
  while(check != 0x68 && check != 0x98) {
	  MPU6050_begin();
	  check = readMPU6050(MPU6050_WHO_AM_I);
	  HAL_Delay(500);
  }

  // 3. Calcula offsets de giroscópio e acelerômetro simultaneamente
  printf("MPU6050 Calculo dos offsets\r\n", check);
  tft_printNewStr(90, WHITE, &mono12x7, 1, " Calculo dos offsets");
  MPU6050_calcOffsets(true, true);	// Versão 1.2.1
  printf("Concluido!\r\n\r\n");
  tft_printNewStr(120, WHITE, &mono12x7, 1, " Concluido!");

  HAL_Delay(2000);

  // 4. Montagem da tela principla em paisagem após a calibração
  tft_draw_mpu_dashboard_layout();

  timer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//Tutorial de aula
		MPU6050_update();
		// Envia os dados a cada 1 segundo
		if((HAL_GetTick() - timer) > 1000)
		{
			// 1. Envio para a Porta Serial (mantido conforme o seu código)
			printf("TEMPERATURA\t%.2f\r\n", MPU6050_getTemp());
			printf("ACELEROMETRO\tX: %+.2f", MPU6050_getAccX());
			printf("\tY: %+.2f", MPU6050_getAccY());
			printf("\tZ: %+.2f\r\n", MPU6050_getAccZ());
			printf("GIROSCOPIO\tX: %+.2f", MPU6050_getGyroX());
			printf("\tY: %+.2f", MPU6050_getGyroY());
			printf("\tZ: %+.2f\r\n", MPU6050_getGyroZ());
			printf("AC ANGULO\tX: %+.2f", MPU6050_getAccAngleX());
			printf("\tY: %+.2f\r\n", MPU6050_getAngleY()); // (ou getAccAngleY)
			printf("ANGULO F COMP\tX: %+.2f", MPU6050_getAngleX());
			printf("\tY: %+.2f", MPU6050_getAngleY());
			printf("\tZ: %+.2f\r\n", MPU6050_getAngleZ());
			printf("=======================================================\r\n");

			// 2. Atualização dos dados na tela LCD em modo paisagem
			tft_update_mpu_data_display();
		}
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

/**
* @brief Retargets the C library printf function to the USART.
* @param None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE BEGIN 0 */

/**
 * @brief Desenha a interface estática do MPU6050 no display em modo paisagem (320x240)
 */
void tft_draw_mpu_dashboard_layout(void)
{
    tft_setRotation(90); // Garante a orientação paisagem (320x240)
    tft_fillScreen(TFT_BLACK);

    // Cabeçalho
    tft_setFont(&mono12x7bold);
    tft_setTextColor(TFT_YELLOW);
    tft_setTextBackColor(TFT_BLACK);
    tft_setCursor(10, 20);
    tft_printStr((uint8_t *)"MPU6050 - DASHBOARD");

    // Linha divisória horizontal
    tft_drawFastHLine(0, 28, 320, TFT_BLUE);

    // Rótulos fixos das seções
    //tft_setFont(&mono12x7);
    tft_setFont(&mono9x7);

    // Coluna / Linha 1: Temperatura
    tft_setCursor(10, 50);
    tft_setTextColor(TFT_CYAN);
    tft_printStr((uint8_t *)"Temp:");

    // Linha divisória horizontal
    tft_drawFastHLine(0, 60, 320, TFT_BLUE);
    // Linha divisória vertical
    tft_drawFastVLine(98, 60, 150, TFT_BLUE);
    tft_drawFastVLine(173, 60, 150, TFT_BLUE);
    tft_drawFastVLine(250, 60, 150, TFT_BLUE);

    // Coluna / Linha 2: Eixos
    tft_setCursor(10, 80);
    tft_setTextColor(TFT_WHITE);
    tft_printStr((uint8_t *)"         X      Y      Z    ");


    // Coluna / Linha 3: Acelerômetro (g)
    tft_setCursor(10, 110);
    tft_setTextColor(TFT_GREEN);
    tft_printStr((uint8_t *)"Acel(g)                     ");

    // Coluna / Linha 4: Giroscópio (deg/s)
    tft_setCursor(10, 140);
    tft_setTextColor(TFT_MAGENTA);
    tft_printStr((uint8_t *)"Gir( /s)                    ");
    tft_drawCircle(60, 134, 2, TFT_MAGENTA);

    // Coluna / Linha 5: Ângulos do Acelerômetro
    tft_setCursor(10, 170);
    tft_setTextColor(TFT_WHITE);
    tft_printStr((uint8_t *)"Ac Ang                      ");

    // Coluna / Linha 6: Ângulos Finais (Filtro Complementar)
    tft_setCursor(10, 200);
    tft_setTextColor(TFT_YELLOW);
    tft_printStr((uint8_t *)"Angulos               F.Comp");

    // Rodapé de status
    tft_drawFastHLine(0, 210, 320, TFT_BLUE);
    tft_setCursor(10, 230);
    tft_setTextColor(TFT_WHITE);
    tft_printStr((uint8_t *)"Status: Executando no STM32");
}

/**
 * @brief Atualiza os valores dinâmicos nos campos do display sem apagar a tela inteira (evita flicker)
 */
void tft_update_mpu_data_display(void)
{
    char str[40];

    //tft_setFont(&mono12x7);
    tft_setFont(&mono9x7);
    tft_setTextColor(TFT_WHITE);
    tft_setTextBackColor(TFT_BLACK);

    // 1. Temperatura (ex: Aprox. x=60, y=50)
    sprintf(str, "%.2f", MPU6050_getTemp());
    tft_setCursor(65, 50);
    tft_printStr_bc((uint8_t *)str);
    sprintf(str, "C", MPU6050_getTemp());
    tft_setCursor(142, 50);
    tft_printStr_bc((uint8_t *)str);
    tft_drawCircle(136, 44, 2, TFT_WHITE);

    // 2. Acelerômetro X, Y, Z
    sprintf(str, "%+.2f", MPU6050_getAccX());
    tft_setCursor(100, 110);
    tft_printStr_bc((uint8_t *)str);

    sprintf(str, "%+.2f", MPU6050_getAccY());
    tft_setCursor(175, 110);
    tft_printStr_bc((uint8_t *)str);

    sprintf(str, "%+.2f", MPU6050_getAccZ());
    tft_setCursor(252, 110);
    tft_printStr_bc((uint8_t *)str);

    // 3. Giroscópio X, Y, Z
    sprintf(str, "%+.1f", MPU6050_getGyroX());
    tft_setCursor(100, 140);
    tft_printStr_bc((uint8_t *)str);

    sprintf(str, "%+.1f", MPU6050_getGyroY());
    tft_setCursor(175, 140);
    tft_printStr_bc((uint8_t *)str);

    sprintf(str, "%+.1f", MPU6050_getGyroZ());
    tft_setCursor(252, 140);
    tft_printStr_bc((uint8_t *)str);

    // 4. Ângulos do Acelerômetro (X e Y)
    sprintf(str, "%+.2f", MPU6050_getAccAngleX());
    tft_setCursor(100, 170);
    tft_printStr_bc((uint8_t *)str);

    sprintf(str, "%+.2f", MPU6050_getAccAngleY());
    tft_setCursor(175, 170);
    tft_printStr_bc((uint8_t *)str);

    // 5. Ângulos Finais do Filtro Complementar (X, Y, Z)
    sprintf(str, "%+.1f", MPU6050_getAngleX());
    tft_setCursor(100, 200);
    tft_printStr_bc((uint8_t *)str);

    sprintf(str, "%+.1f", MPU6050_getAngleY());
    tft_setCursor(175, 200);
    tft_printStr_bc((uint8_t *)str);

    sprintf(str, "%+.1f", MPU6050_getAngleZ());
    tft_setCursor(252, 170);
    tft_printStr_bc((uint8_t *)str);
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
