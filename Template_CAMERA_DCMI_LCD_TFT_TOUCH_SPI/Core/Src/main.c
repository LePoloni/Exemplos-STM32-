/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @date 07/07/2026
  * @details Este projeto visa ler uma câmera OV7670 e enviar os frames para o
  * display TFT via SPI.
  * Vou aproveitar a biblioteca usada para configurar a câmera no modo raiz.
  * Testado apenas para formato RGB 565 (16 bits por pixel).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>	//sprintf
#include "touch_spi.h"
#include "tft_spi_dual_driver.h"
#include "fonts.h"
#include "Camera_OV7670.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Verifique/ajuste os defines abaixo no arquivo Camera_OV7670.h
 * #define RGB
 * #define W320H240
 */

/* Nota
 * RAM no STM32F446
 * STM32F446 possui 128 KB de RAM total do sistema, dividida em blocos:
 * SRAM1: 112 KB (Endereço 0x2000 0000)
 * SRAM2: 16 KB (Endereço 0x2001 C000)
 * O buffer precisa ser menor que 112 KB, para não ocupar a SRAM1 inteira.
 */
#ifdef W320H240
	//Habilite o define abaixo para usar p recurso de corte (CROP) do DCMI
	#define CROP
	#ifdef CROP
		//O valor CAPCNT só pode ser um múltiplo de 4
		//(os dois bits menos significativos são forçados a 0)
		//para permitir a transferência correta de dados
		//através do DMA.
		#define CAM_W 200
		#define CAM_H 200
		#define X0 (320 - CAM_W) / 2
		#define Y0 (240 - CAM_H) / 2
		#define BUFFER_SIZE_PIXELS  CAM_W * CAM_H
	#else
		//Não há memória suficiente para armazenar um frame completo
		//Neste caso, o frame é dividido em duas partes de 320x120
		//Aqui o recurso de CROP também é usado, mas são duas capturas por frame
		#define CAM_W 320
		#define CAM_H 120
		#define X0 0
		#define Y0 0
		#define BUFFER_SIZE_PIXELS  38400 	//120 linhas com 320 pixel cada
	#endif
#else
	#define CAM_W 160
	#define CAM_H 120
	#define X0 80
	#define Y0 60
	#define BUFFER_SIZE_PIXELS  CAM_W * CAM_H
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Imagem BMP convertidas para RGB565
extern const unsigned short eletriciade_200x151[30200];

//Frame buffer para captura da câmera
volatile uint16_t cam_fb[BUFFER_SIZE_PIXELS]; //Também funciona

//Flag e contadores de interrupções do periférico DCMI
volatile bool frame_ready = false;
volatile bool half_frame_ready = false;
volatile uint32_t vsync_cnt = 0;
volatile uint32_t frame_cnt = 0;
volatile uint32_t line_cnt = 0;
volatile uint32_t err_cnt = 0;
uint8_t DCMI_estado = 0;
uint32_t frame_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void tft_loop(void);
void touch_loop(void);
void tft_touch_testeToque(void);
void tft_testeDriver(void);
void tft_testeWindow(void);
void camera_tft_capturaFrameSimples(void);
void camera_tft_capturaFrameDuplo(void);
void Meu_DCMI_DMA_HalfCallback(DMA_HandleTypeDef *hdma);
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
	//SPI prescaler = 4 -> 10,5 MHz e 64 -> 656,25 kHz

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
	int32_t i;
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
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  //Inicializa do TFT e o touch
  init();

//  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
//  HAL_Delay(2000);
//  tft_fillScreen(TFT_BLUE);
//  tft_printNewStr(30, WHITE, &mono12x7, 1, "BLUE");
//  HAL_Delay(500);
//  tft_drawImage(20,100,200,151,eletriciade_200x151);
//  HAL_Delay(2000);
//  tft_testeDriver();

  //Orientação paisagem
  tft_setRotation(90);
  tft_fillScreen(BLACK);
  //Plota molduras no LCD
  for(i=0; i<8; i++)
  {	  	  	  	 //x     y     w               h               color
	  tft_drawRect(i*20, i*15, 320-(2*(i*20)), 240-(2*(i*15)), WHITE);
	  HAL_Delay(100);
  }

  //Mensagem inicial
  HAL_UART_Transmit(&huart2, (uint8_t *)"Este programa recebe os pixels de uma camera OV7670 conectada\r\n", 66, 100);
  HAL_UART_Transmit(&huart2, (uint8_t *)"via periférico DCMI e envia via SPI para display LCD TFT\r\n", 66, 100);

  //Verifica o estado do DCMI (apenas para teste)
  DCMI_estado = HAL_DCMI_GetState (&hdcmi);

  //Liga o PWM (clock da câmera)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  htim3.Instance->CCR1 = 2;

  //Reset da câmera
  HAL_GPIO_WritePin(CAMERA_RST_GPIO_Port, CAMERA_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(CAMERA_RST_GPIO_Port, CAMERA_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  //Configura a câmera OV7670 para uso com periférico DCMI
  cam_setup_DCMI(&hi2c2, &huart2);

  //Ajusta o tamanho da tela com CROP
#ifdef W320H240
  /* Nota
   * RGB565, cada pixel ocupa 2 bytes.
   * O hardware do DCMI do STM32F446 possui um registrador interno que lida com
   * pixels de 8 ou 10 bits por ciclo de clock (MCLK).
   * Se a sua câmera envia RGB565 em 2 ciclos de clock de 8 bits
   * (Pixel = Byte 1 + Byte 2), os parâmetros da função HAL_DCMI_ConfigCrop
   * devem considerar o número de capturas de dados da interface (bytes) e
   * não o número final de pixels na horizontal.
   * Como corrigir: alterar a largura do crop na função multiplicando por 2,
   * pois o DCMI precisa "contar" dois bytes por pixel horizontal
   * Utilizar W-1 e H-1
   */
  HAL_DCMI_ConfigCrop(&hdcmi, X0, Y0, (CAM_W * 2) - 1, CAM_H - 1);
  HAL_DCMI_EnableCrop(&hdcmi);
  //Vincula a interrupção de Metade de Transferência do DMA à sua função
  hdcmi.DMA_Handle->XferHalfCpltCallback = Meu_DCMI_DMA_HalfCallback;
#endif

  //Habilita interrupções DCMI
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME | DCMI_IT_ERR);// | DCMI_IT_VSYNC | DCMI_IT_LINE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Escolha o teste desejado
//	  tft_loop();
//	  touch_loop();
//	  colors_loop();
//	  tft_touch_testeToque();	//Fica em looping infinito
//	  tft_testeWindow();

	  //Captura de frames da câmera e envio para o display
#if !defined(W320H240) || defined(CROP)
	  camera_tft_capturaFrameSimples();
#else
	  camera_tft_capturaFrameDuplo();
#endif
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
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, TOUCH_CS_Pin|TFT_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_RST_GPIO_Port, CAMERA_RST_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : TOUCH_CS_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_DC_Pin */
  GPIO_InitStruct.Pin = TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAMERA_RST_Pin */
  GPIO_InitStruct.Pin = CAMERA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERA_RST_GPIO_Port, &GPIO_InitStruct);

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

void tft_testeWindow(void)
{
	uint16_t buffer[320 * 20];
	uint16_t incremento = 20;
	uint16_t y_atual = 0;
	uint16_t COR = 0;
	tft_setRotation(90);
	tft_setAddrWindow(0, 0, 319, 239);
	while(1)
	{
		y_atual = 0;

		while(y_atual < 320)
		{
			//Envio de vermelho
			for(int i=0; i<(320 * incremento); i++)
				buffer[i] = COR;// TFT_RED;
			COR += 1000;
			//tft_setAddrWindow(0, y_atual, 319, y_atual+incremento-1);
			y_atual = y_atual + incremento;
			tft_enviaPixels(buffer, 320 * incremento);

			//Envio de verde
			for(int i=0; i<(320 * incremento); i++)
				buffer[i] = COR;// TFT_GREEN;
			COR += 1000;
			//tft_setAddrWindow(0, y_atual, 319, y_atual+incremento-1);
			y_atual = y_atual + incremento;
			tft_enviaPixels(buffer, 320 * incremento);

			//Envio de azul
			for(int i=0; i<(320 * incremento); i++)
				buffer[i] = COR;// TFT_BLUE;
			COR += 1000;
			//tft_setAddrWindow(0, y_atual, 319, y_atual+incremento-1);
			y_atual = y_atual + incremento;
			tft_enviaPixels(buffer, 320 * incremento);
		}
		//break;
	}
	tft_fimDados();
}

/*
 * brief Faz a captura de um frame da câmera e envia para o display
 */
void camera_tft_capturaFrameSimples(void)
{
//	frame_time = HAL_GetTick();	//Medida do tempo de um frame em ticks (teste)

	//Dispara snapshot do DCMI via DMA
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)cam_fb, BUFFER_SIZE_PIXELS/2);

	//Opação 1 - Aguarda recepção do frame
//	uint32_t time_out = HAL_GetTick() + 150;
//	while(frame_ready == false)
//	{
//		//Segundo medido para configuração atual, são gastos 75 ms/frame
//		//Isso não depende do tamanho
//		if(HAL_GetTick() > time_out)
//			break;
//		if(err_cnt)
//			break;
//	}
//	//Limpa flag de captura
//	frame_ready = false;
//
////	frame_time = HAL_GetTick() - frame_time;	//Medida do tempo de um frame em ticks (teste)

	//Opção 2 - Aguarda apenas uma tempo mínimo para um possível início de preenchimento
	//do buffer. DMA e SPI competem pelo buffer
	HAL_Delay(10);

	//Sinalização de captura de frame
	tft_fillCircle(X0+10, Y0+10, 4, TFT_RED);

//	frame_time = HAL_GetTick();	//Medida do tempo de um frame em ticks (teste)

	//Envia o frame para o LCD como imagem
	tft_drawImage(X0, Y0, CAM_W, CAM_H, (const uint16_t*) cam_fb);

//	frame_time = HAL_GetTick() - frame_time; //Medida do tempo de um frame em ticks (teste)

//	DCMI_estado = HAL_DCMI_GetError(&hdcmi);	//Verifica erros do DCMI (apenas para teste)
}

void camera_tft_capturaFrameDuplo(void)
{
	//1a metade do frame

	//Ajusta o CROP
	HAL_DCMI_ConfigCrop(&hdcmi, X0, Y0, (CAM_W * 2) - 1, CAM_H - 1);
	//Dispara snapshot do DCMI via DMA
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)cam_fb, BUFFER_SIZE_PIXELS/2);

	//Opção 1 - Aguarda recepção de meio frame
	uint32_t time_out = HAL_GetTick() + 150;
	while(half_frame_ready == false)
	{
		//Segundo medido para configuração atual, são gastos 75 ms/frame
		//Isso não depende do tamanho
		if(HAL_GetTick() > time_out)
			break;
		if(err_cnt)
			break;
	}
	//Limpa flag de captura
	half_frame_ready = false;

	//Opção 2 - Aguarda apenas uma tempo mínimo para um possível início de preenchimento
	//do buffer. DMA e SPI competem pelo buffer
	//HAL_Delay(150);

	//Sinalização de captura de frame
	tft_fillCircle(X0+10, Y0+10, 4, TFT_RED);
	//Envia o frame para o LCD como imagem
	tft_drawImage(X0, Y0, CAM_W, CAM_H, (const uint16_t*) cam_fb);

	//2a metade do frame (supondo que a captura da 1a metada já terminou)

	//Ajusta o CROP
	HAL_DCMI_ConfigCrop(&hdcmi, X0, CAM_H, (CAM_W * 2) - 1, (CAM_H * 2) - 1);
	//Dispara snapshot do DCMI via DMA
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)cam_fb, BUFFER_SIZE_PIXELS/2);

	//Opção 1 - Aguarda recepção de meio frame
	time_out = HAL_GetTick() + 150;
	while(half_frame_ready == false)
	{
		//Segundo medido para configuração atual, são gastos 75 ms/frame
		//Isso não depende do tamanho
		if(HAL_GetTick() > time_out)
			break;
		if(err_cnt)
			break;
	}
	//Limpa flag de captura
	half_frame_ready = false;

	//Opção 2 - Aguarda apenas uma tempo mínimo para um possível início de preenchimento
	//do buffer. DMA e SPI competem pelo buffer
	HAL_Delay(150);
	//Sinalização de captura de frame
	tft_fillCircle(X0+10, Y0+10, 4, TFT_RED);
	//Envia o frame para o LCD como imagem
	tft_drawImage(X0, CAM_H, CAM_W, CAM_H, (const uint16_t*) cam_fb);
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
//#ifdef W320H240
//    // Ponteiro apontando para a metade do array (Início do Buffer B)
//    uint16_t *pixel_data = (uint16_t *)&cam_fb[0];//[HALF_BUFFER_PIXELS];
//
//    // Transmite as próximas 10 linhas para o LCD
//    //tft_enviaPixels(pixel_data, HALF_BUFFER_PIXELS);
//    tft_drawImageLittleEndian(0, 0, CAM_W, 60, (const uint16_t*) pixel_data);
//#else
    frame_ready = true;
    frame_cnt++;
//#endif
}

void Meu_DCMI_DMA_HalfCallback(DMA_HandleTypeDef *hdma)
{
	//Uso futuro
	half_frame_ready = true;
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
//void HAL_DCMI_VsyncCallback(DCMI_HandleTypeDef *hdcmi)
{
	vsync_cnt++;
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    line_cnt++;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
    err_cnt++;

    //Verifica se o erro foi de Overrun
    if (__HAL_DCMI_GET_FLAG(hdcmi, DCMI_FLAG_OVRRI) != RESET)
    {
    	// Limpa a flag de estouro pendente
    	__HAL_DCMI_CLEAR_FLAG(hdcmi, DCMI_FLAG_OVRRI);
    }

    //Força o reset do estado do DCMI para que ele aceite novos comandos
    HAL_DCMI_Stop(hdcmi);
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
