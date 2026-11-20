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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>	//sprintf
#include <tft_spi_dual_driver.h>
#include "touch_spi.h"
#include "fonts.h"

//Biblioteca de ajuste de funções para o SD Card (criada por terceiro)
#include "fatfs_sd.h"
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
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Imagens BMP convertidas para RGB565
extern const uint8_t playmobil_240x240[57600];

FATFS fs0, fs1;		/* Work area (filesystem object) for logical drives */
FIL fsrc, fdst;		/* File objects */
BYTE buffer[4096];   /* File copy buffer */
FRESULT fr;		/* FatFs function common result code */
UINT br, bw;		/* File read/write count */
uint32_t first;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void tft_loop(void);
void touch_loop(void);
void tft_touch_testeToque(void);
void tft_testeDriver(void);
void LCD_TxBMP(unsigned char* data, unsigned int BitmapStart);
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
	int32_t i;
	int32_t size;
	char texto[40];
	char nome[40];
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
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  init();	//Inicializa do TFT e o touch
  tft_setRotation(0);				//Ajusta a orientação da tela
  tft_fillScreen(BLACK);		//Preenche a tela em uma só cor
  //Mensagem de texto
  tft_printNewStr(20, YELLOW, &mono12x7bold, 1,
  							(uint8_t *)"Teste SD Card");

  //Colar os exemplos aqui-------------------------------------------

  //=================================================================
  //Exemplo de CRIAÇÃO e ESCRITA ====================================
  //=================================================================
//  size = sprintf(texto, "Exemplo de CRIACAO e ESCRITA\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  //Prepara a área de trabalho para o FatFs
//  fr = f_mount(&fs0, "", 0);
//  if(fr != FR_OK)
//  {
//  	size = sprintf(texto, "f_mount error: %d\r\n", (int)fr);
//  	HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  	tft_printNewStr(100, RED, &mono12x7bold, 1,
//  			(uint8_t *)"Falha!!!");
//  	return 1;
//  }
//
//  //Cria um arquivo
//  fr = f_open(&fsrc, "newfile.txt", FA_WRITE | FA_CREATE_ALWAYS);	/* Cria o arquivo */
//  if (fr == FR_OK)
//  {
//  	f_write(&fsrc, "Funcionou!\r\n", 11, &bw);	/* Escreve dados no arquivo */
//  	fr = f_close(&fsrc);												/* Fecha o arquivo */
//
//  	if (fr == FR_OK)
//  	{
//  		size = sprintf(texto, "Arquivo criado com sucesso\r\n");
//  		HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  		tft_printNewStr(100, GREEN, &mono12x7bold, 1,
//  				(uint8_t *)"Sucesso!!!");
//  	}
//  }
//  else
//  {
//  	size = sprintf(texto, "f_open error: %d\r\n", (int)fr);
//  	HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  	tft_printNewStr(100, RED, &mono12x7bold, 1,
//  			(uint8_t *)"Falha!!!");
//  	return 1;
//  }

	//=================================================================
	//Exemplo de LEITURA ==============================================
	//=================================================================
//  size = sprintf(texto, "Exemplo de LEITURA\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  //Prepara a área de trabalho para o FatFs
//  fr = f_mount(&fs0, "", 0);
//	if(fr != FR_OK)
//	{
//		size = sprintf(texto, "f_mount error: %d\r\n", (int)fr);
//		HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//		tft_printNewStr(100, RED, &mono12x7bold, 1,
//					   	 	 (uint8_t *)"Falha!!!");
//		return 1;
//	}
//
//	//Lê um arquivo
//	buffer[0] = 0;
//	strcpy(nome, "newfile.txt");
//	//strcpy(nome, "texto/Teste.txt");
//	fr = f_open(&fsrc, nome, FA_READ);
//  if(fr == FR_OK)
//  {
//  	//Entra em um loop de leitura de dados do arquivo e escrita na serial e LCD
//  	//até alcançar o fim do arquivo
//  	do
//  	{
//  		//Lê um bloco de dados do arquivo. Lê o tanto que cabe no buffer,
//  		//incluindo espaços e carcteres nulos
//  		fr = f_read(&fsrc, 				/* [IN] File object */
//  				buffer, 							/* [OUT] Buffer to store read data */
//					sizeof(buffer) - 1,		/* [IN] Number of bytes to read */
//					(UINT *)&br);					/* [OUT] Number of bytes read */
//
//  		//Se houver algum erro na leitura, retorna o erro ao usuário
//  		if(fr != FR_OK)
//  		{
//  			size = sprintf(texto, "f_read error: %d\r\n", (int)fr);
//  			HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  			tft_printNewStr(100, RED, &mono12x7bold, 1,
//  					(uint8_t *)"Falha!!!");
//  			break;
//  		}
//
//  		//Finaliza a string com um nulo
//  		buffer[br] = 0;
//
//  		//Escreve na serial o último pedaço de arquivo lido
//  		size = br;
//  		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 1000);
//
//  		//Escreve no LCD os últimos 16 caracteres lidos
//  		tft_printNewStr(150, WHITE, &mono12x7bold, 1,
//  				(uint8_t *)"Leitura (<=16):");
//  		if(br>16)	buffer[16] = '\0';
//  		else			buffer[br] = '\0';
//  		tft_printNewStr(200, WHITE, &mono12x7bold, 1,
//  				(uint8_t *)buffer);
//  	}
//  	while(br == sizeof(buffer) - 1);
//  	//Repete até a quantidade lida for menor que o buffer, indicação que acabou o arquivo (último pacote)
//  }
//  if (fr == FR_OK)
//  {
//  	size = sprintf(texto, "\r\nArquivo lido com sucesso\r\n");
//  	HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  	tft_printNewStr(100, GREEN, &mono12x7bold, 1,
//  			(uint8_t *)"Sucesso!!!");
//  }

  //=================================================================
  //Exemplo de LEITURA de arquivo BINÁRIO============================
  //=================================================================
//  size = sprintf(texto, "Exemplo de LEITURA de arquivo BINARIO\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  //Prepara a área de trabalho para o FatFs
//  fr = f_mount(&fs0, "", 0);
//  if(fr != FR_OK)
//  {
//  	size = sprintf(texto, "f_mount error: %d\r\n", (int)fr);
//  	HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  	tft_printNewStr(100, RED, &mono12x7bold, 1,
//  			(uint8_t *)"Falha!!!");
//  	return 1;
//  }
//
//  //Lê um arquivo
//  buffer[0] = 0;
//  strcpy(nome, "newfile.txt");
//  //strcpy(nome, "texto/Teste.txt");
//  fr = f_open(&fsrc, nome, FA_READ);
//  if(fr == FR_OK)
//  {
//  	//Entra em um loop de leitura de dados do arquivo e escrita na serial e LCD
//  	//até alcançar o fim do arquivo
//  	do
//  	{
//  		//Lê um bloco de dados do arquivo. Lê o tanto que cabe no buffer,
//  		//incluindo espaços e carcteres nulos
//  		fr = f_read(&fsrc, 				/* [IN] File object */
//  				buffer, 							/* [OUT] Buffer to store read data */
//					sizeof(buffer) - 1,		/* [IN] Number of bytes to read */
//					(UINT *)&br);					/* [OUT] Number of bytes read */
//
//  		//Se houver algum erro na leitura, retorna o erro ao usuário
//  		if(fr != FR_OK)
//  		{
//  			size = sprintf(texto, "f_read error: %d\r\n", (int)fr);
//  			HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  			tft_printNewStr(100, RED, &mono12x7bold, 1,
//  					(uint8_t *)"Falha!!!");
//  			break;
//  		}
//
//  		//Finaliza a string com um nulo
//  		buffer[br] = 0;
//
//  		//Escreve na serial o último pedaço de arquivo lido
//  		for (i = 0; i < br; i++)
//  		{
//  			size = sprintf(texto, "%02X ",buffer[i]);
//  			HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 1000);
//  		}
//
//
//  		//Escreve no LCD os últimos 16 caracteres lidos
//  		tft_printNewStr(150, WHITE, &mono12x7bold, 1,
//  				(uint8_t *)"Leitura Bi (<=8):");
//  		i = 0;
//  		size = sprintf(texto, "%02X",buffer[i++]);
//  		//1os caracteres
//  		tft_printNewStr(200, WHITE, &mono12x7bold, 1,
//  				(uint8_t *)texto);
//  		//Demais caracteres
//  		while(i < 8 && i < br)
//  		{
//  			size = sprintf(texto, "%02X",buffer[i++]);
//  			tft_printStr((uint8_t *)texto);
//  		}
//  	}
//  	while(br == sizeof(buffer) - 1);
//  	//Repete até a quantidade lida for menor que o buffer, indicação que acabou o arquivo (último pacote)
//  }
//  if (fr == FR_OK)
//  {
//  	size = sprintf(texto, "\r\nArquivo lido com sucesso\r\n");
//  	HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
//
//  	tft_printNewStr(100, GREEN, &mono12x7bold, 1,
//  			(uint8_t *)"Sucesso!!!");
//  }

  //=================================================================
  //Exemplo de LEITURA de BITMAP e escrita no LCD====================
  //=================================================================
  size = sprintf(texto, "Exemplo de LEITURA de BITMAP e escrita no LCD\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);

  //Prepara a área de trabalho para o FatFs
  fr = f_mount(&fs0, "", 0);
  if(fr != FR_OK)
  {
  	size = sprintf(texto, "f_mount error: %d\r\n", (int)fr);
  	HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);

  	tft_printNewStr(100, RED, &mono12x7bold, 1,
  			(uint8_t *)"Falha!!!");
  	return 1;
  }

  while(1)
  {
  	//========= 1a imagem ===========================================

  	//Primeiro bloco de dados, onde está o cabeçalho do BMP
  	first = 1;

  	//"Limpa" o buffer
  	buffer[0] = 0;

  	//Define o nome do arquivo BMP 24 bits a ser aberto
  	strcpy(nome, "Imagens/Homerinv.bmp");
  	//strcpy(buffer, "Imagens/Lisa.bmp");

  	//Abre o arquivo como leitura
  	fr = f_open(&fsrc, nome, FA_READ);

  	//Se teve sucesso
  	if(fr == FR_OK)
  	{
  		//Loop de leitura de setores (512 bytes), lê até acabar o arquivo
  		do
  		{
  			//Lê um setor do arquivo e armazena no buffer temporário
  			fr = f_read(&fsrc, 		/* [IN] File object */
  					buffer,			/* [OUT] Buffer to store read data */
						512,				/* [IN] Number of bytes to read */
						(UINT *)&br);		/* [OUT] Number of bytes read */

  			//Se ocorrer algum erro mostra na UART e aborta
  			if(fr != FR_OK)
  			{
  				size = sprintf(texto, "f_read error: %d\r\n", (int)fr);
  				HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
  			}

  			//Termina o bloco lido com um caractere nulo (sem efeito para o LCD)
  			buffer[br] = 0;

  			//Envia byte a byte em hexa pela UART
  			//				for (i = 0; i < br; i++)
  			//				{
  			//					size = sprintf(texto, "%02X ",buffer[i]);
  			//					HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 1000);
  			//				}

  			//Envia o setor para o LCD
  			LCD_TxBMP((unsigned char*)buffer, first);

  			//Limpa a flag de primeiro setor (cabeçalho nos primeiros 54 bytes)
  			first = 0;

  		}
  		while(br == 512);
  		//Repete até a quantidade lida for menor que o buffer, indicação que acabou o arquivo (último pacote)

  		//Fecha o arquivo (não é necessário para leitura, mas recomendado para compatibilidade futura)
  		fr = f_close(&fsrc);
  		//Se teve sucesso
  		if (fr == FR_OK)
  		{
  			size = sprintf(texto, "\r\nArquivo fechado com sucesso\r\n");
  			HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
  		}
  	}

  	if (fr == FR_OK)
  	{
  		size = sprintf(texto, "\r\nArquivo 1 lido com sucesso\r\n");
  		HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
  	}

  	HAL_Delay(3000);

  	tft_printNewStr(200, YELLOW, &mono12x7bold, 1,
  			(uint8_t *)" Escrita no LCD ");
  	tft_printNewStr(220, YELLOW, &mono12x7bold, 1,
  			(uint8_t *)" apos acesso ao ");
  	tft_printNewStr(240, YELLOW, &mono12x7bold, 1,
  			(uint8_t *)" SD Card ");

  	HAL_Delay(2000);

  	tft_fillScreen(BLUE);

  	HAL_Delay(2000);

  	//========= 2a imagem ===========================================

  	//Primeiro bloco de dados, onde está o cabeçalho do BMP
  	first = 1;

  	//"Limpa" o buffer
  	buffer[0] = 0;

  	//Define o nome do arquivo BMP 24 bits a ser aberto
  	strcpy(nome, "Imagens/Homer2in.bmp");
  	//strcpy(buffer, "Imagens/Lisa.bmp");

  	//Abre o arquivo como leitura
  	fr = f_open(&fsrc, nome, FA_READ);

  	//Se teve sucesso
  	if(fr == FR_OK)
  	{
  		//Loop de leitura de setores (512 bytes), lê até acabar o arquivo
  		do
  		{
  			//Lê um setor do arquivo e armazena no buffer temporário
  			fr = f_read(&fsrc, 					/* [IN] File object */
  					buffer,					/* [OUT] Buffer to store read data */
						512,						/* [IN] Number of bytes to read */
						(UINT *)&br);		/* [OUT] Number of bytes read */

  			//Se ocorrer algum erro mostra na UART e aborta
  			if(fr != FR_OK)
  			{
  				size = sprintf(texto, "f_mount error: %d\r\n", (int)fr);
  				HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
  			}

  			//Termina o bloco lido com um caractere nulo (sem efeito para o LCD)
  			buffer[br] = 0;

  			//Envia byte a byte em hexa pela UART
  			//				for (i = 0; i < br; i++)
  			//				{
  			//					size = sprintf(texto, "%02X ",buffer[i]);
  			//					HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 1000);
  			//				}

  			//Envia o setor para o LCD
  			LCD_TxBMP((unsigned char*)buffer, first);

  			//Limpa a flag de primeiro setor (cabeçalho nos primeiros 54 bytes)
  			first = 0;

  		}
  		while(br == 512);
  		//Repete até a quantidade lida for menor que o buffer, indicação que acabou o arquivo (último pacote)

  		//Fecha o arquivo (não é necessário para leitura, mas recomendado para compatibilidade futura)
  		fr = f_close(&fsrc);
  		//Se teve sucesso
  		if (fr == FR_OK)
  		{
  			size = sprintf(texto, "\r\nArquivo fechado com sucesso\r\n");
  			HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
  		}
  	}

  	if (fr == FR_OK)
  	{
  		size = sprintf(texto, "\r\nArquivo 2 lido com sucesso\r\n");
  		HAL_UART_Transmit(&huart2, (uint8_t *)texto, size, 100);
  	}

  	HAL_Delay(3000);

  	tft_printNewStr(200, YELLOW, &mono12x7bold, 1,
  			(uint8_t *)" Escrita no LCD ");
  	tft_printNewStr(220, YELLOW, &mono12x7bold, 1,
  			(uint8_t *)" apos acesso ao ");
  	tft_printNewStr(240, YELLOW, &mono12x7bold, 1,
  			(uint8_t *)" SD Card ");

  	HAL_Delay(2000);

  	tft_fillScreen(BLACK);

  	HAL_Delay(2000);
  }

  //-----------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, TFT_CS_Pin|SD_CS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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

	tft_drawImage(0,0,240,240,(const uint16_t*)playmobil_240x240);
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
			y = (320 - 1) - y;
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

//Envia uma string como um bitmp para o LCD TFT
//Recebe: String com 512 bytes (1 setor de arquivo) que fazem parte da imagem
//				Flag que sinaliza se é o primeiro setor (onde está o cabeçalho)
void LCD_TxBMP(unsigned char* data, unsigned int BitmapStart)
{
	static unsigned int tamanho = 0;
	static unsigned char sobrou[3] = {0,0,0};
	unsigned int i = 0;
	unsigned int setor = 512;
	unsigned short int cor;
	unsigned char tam[11];
	//Leandro (22/09/2020) - Ajuste para ler imagens com dimensões maiores que um byte (320x240)
//	unsigned char altura, x = 0, y = 127, bits_por_pixel;
	unsigned char bits_por_pixel;
//	unsigned int altura, x = 0, y = 239;	//Formato "paisagem"
	unsigned int altura, x = 0, y = 319;	//Formato "retrato"
//	const unsigned int lim_altura = 240, lim_largura = 320;	//Formato "paisagem" - enviar comando setRotation(1);
	const unsigned int lim_altura = 320, lim_largura = 240;	//Formato "retrato"	- enviar comando setRotation(0);
//	static unsigned char largura = 0, erro_bits = 0, bytes_extras = 0, pixels_por_linha = 0;
	static unsigned int largura = 0, pixels_por_linha = 0;
	static unsigned char erro_bits = 0, bytes_extras = 0;

	//Se é o primeiro setor do arquivo, possui o cabeçalho
	if(BitmapStart)
	{
		BitmapStart = 0;

		//Reseta variável estática
		pixels_por_linha = 0;

		//Pula o cabeçalho
		i = 54;
		//Lê o tamanho da área de dados do arquivo em bytes
		tamanho = data[0x22] + (unsigned int)(data[0x23]<<8) + (unsigned int)(data[0x24]<<16) + (unsigned int)(data[0x25]<<24);
		//Leandro (22/09/2020) - Ajuste para ler imagens com dimensões maiores que um byte (320x240)
//		//Leandro (01/09/2019) - Lê a largura e a altura da imagem para definir o tamanho da janela
//		largura = data[18];
//		altura = data[22];
		largura = data[18] + (unsigned int)(data[19]<<8);
		altura = data[22] + (unsigned int)(data[23]<<8);
		//Configura a janela
		tft_setAddrWindow(x, y-altura+1, x+largura-1, y);
		//Envia para o LCD sinalização de início de envio de dados
		tft_inicioDados();
		//Verifica se existirão bytes extras no arquivo em função da largura da imagem
		//Obervação: Existe uma restrição de que cada linha deva ter N bytes, sendo N um número
		//divisível por 4. Caso contrário, o BMP deve ser preenchido com bytes não válidos. Por
		//exemplo, se a imagem tem 1 x 100 pixels em 24 bits/pixel, o BMP teria 3 bytes válidos em
		//cada linha e mais 1 byte que não tem qualquer significado.
		switch((largura*3)%4)
		{
			case 1: bytes_extras = 3; break;
			case 2: bytes_extras = 2; break;
			case 3: bytes_extras = 1; break;
			default: bytes_extras = 0; break;
		}

		//Lê a quantidade de bits por pixel (neste caso é aceito apenas 24 bits por pixel)
		bits_por_pixel = data[28];
		//Testa a quatidade de bits
		//Leandro (22/09/2020) - Ajuste para ler imagens com dimensões maiores que um byte (320x240)
//		if((bits_por_pixel != 24) || (largura > 128) || (altura > 128))
		if((bits_por_pixel != 24) || (largura > lim_largura) || (altura > lim_altura))
			erro_bits = 1;
		else
			erro_bits = 0;
	}

	//Se houver erro na quantidade de bits retorna e não envia para o LCD
	if(erro_bits)
	{
		return;
	}

	//Envia os pixels enquanto não acabar o setor ou o Bitmap
	while((i <= (512-3)) && (tamanho >= 3))	//24 bits por pixels
	{
		//Se completou uma linha
		if(pixels_por_linha == largura)
		{
			//Zera o contador
			pixels_por_linha = 0;
			//Verifica se tem bytes nulos para ignorar
			if(bytes_extras >= sobrou[0])
			{
				//Desconta os bytes_extras-sobrou[0] do tamanho do setor
				tamanho -= (bytes_extras-sobrou[0]);
				//Incrementa a posição do byte a ser lido do setor
				i += (bytes_extras-sobrou[0]);
				//Atualiza o valor da sobra
				sobrou[0] = 0;
				//Verifica se não cabe mais nenhum pixel, encerra o loop
				if((i>(512-3)) || (tamanho < 3))
					break;
			}
			else
			{
				//Atualiza o valor da sobra
				sobrou[0] -= bytes_extras;
			}
			//break;
			if(tamanho<3)
				break;
		}

		if(sobrou[0] == 0)			//Tamanho -= 3
		{
			//((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
			//Seguencia BGR (24 bits) --> RGB (565)
			cor = (data[i] >> 3) | ((data[i+1] & 0xFC) << 3) | ((data[i+2] & 0xF8) << 8);
			i += 3;
			tamanho -= 3;
		}
		else if(sobrou[0] == 1)	//Tamanho -= 2
		{
			//Sobrou a cor Azul
			cor = (sobrou[2] >> 3) | ((data[i] & 0xFC) << 3) | ((data[i+1] & 0xF8) << 8);
			i += 2;
			tamanho -= 2;
		}
		else if(sobrou[0] == 2)	//Tamanho -= 1
		{
			//Sobrou a cor Azul e Verde
			cor = (sobrou[1] >> 3) | ((sobrou[2] & 0xFC) << 3) | ((data[i] & 0xF8) << 8);
			i += 1;
			tamanho -= 1;
		}
		else
		{
			i = 512;
			setor = 0;
			tamanho = 0;
			break;
		}
		//Envia pixel 565 para o LCD
		tft_desenhaPixel(cor);

		sobrou[0] = 0;	//Sobra algum byte apenas no final do setor (i>= 510)

		//Incrementa o número de pixels enviados por linha e testa
		pixels_por_linha++;
	}
	//Se ainda não acabou o arquivo
	if(tamanho >= 3)
	{
		//Salva o número de bytes que sobraram para formar um pixel
		sobrou[0] = 512 - i;
		//Completa os 512 bytes do setor
		tamanho -= sobrou[0];
		//Salva o penúltimo byte
		sobrou[1] = data[510];
		//Salva o último byte
		sobrou[2] = data[511];
	}
	else
	{
		//Envia para o LCD sinalização de fim de envio de dados
		tft_fimDados();
	}
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
