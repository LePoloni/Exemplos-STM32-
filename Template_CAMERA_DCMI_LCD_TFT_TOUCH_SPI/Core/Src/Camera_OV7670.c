/**
 ******************************************************************************
 * @file        Camera_OV7670.c
 * @brief       Biblioteca controle de câmera OV7670 via kit ST NUCLEO-F446RE
 ******************************************************************************
 * @attention
 *
 * <h2><center>Desenvolvida para fins acadêmicos</center></h2>
 *
 * @author      Leandro Poloni Dantas
 * @details     Compatível com câmera OV7670 via GPIOs e DCMI
 * @date        Julho de 2020
 * @details     Compatível com imagens RGB565 e YUV422 320x240 e 160x120
 *
 *****************************************************************************
 * @details	Notas:
 * 			Adaptação do programa utilizado para envio de uma imagem da
 * 			câmera OV7670 do Arduino UNO para um Arduino MEGA com um
 * 			LCD TFT ILI9340 paralelo com touch.
 * 			Ele foi baseado no exemplo de Siarhei Charkes (2015) que enviava
 * 			uma imagem da câmera OV7670 para um software no PC via USB.
 *
 * 			Para configuração de modo RGB565 e YUV, altere o define RGB
 * 			no arquivo Camera_OV7670.h
 * 			Para configuração da resolução 320x240 e 160x120, altere o
 * 			define W320H240	no arquivo Camera_OV7670.h
 * 			Para uso de interface com software via porta serial UART,
 * 			altere o define SOFTWARE no arquivo Camera_OV7670.h
 * 			Na configuração do pino pino de reset, defina o pino
 * 			com o nome indicado:
 * 			Reset: 	TFT_RES
 * 			Usar porta I2C para configuração da câmera e ajustes
 * ***************************************************************************
 * @details	Modificações:
 *
 * @date	01/07/2020
 * @verion	1.0
 * @details	Apenas enviava o frame lido da câmera pela porta serial
 * 			(1 Mbps) para o aplicativo OV7670.
 * 			Foi preciso corrigir alguns bugs no sincronismo do frame.
 *
 * @date	01/09/2020
 * @version	1.1
 * @details	Alguns ajustes em comentários e limpeza do código.
 *
 * @date	13/04/2025
 * @version	1.2
 * @details	Correção em chamada de função da biblioteca tft
 * 			(acréscimo de prefixo "tft_").
 * 			Todos as funções da biblioteca receberam o prefixo "cam".
 *
 * @date	11/07/2026
 * @version	2.0
 * @details	Foi totalmente revisada e compatibilizada com o periférico
 * 			DCMI do STM32 para recepção via DMA de frames.
 * 			Comentários compatibilizados com o padrão Doxygen.
 *
 *****************************************************************************
 */

/*
 * *******************************************************************
 * Includes **********************************************************
 * *******************************************************************
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Camera_OV7670.h"
#include "tft_spi_dual_driver.h"

/*
 * *******************************************************************
 * Variáveis e Constantes ********************************************
 * *******************************************************************
 */

I2C_HandleTypeDef *HI2C;
UART_HandleTypeDef *HUART;

/*
 * @brief Configuração de resolução da câmera
 */
const struct regval_list qvga_ov7670[] = {
#ifdef W320H240
  { REG_COM14, 0x19 },  //320x240
  //{ 0x72, 0x11 },       //SCALING DCWCTR 11=H/2,V/2 22=H/4,V/4 33=H/8,V/8
  { 0x72, 0x11 },
  //{ 0x73, 0xf1 },       //SCALING PCLK_DIV 0=div1 1=div2 2=div4 3=div8 4=div16 (Trabalha junto com COM14)
  { 0x73, 0xf1 },
#else
  { REG_COM14, 0x1A },  //160x120  Divide o PCLK por 4
  { 0x72, 0x22 },
  { 0x73, 0xf2 },
#endif
//  //{ 0x72, 0x11 },       //SCALING DCWCTR 11=H/2,V/2 22=H/4,V/4 33=H/8,V/8
//  { 0x72, 0x11 },
//  //{ 0x73, 0xf1 },       //SCALING PCLK_DIV 0=div1 1=div2 2=div4 3=div8 4=div16 (Trabalha junto com COM14)
//  { 0x73, 0xf1 },

  { REG_HSTART, 0x16 },
  { REG_HSTOP, 0x04 },
  { REG_HREF, 0xa4 },
  { REG_VSTART, 0x02 },
  { REG_VSTOP, 0x7a },
  { REG_VREF, 0x0a },


/*  { REG_HSTART, 0x16 },
  { REG_HSTOP, 0x04 },
  { REG_HREF, 0x24 },
  { REG_VSTART, 0x02 },
  { REG_VSTOP, 0x7a },
  { REG_VREF, 0x0a },*/
  { 0xff, 0xff }, /* END MARKER */
};

/*
 * Configuração de cores YUV422
 */
const struct regval_list yuv422_ov7670[] = {
  { REG_COM7, 0x0 },  /* Selects YUV mode */
  { REG_RGB444, 0 },  /* No RGB444 please */
  { REG_COM1, 0 },
  { REG_COM15, COM15_R00FF },
  { REG_COM9, 0x6A }, /* 128x gain ceiling; 0x8 is reserved bit */
  { 0x4f, 0x80 },   /* "matrix coefficient 1" */
  { 0x50, 0x80 },   /* "matrix coefficient 2" */
  { 0x51, 0 },    /* vb */
  { 0x52, 0x22 },   /* "matrix coefficient 4" */
  { 0x53, 0x5e },   /* "matrix coefficient 5" */
  { 0x54, 0x80 },   /* "matrix coefficient 6" */
  { REG_COM13, COM13_UVSAT },
  { 0xff, 0xff },   /* END MARKER */
};

/*
 * Configuração de cores RGB565
 */
const struct regval_list rgb565_ov7670[] = {
  { REG_COM7, COM7_RGB },  /* Selects RGB mode */
  { REG_RGB444, 0 },  /* No RGB444 please */
  { REG_COM1, 0 },
  { REG_COM15, COM15_RGB565 },  /*Outrange 00-FF, RGB565  opção: |0Xc0*/
  { REG_COM9, 0x6A }, /* 128x gain ceiling; 0x8 is reserved bit */
  { 0x4f, 0x80 },   /* "matrix coefficient 1" */
  { 0x50, 0x80 },   /* "matrix coefficient 2" */
  { 0x51, 0 },    /* vb */
  { 0x52, 0x22 },   /* "matrix coefficient 4" */
  { 0x53, 0x5e },   /* "matrix coefficient 5" */
  { 0x54, 0x80 },   /* "matrix coefficient 6" */
  { REG_COM13, COM13_UVSAT },
  { 0xff, 0xff },   /* END MARKER */
};

/*
 * Configuração padrão
 */
const struct regval_list ov7670_default_regs[] = {//from the linux driver
  { REG_COM7, COM7_RESET },
  { REG_TSLB, 0x04 }, /* OV */
  { REG_COM7, 0 },  /* VGA */
  /*
  * Set the hardware window.  These values from OV don't entirely
  * make sense - hstop is less than hstart.  But they work...
  */
  { REG_HSTART, 0x13 }, { REG_HSTOP, 0x01 },
  { REG_HREF, 0xb6 }, { REG_VSTART, 0x02 },
  { REG_VSTOP, 0x7a }, { REG_VREF, 0x0a },

  { REG_COM3, 0 }, { REG_COM14, 0 },
  /* Mystery scaling numbers */
  { 0x70, 0x3a }, { 0x71, 0x35 },
  { 0x72, 0x11 }, { 0x73, 0xf0 },
  { 0xa2,/* 0x02 changed to 1*/1 }, { REG_COM10, 0x0 },
  /* Gamma curve values */
  { 0x7a, 0x20 }, { 0x7b, 0x10 },
  { 0x7c, 0x1e }, { 0x7d, 0x35 },
  { 0x7e, 0x5a }, { 0x7f, 0x69 },
  { 0x80, 0x76 }, { 0x81, 0x80 },
  { 0x82, 0x88 }, { 0x83, 0x8f },
  { 0x84, 0x96 }, { 0x85, 0xa3 },
  { 0x86, 0xaf }, { 0x87, 0xc4 },
  { 0x88, 0xd7 }, { 0x89, 0xe8 },
  /* AGC and AEC parameters.  Note we start by disabling those features,
  then turn them only after tweaking the values. */
  { REG_COM8, COM8_FASTAEC | COM8_AECSTEP },
  { REG_GAIN, 0 }, { REG_AECH, 0 },
  { REG_COM4, 0x40 }, /* magic reserved bit */
  { REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
  { REG_BD50MAX, 0x05 }, { REG_BD60MAX, 0x07 },
  { REG_AEW, 0x95 }, { REG_AEB, 0x33 },
  { REG_VPT, 0xe3 }, { REG_HAECC1, 0x78 },
  { REG_HAECC2, 0x68 }, { 0xa1, 0x03 }, /* magic */
  { REG_HAECC3, 0xd8 }, { REG_HAECC4, 0xd8 },
  { REG_HAECC5, 0xf0 }, { REG_HAECC6, 0x90 },
  { REG_HAECC7, 0x94 },
  { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC },
  { 0x30, 0 }, { 0x31, 0 },//disable some delays
  /* Almost all of these are magic "reserved" values.  */
  { REG_COM5, 0x61 }, { REG_COM6, 0x4b },
  { 0x16, 0x02 }, { REG_MVFP, 0x07 },
  { 0x21, 0x02 }, { 0x22, 0x91 },
  { 0x29, 0x07 }, { 0x33, 0x0b },
  { 0x35, 0x0b }, { 0x37, 0x1d },
  { 0x38, 0x71 }, { 0x39, 0x2a },
  { REG_COM12, 0x78 }, { 0x4d, 0x40 },
  { 0x4e, 0x20 }, { REG_GFIX, 0 },
  /*{0x6b, 0x4a},*/{ 0x74, 0x10 },
  { 0x8d, 0x4f }, { 0x8e, 0 },
  { 0x8f, 0 }, { 0x90, 0 },
  { 0x91, 0 }, { 0x96, 0 },
  { 0x9a, 0 }, { 0xb0, 0x84 },
  { 0xb1, 0x0c }, { 0xb2, 0x0e },
  { 0xb3, 0x82 }, { 0xb8, 0x0a },

  /* More reserved magic, some of which tweaks white balance */
  { 0x43, 0x0a }, { 0x44, 0xf0 },
  { 0x45, 0x34 }, { 0x46, 0x58 },
  { 0x47, 0x28 }, { 0x48, 0x3a },
  { 0x59, 0x88 }, { 0x5a, 0x88 },
  { 0x5b, 0x44 }, { 0x5c, 0x67 },
  { 0x5d, 0x49 }, { 0x5e, 0x0e },
  { 0x6c, 0x0a }, { 0x6d, 0x55 },
  { 0x6e, 0x11 }, { 0x6f, 0x9e }, /* it was 0x9F "9e for advance AWB" */
  { 0x6a, 0x40 }, { REG_BLUE, 0x40 },
  { REG_RED, 0x60 },
  { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC | COM8_AWB },

  /* Matrix coefficients */
  { 0x4f, 0x80 }, { 0x50, 0x80 },
  { 0x51, 0 },    { 0x52, 0x22 },
  { 0x53, 0x5e }, { 0x54, 0x80 },
  { 0x58, 0x9e },

  { REG_COM16, COM16_AWBGAIN }, { REG_EDGE, 0 },
  { 0x75, 0x05 }, { REG_REG76, 0xe1 },
  { 0x4c, 0 },     { 0x77, 0x01 },
  { REG_COM13, /*0xc3*/0x48 }, { 0x4b, 0x09 },
  { 0xc9, 0x60 },   /*{REG_COM16, 0x38},*/

  { 0x55, 0x00 },   //Poloni 08-06-20 Brilho (não existia no setup), qualquer valor acima de 0 já é intenso
  //{ 0x56, 0x40 },   //Poloni 08-06-20 Contraste
  { 0x56, 0x40 },   //Poloni 08-06-20 Contraste

  { 0x34, 0x11 }, { REG_COM11, COM11_EXP | COM11_HZAUTO },
  { 0xa4, 0x82/*Was 0x88*/ }, { 0x96, 0 },
  { 0x97, 0x30 }, { 0x98, 0x20 },
  { 0x99, 0x30 }, { 0x9a, 0x84 },
  { 0x9b, 0x29 }, { 0x9c, 0x03 },
  { 0x9d, 0x4c }, { 0x9e, 0x3f },
  { 0x78, 0x04 },

  /* Extra-weird stuff.  Some sort of multiplexor register */
  { 0x79, 0x01 }, { 0xc8, 0xf0 },
  { 0x79, 0x0f }, { 0xc8, 0x00 },
  { 0x79, 0x10 }, { 0xc8, 0x7e },
  { 0x79, 0x0a }, { 0xc8, 0x80 },
  { 0x79, 0x0b }, { 0xc8, 0x01 },
  { 0x79, 0x0c }, { 0xc8, 0x0f },
  { 0x79, 0x0d }, { 0xc8, 0x20 },
  { 0x79, 0x09 }, { 0xc8, 0x80 },
  { 0x79, 0x02 }, { 0xc8, 0xc0 },
  { 0x79, 0x03 }, { 0xc8, 0x40 },
  { 0x79, 0x05 }, { 0xc8, 0x30 },
  { 0x79, 0x26 },
  { 0xff, 0xff }, /* END MARKER */
};

/*
 * Configuração para DCMI
 */
const struct regval_list OV7670_reg[] = {
  /* Color mode related */
  {0x12, 0x14},   // QVGA, RGB
  {0x8C, 0x00},   // RGB444 Disable
  {0x40, 0x10 + 0xc0},   // RGB565, 00 - FF
  {0x3A, 0x04 + 8},   // UYVY (why?)
  {0x3D, 0x80 + 0x00},   // gamma enable, UV auto adjust, UYVY
  {0xB0, 0x84}, // important

  /* clock related */
  {0x0C, 0x04},  // DCW enable
  {0x3E, 0x19},  // manual scaling, pclk/=2 (320x240), pclk/=4 (160x120)
  {0x70, 0x3A},  // scaling_xsc
  {0x71, 0x35},  // scaling_ysc
  {0x72, 0x11}, // down sample by 2
  {0x73, 0xf1}, // DSP clock /= 2

  /* windowing (empirically decided...) */
  {0x17, 0x16},   // HSTART
  {0x18, 0x04},   // HSTOP
  {0x32, 0x80},   // HREF
  {0x19, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
  {0x1a, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
  {0x03, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

  /* color matrix coefficient */
#if 0
  {0x4f, 0xb3},
  {0x50, 0xb3},
  {0x51, 0x00},
  {0x52, 0x3d},
  {0x53, 0xa7},
  {0x54, 0xe4},
  {0x58, 0x9e},
#else
  {0x4f, 0x80},
  {0x50, 0x80},
  {0x51, 0x00},
  {0x52, 0x22},
  {0x53, 0x5e},
  {0x54, 0x80},
  {0x58, 0x9e},
#endif

  /* 3a */
//  {0x13, 0x84},
//  {0x14, 0x0a},   // AGC Ceiling = 2x
//  {0x5F, 0x2f},   // AWB B Gain Range (empirically decided)
//                  // without this bright scene becomes yellow (purple). might be because of color matrix
//  {0x60, 0x98},   // AWB R Gain Range (empirically decided)
//  {0x61, 0x70},   // AWB G Gain Range (empirically decided)
  {0x41, 0x38},   // edge enhancement, de-noise, AWG gain enabled


  /* gamma curve */
#if 1
  {0x7b, 16},
  {0x7c, 30},
  {0x7d, 53},
  {0x7e, 90},
  {0x7f, 105},
  {0x80, 118},
  {0x81, 130},
  {0x82, 140},
  {0x83, 150},
  {0x84, 160},
  {0x85, 180},
  {0x86, 195},
  {0x87, 215},
  {0x88, 230},
  {0x89, 244},
  {0x7a, 16},
#else
  /* gamma = 1 */
  {0x7b, 4},
  {0x7c, 8},
  {0x7d, 16},
  {0x7e, 32},
  {0x7f, 40},
  {0x80, 48},
  {0x81, 56},
  {0x82, 64},
  {0x83, 72},
  {0x84, 80},
  {0x85, 96},
  {0x86, 112},
  {0x87, 144},
  {0x88, 176},
  {0x89, 208},
  {0x7a, 64},
#endif

  /* fps */
//  {0x6B, 0x4a}, //PLL  x4
  {0x11, 0x00}, // pre-scalar = 1/1

  /* others */
  {0x1E, 0x31}, //mirror flip
//  {0x42, 0x08}, // color bar

  {0xFF, 0xFF},
};


/*
 * *******************************************************************
 * Funções ***********************************************************
 * *******************************************************************
 */

/*

* @brief Em caso de erro, pisca led built-in do kit
 */
void cam_error_led(void)
{
  while (1)
  {
	//wait for reset
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(100);
  }
}


/*
 * Escrita em registrador
 */
void cam_wrReg(uint8_t reg, uint8_t dat)
{
	uint8_t pacote[2];
	pacote[0] = reg;
	pacote[1] = dat;
	 //HAL_I2C_Master_Transmit(hi2c, DevAddress, pData,  Size, Timeout)
	if(HAL_I2C_Master_Transmit(HI2C, camAddr_WR, pacote, 2,    5) == HAL_ERROR)
	{
		cam_error_led();
	}
	HAL_Delay(1);
}

/*
 * Leitura de regitrador
 */

uint8_t cam_rdReg(uint8_t reg)
{
	uint8_t dat;
	HAL_I2C_Master_Transmit(HI2C, camAddr_WR, &reg, 1, 5);
	if(HAL_I2C_Master_Receive(HI2C, camAddr_RD, &dat, 1, 5) == HAL_ERROR)
		cam_error_led();
	return dat;
}

/*
 * Escrita de pacote de registradores
 */
void cam_wrSensorRegs8_8(const struct regval_list reglist[])
{
  uint8_t reg_addr, reg_val;
  const struct regval_list *next = reglist;

  //Define um valor inicial para as variáveis, o padrão testava antes de atribuir valor
  reg_addr = pgm_read_byte(&next->reg_num);
  reg_val = pgm_read_byte(&next->value);

  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = pgm_read_byte(&next->reg_num);
    reg_val = pgm_read_byte(&next->value);
    cam_wrReg(reg_addr, reg_val);
    next++;
  }
}

/*
 * @brief Ajuste de cores da câmera
 */
void cam_setColor(void)
{
#ifdef RGB  //Poloni 08-06-20
  cam_wrSensorRegs8_8(rgb565_ov7670);
#else
  cam_wrSensorRegs8_8(yuv422_ov7670);
#endif
}

/*
 * @brief Ajuste de resolução da câmera
 */
void cam_setRes(void)
{
  cam_wrReg(REG_COM3, 4); // REG_COM3 enable scaling
  cam_wrSensorRegs8_8(qvga_ov7670);
}

/*
 * @brief Inicializa registradores da câmera para uso via GPIOs
 */
void cam_init(void)
{
  cam_wrReg(0x12, 0x80);			//reseta todos os registradores para o valor default
  HAL_Delay(100);
  cam_wrSensorRegs8_8(ov7670_default_regs);
  cam_wrReg(REG_COM10, 32);			//PCLK does not toggle on HBLANK.
}

/*
 * @brief Inicializa registradores da câmera para uso via periférico DCMI
 */
void cam_init_DCMI(void)
{
	cam_wrReg(0x12, 0x80);			//reseta todos os registradores para o valor default
	HAL_Delay(100);
	cam_wrSensorRegs8_8(OV7670_reg);
}

/*
 * @brief Captura imagem via GPIOs
 */
void cam_captureImg(uint16_t wg, uint16_t hg)
{
	//Observação: evitei o uso da biblioteca HAL para diminuir a latência dos comandos

	uint16_t y, x, pixel;
	uint16_t R = 0, G = 0, B = 0, pixelx[wg];


	HAL_UART_Transmit(HUART, "*RDY*", 5, 5);	//Envia o aviso de novo frame para o programa OV7670

	while (!VSYNC);	//Espera uma borda de subida	(__/''')
	while ( VSYNC);	//Espera uma borda de descida	('''\__)

	for(y = 0; y < hg; y++)
	{

		//Não é obrigatório, funciona sem o teste, 24/7 tive que comentar por falha
		//while (!HREF);	//Espera uma borda de subida	(__/''')

		for(x = 0; x < wg; x++)
		{
			while ( PCLK);	//Espera uma borda de descida	('''\__)
			while (!PCLK);	//Espera uma borda de subida	(__/''')

			pixel = PIXEL;	//Lê o pixel paralelo

			//Habilitar para usar aplicativo no PC OV7670 - 320x240 monocromático ou RGB MSB
			//HUART->Instance->DR = pixel;	//Transmite o pixel pela serial, DATA byte mais significativo do YUV: Y (luma) ou RGB: 5R e 3G)
#ifndef RGB
			//Habilitar para enviar 320x240 monocromático para o LCD TFT
			//Coversão para de YUV para RGB565 monocromático
			R = (pixel & 0b11111000)<<8;
			G = (pixel & 0b11111100)<<3;
			B = (pixel & 0b11111000)>>3;
			pixelx[x] = R | G | B;
#else
			//Habilitar para enviar 320x240 RGB para o LCD TFT
			pixel = pixel << 8; //RGB MSB
#endif

			while ( PCLK);	//Espera uma borda de descida	('''\__)
			while (!PCLK);	//Espera uma borda de subida	(__/''')

#ifdef RGB
			pixel |= PIXEL;	//Lê o pixel paralelo (byte menos significativo)
			//Habilitar para enviar pela serial - 320x240 RGB LSB
			//HUART->Instance->DR = pixel;	//Transmite o pixel pela serial, DATA byte menos significativo do RGB: 3G e 5B)
			//Habilitar para enviar 320x240 RGB para o LCD TFT
			pixelx[x] = pixel;
#endif
		}

		//Tratamento e plotagem de pixels no display TFT
		for(x = 0; x < wg; x++)
		{
			//Plota pixels
			tft_desenhaPixel(pixelx[x]);
		}

		while (HREF);	//Espera uma borda de descida	('''\__)
	}
	HAL_Delay(10);
}

/*
 * @brief Set-up completo da câmera para uso via GPIOs
 */
void cam_setup(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	//Atribuo um ponteiro ao handler da porta I2C e da UART
	HI2C = hi2c;
	HUART = huart;

	cam_init();
	cam_setRes();           //Aqui é ajustado o tamanho da imagem
	cam_setColor();
#ifdef SOFTWARE
	cam_wrReg(0x11, 24);  //Prescaler freq. de saída de dados: 24+1 //Vide: https://circuitdigest.com/microcontroller-projects/how-to-use-ov7670-camera-module-with-arduino
#else
	#ifdef W320H240
		cam_wrReg(REG_CLKRC, 1);//10);     //Valor mínimo empírico do prescaler para escrita direta no LCD
	#else
		#ifdef RGB
			cam_wrReg(REG_CLKRC, 1);//11);     //Valor mínimo empírico do prescaler para escrita direta no LCD
		#else
			cam_wrReg(REG_CLKRC, 24);     //Valor mínimo empírico do prescaler para escrita direta no LCD
		#endif
	#endif
#endif
}

/*
 * @brief Set-up completo da câmera para uso via periférico DCMI
 */
void cam_setup_DCMI(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	//Atribuo um ponteiro ao handler da porta I2C e da UART
	HI2C = hi2c;
	HUART = huart;

	cam_init_DCMI();
	cam_setRes();           //Aqui é ajustado o tamanho da imagem
	cam_setColor();
}


/*
 * @brief Captura imagem via GPIOs com definição de tamanho
 */
void cam_loop()
{
#ifdef W320H240
	cam_captureImg(320, 240); //320x240
#else
	cam_captureImg(160, 120); //160x120
#endif
}
