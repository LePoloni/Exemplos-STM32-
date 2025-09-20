/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSBH;    /* Return value for USBH */
char USBHPath[4];   /* USBH logical drive path */
FATFS USBHFatFS;    /* File system object for USBH logical drive */
FIL USBHFile;       /* File object for USBH */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USBH driver ###########################*/
  retUSBH = FATFS_LinkDriver(&USBH_Driver, USBHPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  //return 0;

	//Essa função precisa ser escrita de acordo a leitura do RTC e o
	//modelo de preenchimento disponível em:
	//http://elm-chan.org/fsw/ff/doc/fattime.html
	/*
	 * Currnet local time shall be returned as bit-fields packed into a DWORD value.
	 * The bit fields are as follows:
	 * bit31:25 Year origin from the 1980 (0..127, e.g. 37 for 2017)
	 * bit24:21 Month (1..12)
	 * bit20:16 Day of the month (1..31)
	 * bit15:11 Hour (0..23)
	 * bit10:5  Minute (0..59)
	 * bit4:0   Second / 2 (0..29, e.g. 25 for 50)
	 */
	//Vide arquivo main.c
	return le_data_e_hora_para_fucao_get_fattime_do_FatFs();

  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
