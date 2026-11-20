/**
 * @file MPU6050_light_STM32.c
 * @author Leandro Poloni Dantas
 * @brief Implementação da biblioteca MPU6050 para microcontroladores STM32 usando a HAL.
 * @version 1.2.1
 * @date 2026
 * 
 * @details Contém as rotinas de comunicação I2C, cálculo de offsets (giroscópio e acelerômetro),
 *          atualização do filtro complementar e obtenção de ângulos dinâmicos.
 *          Foi originariamente publicada para Arduino por Romain JL. Fétick, Tockn e
 *          adaptada de C++ para C para rodar nos MCUs STM32F4xx.
 */

#include "stm32f4xx_hal.h"
#include "MPU6050_light_STM32.h"
#include <math.h>
#include <stdbool.h>

/* --- Variáveis Globais Privadas do Módulo --- */
float gyroXoffset = 0.0f, gyroYoffset = 0.0f, gyroZoffset = 0.0f; /*!< Deslocamentos estáticos do giroscópio */
float accXoffset = 0.0f, accYoffset = 0.0f, accZoffset = 0.0f;   /*!< Deslocamentos estáticos do acelerômetro */
float temp = 0.0f;                                              /*!< Temperatura atual em °C */
float accX = 0.0f, accY = 0.0f, accZ = 0.0f;                    /*!< Acelerações lineares corrigidas em g */
float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;                 /*!< Taxas angulares corrigidas em °/s */
float angleAccX = 0.0f, angleAccY = 0.0f;                       /*!< Ângulos instantâneos calculados pelo acelerômetro */
float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f;              /*!< Ângulos finais filtrados (Filtro Complementar) */
long  preInterval = 0;                                          /*!< Armazena o carimbo de tempo anterior para cálculo do dt */
float accCoef = 0.02f, gyroCoef = 0.98f;                        /*!< Coeficientes de ponderação do filtro complementar */

/** @brief Ponteiro para a instância do barramento I2C utilizada pela HAL. */
I2C_HandleTypeDef *HI2C;

/**
 * @brief Inicializa o driver do MPU6050 com os coeficientes padrão do filtro complementar.
 * @param hi2c Ponteiro para o handler do I2C da HAL (ex: &hi2c1).
 */
void MPU6050_init(I2C_HandleTypeDef *hi2c)
{
	HI2C = hi2c;
	accCoef = 1.0f - DEFAULT_GYRO_COEFF;
	gyroCoef = DEFAULT_GYRO_COEFF;
	MPU6050_begin();
}

/**
 * @brief Inicializa o driver do MPU6050 definindo coeficientes personalizados para o filtro.
 * @param hi2c Ponteiro para o handler do I2C da HAL.
 * @param aC Coeficiente de peso do acelerômetro.
 * @param gC Coeficiente de peso do giroscópio.
 */
void MPU6050_init2(I2C_HandleTypeDef *hi2c, float aC, float gC)
{
	HI2C = hi2c;
	accCoef = aC;
	gyroCoef = gC;
	MPU6050_begin();
}

/**
 * @brief Configura os registradores do sensor por I2C e executa a primeira leitura base.
 */
void MPU6050_begin()
{
	// Passo 1: Envia o comando de reset para zerar todos os registradores e limpar lixos anteriores
	writeMPU6050(MPU6050_PWR_MGMT_1, 0x80);
	HAL_Delay(100); // Aguarda o hardware estabilizar após o reset

	// Passo 2: Acorda o sensor do modo Sleep e configura a fonte de clock com o PLL do eixo X do giroscópio
	writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
	HAL_Delay(50);  // Pequena pausa para garantir a estabilização do clock

	// Passo 3: Configura os demais parâmetros com o chip acordado e operacional
	writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // Taxa de amostragem padrão
	writeMPU6050(MPU6050_CONFIG, 0x00);       // Configuração de filtro digital (DLPF)
	writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // Configura a faixa do giroscópio para ±500 °/s
	writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // Configura a faixa do acelerômetro para ±2g

	// Passo 4: Realiza a primeira leitura para inicializar as variáveis de ângulo e o tempo
	MPU6050_update();
	angleX = MPU6050_getAccAngleX();
	angleY = MPU6050_getAccAngleY();
	preInterval = HAL_GetTick();
}

/**
 * @brief Escreve um byte em um registrador específico do MPU6050.
 * @param reg Endereço do registrador de destino.
 * @param data Dado (byte) a ser gravado.
 */
void writeMPU6050(uint8_t reg, uint8_t data)
{
    uint8_t pacote[2];
  	pacote[0] = reg;
  	pacote[1] = data;
  	
  	if(HAL_I2C_Master_Transmit(HI2C, MPU6050_ADDR << 1, pacote, 2, 5) == HAL_ERROR)
  	{
  		error_led();
  	}
}

/**
 * @brief Lê um único byte de um registrador do MPU6050.
 * @param reg Endereço do registrador de origem.
 * @return uint8_t O valor do byte lido.
 */
uint8_t readMPU6050(uint8_t reg)
{
	uint8_t data;
	HAL_I2C_Master_Transmit(HI2C, (MPU6050_ADDR << 1) | 1, &reg, 1, 5);
	if(HAL_I2C_Master_Receive(HI2C, (MPU6050_ADDR << 1) | 1, &data, 1, 5) == HAL_ERROR)
		error_led();
	return data;
}

/**
 * @brief Lê múltiplos bytes sequenciais a partir de um registrador do MPU6050.
 * @param reg Endereço do registrador inicial.
 * @param data Ponteiro para o buffer de destino dos dados.
 * @param size Quantidade de bytes a serem lidos.
 * @return uint8_t Retorna 0 em caso de sucesso.
 */
uint8_t readMPU6050_n(uint8_t reg, uint8_t *data, uint8_t size)
{
	HAL_I2C_Master_Transmit(HI2C, MPU6050_ADDR << 1, &reg, 1, 5);
	if(HAL_I2C_Master_Receive(HI2C, (MPU6050_ADDR << 1) | 1, data, size, 5) == HAL_ERROR)
		error_led();
	return 0;
}

/**
 * @brief Define manualmente os offsets de correção para o giroscópio.
 * @param x Offset do eixo X.
 * @param y Offset do eixo Y.
 * @param z Offset do eixo Z.
 */
void MPU6050_setGyroOffsets(float x, float y, float z)
{
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

/**
 * @brief Define manualmente os offsets de correção para o acelerômetro.
 * @param x Offset do eixo X.
 * @param y Offset do eixo Y.
 * @param z Offset do eixo Z.
 */
void MPU6050_setAccOffsets(float x, float y, float z)
{
	accXoffset = x;
	accYoffset = y;
	accZoffset = z;
}

/**
 * @brief Coleta amostras em repouso e calcula automaticamente os offsets do sensor.
 * @param run_gyro Se verdadeiro, calcula e atualiza os offsets do giroscópio.
 * @param run_accel Se verdadeiro, calcula e atualiza os offsets do acelerômetro.
 */
void MPU6050_calcOffsets(bool run_gyro, bool run_accel)
{
	float ag[3] = {0.0f, 0.0f, 0.0f};
	float aa[3] = {0.0f, 0.0f, 0.0f};
	uint8_t raw[14];
	int16_t rawData[7];

	float backup_g[3] = {gyroXoffset, gyroYoffset, gyroZoffset};
	float backup_a[3] = {accXoffset, accYoffset, accZoffset};

	MPU6050_setGyroOffsets(0, 0, 0);
	MPU6050_setAccOffsets(0, 0, 0);

	for(int i = 0; i < GYRO_OFFSET_NB_MES; i++)
	{
		readMPU6050_n(0x3B, raw, 14);

		for(int j = 0; j < 7; j++)
		{
			rawData[j]  = raw[j*2] << 8;
			rawData[j] |= raw[j*2+1];
		}

		aa[0] += (float)rawData[0] / ACC_LSB_2_G;
		aa[1] += (float)rawData[1] / ACC_LSB_2_G;
		aa[2] += (float)rawData[2] / ACC_LSB_2_G;

		ag[0] += (float)rawData[4] / GYRO_LSB_2_DEGSEC;
		ag[1] += (float)rawData[5] / GYRO_LSB_2_DEGSEC;
		ag[2] += (float)rawData[6] / GYRO_LSB_2_DEGSEC;
	}

	if(run_accel)
	{
		accXoffset = aa[0] / GYRO_OFFSET_NB_MES;
		accYoffset = aa[1] / GYRO_OFFSET_NB_MES;
		accZoffset = (aa[2] / GYRO_OFFSET_NB_MES) - 1.0f; // Considera +1g no eixo Z vertical
	}
	else
	{
		MPU6050_setAccOffsets(backup_a[0], backup_a[1], backup_a[2]);
	}

	if(run_gyro)
	{
		gyroXoffset = ag[0] / GYRO_OFFSET_NB_MES;
		gyroYoffset = ag[1] / GYRO_OFFSET_NB_MES;
		gyroZoffset = ag[2] / GYRO_OFFSET_NB_MES;
	}
	else
	{
		MPU6050_setGyroOffsets(backup_g[0], backup_g[1], backup_g[2]);
	}
}

/**
 * @brief Atualiza os dados brutos, aplica os offsets, calcula o tempo (dt) e executa o Filtro Complementar.
 */
void MPU6050_update()
{
	uint8_t raw[14];
	int16_t rawData[7];

	readMPU6050_n(0x3B, raw, 14);

	for(int i = 0; i < 7; i++)
	{
		rawData[i]  = raw[i*2] << 8;
		rawData[i] |= raw[i*2+1];
	}

	// Conversão e aplicação de offsets
	accX = ((float)rawData[0]) / ACC_LSB_2_G - accXoffset;
	accY = ((float)rawData[1]) / ACC_LSB_2_G - accYoffset;
	accZ = ((float)rawData[2]) / ACC_LSB_2_G - accZoffset;
	
	temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
	
	gyroX = ((float)rawData[4]) / GYRO_LSB_2_DEGSEC - gyroXoffset;
	gyroY = ((float)rawData[5]) / GYRO_LSB_2_DEGSEC - gyroYoffset;
	gyroZ = ((float)rawData[6]) / GYRO_LSB_2_DEGSEC - gyroZoffset;

	// Cálculo do ângulo instantâneo via Acelerômetro (Trigonometria)
	float sgZ = (accZ >= 0.0f) - (accZ < 0.0f);
	angleAccX = atan2(accY, sgZ * sqrt(accZ * accZ + accX * accX)) * RAD_2_DEG;
	angleAccY = -atan2(accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;

	// Cálculo do intervalo de tempo (dt) em segundos
	unsigned long Tnew = HAL_GetTick();
	float dt = (float)(Tnew - preInterval) * 1e-3f;
	preInterval = Tnew;

	// Aplicação do Filtro Complementar
	angleX = (gyroCoef * (angleX + gyroX * dt)) + (accCoef * angleAccX);
	angleY = (gyroCoef * (angleY + gyroY * dt)) + (accCoef * angleAccY);
	angleZ += gyroZ * dt;
}

/**
 * @brief Tratamento de erro crítico que bloqueia a execução piscando um LED de indicação.
 */
void error_led(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(100);
	}
}

/* --- Funções de Retorno (Getters) --- */
float MPU6050_getTemp(){ return temp; }

float MPU6050_getAccX(){ return accX; }
float MPU6050_getAccY(){ return accY; }
float MPU6050_getAccZ(){ return accZ; }

float MPU6050_getGyroX(){ return gyroX; }
float MPU6050_getGyroY(){ return gyroY; }
float MPU6050_getGyroZ(){ return gyroZ; }

float MPU6050_getGyroXoffset(){ return gyroXoffset; }
float MPU6050_getGyroYoffset(){ return gyroYoffset; }
float MPU6050_getGyroZoffset(){ return gyroZoffset; }

float MPU6050_getAccXoffset(){ return accXoffset; }
float MPU6050_getAccYoffset(){ return accYoffset; }
float MPU6050_getAccZoffset(){ return accZoffset; }

float MPU6050_getAccAngleX(){ return angleAccX; }
float MPU6050_getAccAngleY(){ return angleAccY; }

float MPU6050_getAngleX(){ return angleX; }
float MPU6050_getAngleY(){ return angleY; }
float MPU6050_getAngleZ(){ return angleZ; }
