/**
 * @file MPU6050_light_STM32.h
 * @author Leandro Poloni Dantas
 * @brief Biblioteca adaptada para MCU STM32 (HAL) do sensor inercial MPU6050.
 * @version 1.2.1
 * @date 2026
 * 
 * @details Esta biblioteca fornece uma interface simples para leitura de dados 
 *          de acelerômetro e giroscópio do MPU6050 via barramento I2C, incluindo 
 *          filtração complementar e rotinas automáticas de calibração de offset.
 *          Foi originariamente publicada para Arduino por Romain JL. Fétick, Tockn e
 *          adaptada de C++ para C para rodar nos MCUs STM32F4xx.
 *
 * @details O mapa de registradores é fornecido em
 *          https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 *          Mapeamento das diferentes configurações de giroscópio e acelerômetro:
 *
 *          GYRO_CONFIG_[0,1,2,3] faixa  = +- [250, 500, 1000, 2000] °/s
 *                               sensi =    [131, 65.5, 32.8, 16.4] bit/(°/s)
 *
 *          ACC_CONFIG_[0,1,2,3]  faixa  = +- [  2,    4,    8,   16] vezes a gravidade (9.81 m/s^2)
 *                               sensi =    [16384, 8192, 4096, 2048] bit/gravidade
 */

#ifndef MPU6050_LIGHT_STM32_H
#define MPU6050_LIGHT_STM32_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/** @defgroup MPU6050_Registers Endereços e Registradores do MPU6050
 *  @{
 */
#define MPU6050_ADDR         0x68  /*!< Endereço I2C padrão do MPU6050 (AD0 ligado ao GND) */
#define MPU6050_SMPLRT_DIV   0x19  /*!< Registrador de Divisor da Taxa de Amostragem */
#define MPU6050_CONFIG       0x1a  /*!< Registrador de Configuração Geral e DLPF */
#define MPU6050_GYRO_CONFIG  0x1b  /*!< Registrador de Configuração do Giroscópio */
#define MPU6050_ACCEL_CONFIG 0x1c  /*!< Registrador de Configuração do Acelerômetro */
#define MPU6050_WHO_AM_I     0x75  /*!< Registrador de Identificação do Dispositivo */
#define MPU6050_PWR_MGMT_1   0x6b  /*!< Registrador de Gerenciamento de Energia 1 */
#define MPU6050_TEMP_H       0x41  /*!< Byte Mais Significado (MSB) da Temperatura */
#define MPU6050_TEMP_L       0x42  /*!< Byte Menos Significado (LSB) da Temperatura */
/** @} */

/** @defgroup MPU6050_Constants Constantes de Conversão e Configuração
 *  @{
 */
#define GYRO_LSB_2_DEGSEC    65.5     /*!< Fator de escala do giroscópio para faixa de ±500 °/s [bit/(°/s)] */
#define ACC_LSB_2_G          16384.0  /*!< Fator de escala do acelerômetro para faixa de ±2g [bit/gravity] */
#define RAD_2_DEG            57.29578 /*!< Constante de conversão de Radianos para Graus [°/rad] */
#define GYRO_OFFSET_NB_MES   3000     /*!< Número de amostras coletadas para a calibração de offsets */
#define TEMP_LSB_2_DEGREE    340.0    /*!< Fator de conversão do sensor de temperatura [bit/°C] */
#define TEMP_LSB_OFFSET      12412.0  /*!< Offset de temperatura padrão do fabricante */
#define DEFAULT_GYRO_COEFF   0.98     /*!< Coeficiente padrão do filtro complementar (Peso do Giroscópio) */
/** @} */

/**
 * @brief Inicializa a comunicação com o MPU6050 utilizando os coeficientes padrão do filtro complementar.
 * @param hi2c Ponteiro para a estrutura de configuração do barramento I2C HAL (ex: &hi2c1).
 */
void MPU6050_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Inicializa o MPU6050 permitindo definir coeficientes personalizados para o filtro complementar.
 * @param hi2c Ponteiro para a estrutura de configuração do barramento I2C HAL.
 * @param aC Coeficiente de peso do acelerômetro (ex: 0.02).
 * @param gC Coeficiente de peso do giroscópio (ex: 0.98).
 */
void MPU6050_init2(I2C_HandleTypeDef *hi2c, float aC, float gC);

/**
 * @brief Configura os registradores internos do sensor e realiza a primeira leitura de referência.
 */
void MPU6050_begin();

/**
 * @brief Define manualmente os offsets de correção para os eixos do giroscópio.
 * @param x Offset para o eixo X do giroscópio.
 * @param y Offset para o eixo Y do giroscópio.
 * @param z Offset para o eixo Z do giroscópio.
 */
void MPU6050_setGyroOffsets(float x, float y, float z);

/**
 * @brief Define manualmente os offsets de correção para os eixos do acelerômetro.
 * @param x Offset para o eixo X do acelerômetro.
 * @param y Offset para o eixo Y do acelerômetro.
 * @param z Offset para o eixo Z do acelerômetro.
 */
void MPU6050_setAccOffsets(float x, float y, float z);

/**
 * @brief Executa a rotina automática de cálculo de offset coletando múltiplas amostras com o sensor em repouso.
 * @param run_gyro Booleano para indicar se a calibração do giroscópio deve ser executada.
 * @param run_accel Booleano para indicar se a calibração do acelerômetro deve ser executada.
 */
void MPU6050_calcOffsets(bool run_gyro, bool run_accel);

/**
 * @brief Escreve um dado em um registrador específico do MPU6050 via I2C.
 * @param reg Endereço do registrador de destino.
 * @param data Byte de dado a ser gravado.
 */
void writeMPU6050(uint8_t reg, uint8_t data);

/**
 * @brief Lê um único byte de um registrador do MPU6050.
 * @param reg Endereço do registrador a ser lido.
 * @return uint8_t O valor do byte lido.
 */
uint8_t readMPU6050(uint8_t reg);

/**
 * @brief Lê múltiplos bytes sequenciais a partir de um registrador do MPU6050.
 * @param reg Endereço do registrador inicial.
 * @param data Ponteiro para o buffer de armazenamento dos dados lidos.
 * @param size Quantidade de bytes a serem lidos.
 * @return uint8_t Retorna 0 em caso de sucesso.
 */
uint8_t readMPU6050_n(uint8_t reg, uint8_t *data, uint8_t size);

/**
 * @brief Obtém a temperatura interna medida pelo chip em graus Celsius.
 * @return float Temperatura em °C.
 */
float MPU6050_getTemp();

/** @defgroup MPU6050_Getters Funções de Leitura de Dados Brutos/Corrigidos
 *  @{
 */
float MPU6050_getAccX();
float MPU6050_getAccY();
float MPU6050_getAccZ();

float MPU6050_getGyroX();
float MPU6050_getGyroY();
float MPU6050_getGyroZ();

float MPU6050_getGyroXoffset();
float MPU6050_getGyroYoffset();
float MPU6050_getGyroZoffset();

float MPU6050_getAccXoffset();
float MPU6050_getAccYoffset();
float MPU6050_getAccZoffset();
/** @} */

/**
 * @brief Atualiza as leituras do sensor, calcula o intervalo de tempo (dt) e executa o Filtro Complementar.
 * @note Esta função deve ser chamada repetidamente no loop principal do microcontrolador.
 */
void MPU6050_update();

/** @defgroup MPU6050_Angles Funções de Obtenção de Ângulos Calculados
 *  @{
 */
float MPU6050_getAccAngleX();
float MPU6050_getAccAngleY();

float MPU6050_getAngleX();
float MPU6050_getAngleY();
float MPU6050_getAngleZ();
/** @} */

/**
 * @brief Função de tratamento de erro crítico que trava o código piscando um LED de indicação.
 */
void error_led(void);

#endif /* MPU6050_LIGHT_STM32_H */
