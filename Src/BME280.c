/*
 * 	BME280.c
 *
 * 	@author Antonio Carretero Sahuquillo
 *
 * 	El siguiente archivo corresponde con la implementación de las funciones
 * 	que controlan el sensor BME280, cuyos prototipos se encuentran en la cabecera BME280.h.
 *
 * 	El archivo de implementación se estructura de la siguiente manera:
 *
 */

//Includes
#include "BME280.h"

//Prototipos de las funciones estáticas:
static BME280_STATUS read_reg(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t * pStoreData);
static BME280_STATUS read_burst_raw(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c);
static BME280_STATUS write_reg(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t * pWriteData, uint8_t data_size);
static BME280_STATUS soft_reset(BME280_H * bme280_handler, I2C_HandleTypeDef * hi2c);
static BME280_STATUS get_corrections(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c);
static int32_t BME280_compensate_T_int32(BME280_H * bme280_handler);
//static uint32_t BME280_compensate_P_int64(BME280_H * bme280_handler);
static uint32_t BME280_compensate_P_int32(BME280_H * bme280_handler);
static uint32_t bme280_compensate_H_int32(BME280_H * bme280_handler);

/************************************************
 * FUNCIONES PARA EL USO DEL DISPOSITIVO BME280 *
 *												*/

/*
 * @brief Función que inicializa y comprueba la conexión con el dispositivo por I2C.
 *
 * @param BME280_* bme280_handler: Manejador del dispositivo.
 * @param I2C_HandleTypedef * hi2c: Manejador de la interfaz I2C.
 *
 * @return BME280_STATUS
 */
BME280_STATUS bme280_init(BME280_H * bme280_handler, I2C_HandleTypeDef * hi2c)
{
	//Antes de comenzar, se realiza un reseteo por software del dispositivo.
	if(soft_reset(bme280_handler, hi2c)!=BME280_OK)
	{
		return BME280_ERR;
	}

	//Para asegurarnos de que existe conexión, leemos el id del chip.
	if((read_reg(hi2c, ID_ADDR, &bme280_handler->chip_id) != BME280_OK) || (bme280_handler->chip_id != BME280_CHIP_ID))
	{
		return BME280_ERR;
	}

	return BME280_OK;
}

/*
 * @brief: Función que añade los datos de configuración al manejador del dispositivo. Y
 * tras ello, los escribe en los registros correspondientes en el dispositivo bme280.
 *
 *
 * @param BME280_h * bme280_handler: Manejador del dispositivo BME280.
 * @param I2C_HandleTypeDef* hi2c: Manejador de la interfaz I2C.
 *
 * @param uint8_t oversample_hum: Número de sobremuestreo para la humedad.
 * @param uint8_t oversample_press: Número de sobremuestreo para la presión.
 * @param uint8_t oversample_temp: Número de sobremuestreo para la temperatura.
 *
 * @param uint8_t mode_meas: Modo de medida.
 *
 * @param uint8_t t_standby: Tiempo de espera en modo normal.
 * @param uint8_t t_filt: Tiempo de espera del filtro IIR.
 *
 * @param uint8_t wspi3: Opción para 3-wired-SPI
 *
 * La configuración que usaremos será: x, x, OSRS_OVERSAMPLE_SKIP, OSRS_OVERSAMPLE_SKIP, OSRS_OVERSAMPLE_SKIP, MODE_NORMAL, T_STANBY_125, FILTER_OFF, WSPI3_DISABLE
 *
 *
 * @retval BME280_STATUS
 */
BME280_STATUS bme280_config(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c, uint8_t oversample_hum, uint8_t oversample_press, uint8_t oversample_temp, uint8_t mode_meas, uint8_t t_standby, uint8_t t_filt, uint8_t wspi3)
{
	//Variables para la comprobación de la escritura de la configuración.
	uint8_t ctrl_hum_test = 0;
	uint8_t ctrl_meas_test = 0;
	uint8_t ctrl_config_test = 0;

//	//Lectura de los valores de corrección (Solo necesitan ser leídos una vez y no cambian nunca)
//	if(get_corrections(bme280_handler, hi2c)!=BME280_OK)
//	{
//		return BME280_ERR;
//	}

	//Variables locales con las palabras de configuración completas:
	uint8_t ctrl_hum_configuration = oversample_hum;

	uint8_t ctrl_meas_configuration[3];
	ctrl_meas_configuration[0] = (oversample_temp<<5);
	ctrl_meas_configuration[1] = (oversample_press<<2);
	ctrl_meas_configuration[2] = mode_meas;

	uint8_t ctrl_config_configuration[3];
	ctrl_config_configuration[0] = (t_standby<<5);
	ctrl_config_configuration[1] = (t_filt<<2);
	ctrl_config_configuration[2] = wspi3;


	//Escritura de la configuración en el manejador
	bme280_handler->config.ctrl_hum = ctrl_hum_configuration;
	bme280_handler->config.ctrl_meas = ctrl_meas_configuration[0] | ctrl_meas_configuration[1] | ctrl_meas_configuration[2];
	bme280_handler->config.ctrl_config = ctrl_config_configuration[0] | ctrl_config_configuration[1] | ctrl_config_configuration[2];


	//Escritura de la configuración en el dispositivo y comprobación de escritura.
	if(write_reg(hi2c, CTRL_HUM_ADDR, &bme280_handler->config.ctrl_hum, 1)!=BME280_OK)
	{
		return BME280_ERR;
	}
	read_reg(hi2c, CTRL_HUM_ADDR, &ctrl_hum_test);
	if(ctrl_hum_test != bme280_handler->config.ctrl_hum)
	{
		return BME280_ERR;
	}



	if(write_reg(hi2c, CTRL_MEAS_ADDR, &bme280_handler->config.ctrl_meas, 1)!=BME280_OK)
	{
		return BME280_ERR;
	}
	read_reg(hi2c, CTRL_MEAS_ADDR, &ctrl_meas_test);
	if(ctrl_meas_test!=bme280_handler->config.ctrl_meas)
	{
		return BME280_ERR;
	}


	if(write_reg(hi2c, CTRL_CONFIG_ADDR, &bme280_handler->config.ctrl_config, 1)!=BME280_OK)
	{
		return BME280_ERR;
	}
	read_reg(hi2c, CTRL_CONFIG_ADDR, &ctrl_config_test);
	if(ctrl_config_test!=bme280_handler->config.ctrl_config)
	{
		return BME280_ERR;
	}

	return BME280_OK;
}

/**
 * @brief Funcion que obtiene los valores en unidades del SI
 *
 * @param BME280_H * bme280_handler: Manejador del dispositivo BME280
 * @param I2C_HandleTypeDef* hi2c: Manejador de la interfaz I2C
 *
 * @retval BME280_STATUS
 */
BME280_STATUS bme280_get_values(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c)
{
	//Lectura de los valores de corrección (Solo necesitan ser leídos una vez y no cambian nunca)
	if(get_corrections(bme280_handler, hi2c)!=BME280_OK)
	{
		return BME280_ERR;
	}

	//Lectura de los valores en crudo del sensor.
	if(read_burst_raw(bme280_handler, hi2c)!=BME280_OK)
	{
		return BME280_ERR;
	}

	//Segundo, se obtienen los valores compensados.
	bme280_handler->rslt.T = BME280_compensate_T_int32(bme280_handler);
	temp = bme280_handler->rslt.T;

	//bme280_handler->rslt.P = BME280_compensate_P_int64(bme280_handler);
	bme280_handler->rslt.P = BME280_compensate_P_int32(bme280_handler);
	press = bme280_handler->rslt.P;

	bme280_handler->rslt.rh = bme280_compensate_H_int32(bme280_handler);
	rh = bme280_handler->rslt.rh;

	return BME280_OK;
}


/* *************************************************
 * FUNCIONES PARA LECTURA Y ESCRITURA / RESET DE REGISTROS */
/* Estas funciones serán estáticas, pues no deben utilizarse fuera de
 * las funciones que no se encuentren en este fichero. (El usuario en el main no podrá llamarlas)*/


/**
 * @brief Esta función es la encargada de leer los registros del dispositivo,
 * y almacenar en la dirección indicada el resultado.
 *
 * @param I2C_HandleTypeDef* hi2c: Manejador de la interfaz I2C de la placa STM32.
 * @param uint8_t reg_addr: Dirección del registro que se desea leer.
 * @param uint8_t * pStoreData: Dirección de memoria donde almacenar el valor leído.
 *
 * @retval BME280_STATUS
 */
static BME280_STATUS read_reg(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t * pStoreData)
{
	if(HAL_I2C_Mem_Read(hi2c, (uint16_t)(BME280_DEV_ADDR<<1), (uint16_t)reg_addr, 1, pStoreData, 1, HAL_MAX_DELAY)==HAL_ERROR)
	{
		return BME280_ERR;
	}
	return BME280_OK;
}

/*
 * @brief Función que lee en modo burst (de una) todos los datos de las medidas, en el momento de la llamada.
 *
 * @param  BME280_H * bme280_handler: Manejador del dispositivo
 * @param I2C_HandleTypeDef* hi2c: Manejador de la interfaz I2C
 *
 * @retval BME280_TATUS
 */
static BME280_STATUS read_burst_raw(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c)
{
	//Variable local donde se almacenará en serie todos los datos.
	uint8_t raw_data[9];

	//Lectura de todos los datos en modo burst.
	if(HAL_I2C_Mem_Read(hi2c, (BME280_DEV_ADDR<<1), PRESS_MSB_ADDR, 1, (uint8_t *)&raw_data, 8, HAL_MAX_DELAY)==HAL_ERROR)
	{
		return BME280_ERR;
	}

	//Almacenamiento de los datos en crudo en el manejador del dispositivo.
	bme280_handler->meas_raw.press_raw = ((raw_data[0]<<12) | (raw_data[1]<<4) | (raw_data[2]>>4));

	bme280_handler->meas_raw.temp_raw = ((raw_data[3]<<12) | (raw_data[4]<<4) | (raw_data[5]>>4));

	bme280_handler->meas_raw.hum_raw = (raw_data[6]<<8) | raw_data[7];

	return BME280_OK;
}

/*
 * @brief Esta función es la encargada de escribir en los registros del dispositivo,
 * cuyo valor es tomado de la dirección indicada como parámetro.
 *
 * @param I2C_HandleTypeDef* hi2c: Manejador de la interfaz I2C de la placa STM32.
 * @param uint8_t reg_addr: Dirección del registro que se desea escribir.
 * @param uint8_t * pWriteData: Dirección de memoria donde se almacena el valor que se desea escribir.
 * @param uint8_t data_size: Tamaño en bytes de los datos que se quieren escribir.
 *
 * @retval BME280_STATUS
 */
static BME280_STATUS write_reg(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t * pWriteData, uint8_t data_size)
{
	if(HAL_I2C_Mem_Write(hi2c, (uint16_t)(BME280_DEV_ADDR << 1) | 0x01, (uint16_t)reg_addr, 1, pWriteData, data_size, HAL_MAX_DELAY)==HAL_ERROR)
	{
		return BME280_ERR;
	}
	return BME280_OK;
}


/*
 * @brief Escribe, a través de la interfaz i2c, la palabra de reset en su respectivo registro y
 * reinicia el valor del id del chip, para su futura lectura.
 *
 * @param BME280_h * bme280_handler: Manejador del dispositivo
 * @param I2C_HandleTypeDef * :Manejador de la interfaz I2C
 *
 * @retval BME280_STATUS
 */
static BME280_STATUS soft_reset(BME280_H * bme280_handler, I2C_HandleTypeDef * hi2c)
{
	/* Variable local con la palabra de reseteo */
	uint8_t reset_word = SOFT_RESET_VAL;

	/* Escritura de la palabra de reset */
	if(write_reg(hi2c, RESET_ADDR, (uint8_t *)&reset_word, 1) != BME280_OK)
	{
		return BME280_ERR;
	}

	/* Proceso del reseteo id del handler*/
	bme280_handler->chip_id = 0x00;

	bme280_handler->status = STATUS_RESET_VAL;

	bme280_handler->config.ctrl_config = CTRL_CONFIG_RESET_VAL;
	bme280_handler->config.ctrl_hum = CTRL_HUM_RESET_VAL;
	bme280_handler->config.ctrl_meas = CTRL_MEAS_RESET_VAL;

	bme280_handler->meas_raw.hum_raw = ((HUM_MSB_RESET_VAL << 8) | (HUM_LSB_RESET_VAL)) & 0xFF;
	bme280_handler->meas_raw.temp_raw = ((TEMP_MSB_RESET_VAL<<16) | (TEMP_LSB_RESET_VAL<<8) | (TEMP_XLSB_RESET_VAL)) & 0x00FFFFFF;
	bme280_handler->meas_raw.press_raw = ((PRESS_MSB_RESET_VAL<<16) | (PRESS_LSB_RESET_VAL<<8) | (PRESS_XLSB_RESET_VAL)) & 0x00FFFFFF;


	return BME280_OK;
}

/**
 * @brief Función para la obtención de los datos de corrección.
 *
 * @param BME280_H * bme280_handler: Handler del dispositivo.
 * @param I2C_HandleTypeDef* hi2c: Handler de la interfaz I2C.
 *
 * @retval BME280_STATUS
 */
static BME280_STATUS get_corrections(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c)
{
	//Variables locales donde se almacenarán los registros leídos
	uint8_t first_dig[25];
	uint8_t second_dig[6];

	//Lectura de los registros de corrección del BME280
	if(HAL_I2C_Mem_Read(hi2c, BME280_DEV_ADDR<<1, 0x88, 1, (uint8_t *)&first_dig, 25, 1000)!=HAL_OK)
	{
		return BME280_ERR;
	}

	if(HAL_I2C_Mem_Read(hi2c, BME280_DEV_ADDR<<1, 0xE1, 1, (uint8_t *)&second_dig, 6, 1000)!=HAL_OK)
	{
		return BME280_ERR;
	}

	//Almacenamiento de los registros de corrección en la estructura del dispositivo.
	bme280_handler->calib.dig_T1 = (first_dig[1]<<8) | first_dig[0];
	bme280_handler->calib.dig_T2 = (first_dig[3]<<8) | first_dig[2];
	bme280_handler->calib.dig_T3 = (first_dig[5]<<8) | first_dig[4];

	bme280_handler->calib.dig_P1 = (first_dig[7]<<8) | first_dig[6];
	bme280_handler->calib.dig_P2 = (first_dig[9]<<8) | first_dig[8];
	bme280_handler->calib.dig_P3 = (first_dig[11]<<8) | first_dig[10];
	bme280_handler->calib.dig_P4 = (first_dig[13]<<8) | first_dig[12];
	bme280_handler->calib.dig_P5 = (first_dig[15]<<8) | first_dig[14];
	bme280_handler->calib.dig_P6 = (first_dig[17]<<8) | first_dig[16];
	bme280_handler->calib.dig_P7 = (first_dig[19]<<8) | first_dig[18];
	bme280_handler->calib.dig_P8 = (first_dig[21]<<8) | first_dig[20];
	bme280_handler->calib.dig_P9 = (first_dig[23]<<8) | first_dig[22];

	bme280_handler->calib.dig_H1 = first_dig[24];
	bme280_handler->calib.dig_H2 = (second_dig[1]<<8) | second_dig[0];
	bme280_handler->calib.dig_H3 = second_dig[2];
	bme280_handler->calib.dig_H4 = (second_dig[3]<<4) | (second_dig[4] & 0x0f);
	bme280_handler->calib.dig_H5 = (second_dig[5]<<4) | (second_dig[4]>>4);
	bme280_handler->calib.dig_H6 = second_dig[6];


	return BME280_OK;
}

/*
 * @brief Compensación de los valores de temperatura
 *
 * @param BME280_H * bme280_handler: Manejador del sensor BME280
 *
 * @retval int32_t: Valor de la temperatura con resolución de 0.01 DegC (T/100)
 */
static int32_t BME280_compensate_T_int32(BME280_H * bme280_handler)
{
	int32_t var1, var2, T;
	var1 = ((((bme280_handler->meas_raw.temp_raw>>3) - ((int32_t)bme280_handler->calib.dig_T1<<1))) * ((int32_t)bme280_handler->calib.dig_T2)) >> 11;
	var2 = (((((bme280_handler->meas_raw.temp_raw>>4) - ((int32_t)bme280_handler->calib.dig_T1)) * ((bme280_handler->meas_raw.temp_raw>>4) - ((int32_t)bme280_handler->calib.dig_T1))) >> 12) * ((int32_t)bme280_handler->calib.dig_T3)) >> 14;

	bme280_handler->calib.t_fine = var1 + var2;
	T = (bme280_handler->calib.t_fine * 5 + 128) >> 8;
	return T;
}

/*
 * @brief Compensación de los valores de presión
 *
 * @param BME280_H * bme280_handler: Manejador del sensor BME280
 *
 * @retval uint32_t: Valor de la presión en Pa
 */
static uint32_t BME280_compensate_P_int32(BME280_H * bme280_handler)
{
//	int64_t var1, var2, p;
//	var1 = ((int64_t)bme280_handler->calib.t_fine) - 128000;
//	var2 = var1 * var1 * (int64_t)bme280_handler->calib.dig_P6;
//	var2 = var2 + ((var1*(int64_t)bme280_handler->calib.dig_P5)<<17);
//	var2 = var2 + (((int64_t)bme280_handler->calib.dig_P4)<<35);
//	var1 = ((var1 * var1 * (int64_t)bme280_handler->calib.dig_P3)>>8) + ((var1 * (int64_t)bme280_handler->calib.dig_P2)<<12);
//	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280_handler->calib.dig_P1)>>33;
//	if (var1 == 0)
//	{
//		return 0; //Evita excepciones causadas por dividir por cero.
//	}
//	p = 1048576-bme280_handler->meas_raw.press_raw;
//	p = (((p<<31)-var2)*3125)/var1;
//	var1 = (((int64_t)bme280_handler->calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
//	var2 = (((int64_t)bme280_handler->calib.dig_P8) * p) >> 19;
//	p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_handler->calib.dig_P7)<<4);
//	return (uint32_t)p;

	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)bme280_handler->calib.t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)bme280_handler->calib.dig_P6);
	var2 = var2 + ((var1*((int32_t)bme280_handler->calib.dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)bme280_handler->calib.dig_P4)<<16);
	var1 = (((bme280_handler->calib.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)bme280_handler->calib.dig_P2) * var1)>>1))>>18;
	var1 = ((((32768+var1))*((int32_t)bme280_handler->calib.dig_P1))>>15);
	if(var1 == 0)
	{
		return 0; // Evita excepciones causadas por dividir entre cero.
	}
	p = (((uint32_t)(((int32_t)1048576)-bme280_handler->meas_raw.press_raw)-(var2>>12)))*3125;
	if(p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)bme280_handler->calib.dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)bme280_handler->calib.dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + bme280_handler->calib.dig_P7) >> 4));
	return p;
}

/**
 * @brief Compensación de los valores de humedad
 *
 * @param BME280_H * bme280_handler: Manejador del sensor BME280
 *
 * @retval uint32_t: Valor de humedad en %RH (/1024)
 */
static uint32_t bme280_compensate_H_int32(BME280_H * bme280_handler)
{
	int32_t v_x1_u32r;

	v_x1_u32r = (bme280_handler->calib.t_fine - ((int32_t)76800));
	v_x1_u32r = ((((((int32_t)bme280_handler->meas_raw.hum_raw << 14) - (((int32_t)bme280_handler->calib.dig_H4) << 20) - (((int32_t)bme280_handler->calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bme280_handler->calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bme280_handler->calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bme280_handler->calib.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme280_handler->calib.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}
