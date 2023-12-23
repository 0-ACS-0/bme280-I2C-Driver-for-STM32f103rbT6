/*
 *	BME288.h
 *
 *	@author Antonio Carretero Sahuquillo
 *
 *	El siguiente archivo corresponde con la cabcera del código fuente BME280.c.
 *	En este, se definen valores importantes, el manejador del dispositivo y los prototipos
 *	de las funciones, cuya implementación se encuentra en el .c.
 *
 *	La cabcera se estructura de la siguiente manera:
 *	SECCIÓN DE LAS DEFINICIONES:
 *		-Direcciones de registros. L18
 *		-Máscaras para el control de datos. L47
 *		-Máscaras para el control de estado. L63
 *		-Máscaras para el control de las medidas. L70
 *		-Valores de configuración. L91
 *		-Valores de reset. L129
 *
 *	SECCIÓN DE ESTRUCTURAS:
 *		-Estructuras secundarias para el manejador. L143
 *		-Estructura principal del manejador. L160
 *		-Estados de retorno. L177
 *
 *  SECCIÓN DE PROTOTIPOS DE FUNCIONES. L188
 *
 */
#ifndef BME280_DRIVER
#define BME280_DRIVER



//Librerías necesarias para el driver
#include <stdint.h>
#include "stm32f1xx_hal.h"

//Variables externas (del main) usados para transportar los datos de temperatura, presión y humedad.
extern int32_t temp;
extern uint32_t rh, press;

/******************** SECCIÓN DE LAS DEFINICIONES ********************/
#define BME280_DEV_ADDR 0x76 //Direccion por defecto para SDO -- GND
#define BME280_CHIP_ID 0x60  //ID del dispositivo

/**************************************************
 * DEFINICIÓN DE LAS DIRECCIONES DE LOS REGISTROS *
 *	 	 	 	 	 	 	 	 	 	 	 	  */

#define RESET_ADDR 0xE0 		//Registro del soft reset word.

#define ID_ADDR 0xD0			//Registro del número de ID del dispositivo.

#define STATUS_ADDR 0xF3		//Registro que contiene el estado del dispositivo.

#define CTRL_MEAS_ADDR 0xF4		//Registro de opciones de adquisición de temperatura y presión.
#define CTRL_HUM_ADDR 0xF2		//Registro de opciones de adquisición de humedad.
#define CTRL_CONFIG_ADDR 0xF5		//Registro de opciones para el rate, filtro e interfaz.

// Registros que contienen la medida en crudo de la presión. (MSB[19:12], LSB[11:4], XLSB[3:0])
#define PRESS_MSB_ADDR 0xF7
#define PRESS_LSB_ADDR 0xF8
#define PRESS_XLSB_ADDR 0xF9

// Registros que contienen la medida en crudo de la temperatura. (MSB[19:12], LSB[11:4], XLSB[3:0])
#define TEMP_MSB_ADDR 0xFA
#define TEMP_LSB_ADDR 0xFB
#define TEMP_XLSB_ADDR 0xFC

// Registros que contienen la medida en crudo de la humedad.(MSB[15:8], LSB[7:0])
#define HUM_MSB_ADDR 0xFD
#define HUM_LSB_ADDR 0xFE



/**	********************************************************
 * DEFINICIÓN DE LAS MÁSCARAS PARA EL CONTROL DE LOS DATOS *
 *	 					 	 	 						   */

#define CTRL_HUM_MASK 0x07;	//Máscara ctrl_hum

#define CTRL_MEAS_MSB_MASK 0xE0;  //Máscara ctrl_meas para los msb
#define CTRL_MEAS_LSB_MASK 0x1C;  //Máscara ctrl_meas para los lsb
#define CTRL_MEAS_XLSB_MASK 0x03; //Máscara ctrl_meas para los xlsb

#define CTRL_CONFIG_MSB_MASK 0xE0;  //Máscara ctrl_config para los msb
#define CTRL_CONFIG_LSB_MASK 0x1C;  //Máscara ctrl_config para los lsb
#define CTRL_CONFIG_3WSPI_MASK 0x01;		//Máscara ctrl_config para 3-wire SPI


/**	**********************************************************
 * DEFINICIÓN DE LAS MÁSCARAS PARA EL CONTROL DE LOS ESTADOS *
 *	 					 	 	 						     */
#define STATUS_MEASURING_MASK 0x08  //Máscara status para el estado de conversión.
#define STATUS_UPDATING_MASK 0x01   //Máscara status para el estado de actualización.


/**	**********************************************************
 * DEFINICIÓN DE LAS MÁSCARAS PARA EL CONTROL DE LAS MEDIDAS *
 *	 					 	 	 						     */
//Máscaras registro de temperatura
#define TEMP_MSB_MASK 0XFF0000
#define TEMP_LSB_MASK 0XFF00
#define TEMP_XLSB_MASK 0XF
#define TEMP_MASK 0x7FFFF

//Máscaras registro de presión
#define PRESS_MSB_MAASK 0XFF00000
#define PRESS_LSB_MASK 0XFF00
#define PRESS_XLSB_MASK 0XF
#define PRESS_MASK  0x7FFFF

//Máscaras registro de humedad
#define HUM_MSB_MASK 0XFF00
#define HUM_LSB_MASK 0XFF
#define HUM_MASK 0xFFFF


/**********************************************
 * DEFINICIÓN DE LOS VALORES DE CONFIGURACIÓN *
 */
// Control de oversampling (ctrl_hum y ctrl_meas[MSB, LSB])
#define OSRS_OVERSAMPLE_SKIP 0x00
#define OSRS_OVERSAMPLE_1 0x01
#define OSRS_OVERSAMPLE_2 0x02
#define OSRS_OVERSAMPLE_4 0x03
#define OSRS_OVERSAMPLE_8 0x04
#define OSRS_OVERSAMPLE_16 0x05

//Control del modo de medición (ctrl_meas[XLSB])
#define MODE_SLEEP 0x00
#define MODE_FORCED 0x01
//#define CTRL_MEAS_MODE_FORCED 0x02 //(Equivale al anterior)
#define MODE_NORMAL 0x03

//Control del tiempo inactivo en modo normal (ctrl_config[MSB])
#define T_STANBY_05 0X00
#define T_STANBY_62_5 0X01
#define T_STANBY_125 0X02
#define T_STANBY_250 0X03
#define T_STANBY_500 0X04
#define T_STANBY_1000 0X05
#define T_STANBY_10 0X06
#define T_STANBY_20 0X07

//Control del tiempo constante del filtro IIR (ctrl_config[LSB])
#define FILTER_OFF 0X00
#define FILTER_2 0X01
#define FILTER_4 0X02
#define FILTER_8 0X03
#define FILTER_16 0X04

//Control de activación del 3-wire SPI (ctrl_config[0])
#define WSPI3_DISABLE 0x00
#define WSPI3_ENABLE 0x01

/*********************************
 * DEFINICIÓN DE LOS VALORES DE RESET *
 */
#define SOFT_RESET_VAL 0xB6		//Valor para hacer un soft reset

#define HUM_LSB_RESET_VAL 0x00  //Valores de reset para los registros de medidas.
#define HUM_MSB_RESET_VAL 0x80
#define TEMP_XLSB_RESET_VAL 0x00
#define TEMP_LSB_RESET_VAL 0x00
#define TEMP_MSB_RESET_VAL 0X80
#define PRESS_XLSB_RESET_VAL 0X00
#define PRESS_LSB_RESET_VAL 0X00
#define PRESS_MSB_RESET_VAL 0x80

#define CTRL_CONFIG_RESET_VAL 0x00	//Valores de reset para los registros de configuración.
#define CTRL_MEAS_RESET_VAL 0x00
#define CTRL_HUM_RESET_VAL 0x00

#define STATUS_RESET_VAL 0x00

#define ID_RESET_VAL 0x60



/******************** SECCIÓN DE ESTRUCTURAS ********************/

/*														  *
 * ESTRUCTURAS SECUNDARIAS PARA EL MANEJO DEL DISPOSITIVO *
 *														  */
struct bme280_config{
	uint8_t ctrl_hum;			//Opciones de configuración de la humedad.
	uint8_t ctrl_meas;			//Opciones de configuración de la presión, temperatura y modo.
	uint8_t ctrl_config;		//Opciones de configuración del rate, filtro e interfaz.
};


struct bme280_meas_raw{
	int32_t press_raw;			//Registro del valor de temperatura en crudo
	int32_t temp_raw;			//Registro del valor de temperatura en crudo
	int16_t hum_raw;			//Registro del valor de humedad en crudo
};

struct bme280_calib{
	int32_t t_fine;				//Transporta la temperatura con buena resolución sobre el cálculo de humedad y presión.

	uint16_t dig_T1;			//Datos de calibración de temperatura
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;			//Datos de calibración de presión.
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;				//Datos de calibración de humedad.
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;

};

struct bme280_rslt{		//Medidas resultado
	int32_t T;
	uint32_t P;
	uint32_t rh;
};

/*											 		   *
 * ESTRUCTURA PRINCIPAL PARA EL MANEJADOR DEL DISPOSITIVO *
 *											 		   */
//Manejador del driver
typedef struct bme280_struct{

	uint8_t chip_id;					//Numero de identificación del dispositivo.

	uint8_t status;						//Estado del dispositivo.

	struct bme280_config config; 		//Estructura con la configuración del dispositivo.

	struct bme280_calib calib;			//Estructura con los datos de calibración y corrección de medidas.

	struct bme280_meas_raw meas_raw; 	//Estructura con los valores en crudo de las medidas.

	struct bme280_rslt rslt;					//Estructura con los datos resultados calculados.

}BME280_H;

/* Estados de retorno del driver */
typedef enum bme280_retval{
	BME280_OK,
	BME280_ERR
}BME280_STATUS;

/*										  *
 * PROTOTIPOS DE LAS FUNCIONES DEL DRIVER *
 *										  */
BME280_STATUS bme280_init(BME280_H * bme280_handler, I2C_HandleTypeDef * hi2c);
BME280_STATUS bme280_config(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c, uint8_t oversample_hum, uint8_t oversample_press, uint8_t oversample_temp, uint8_t mode_meas, uint8_t t_standby, uint8_t t_filt, uint8_t wspi3);
BME280_STATUS bme280_get_values(BME280_H * bme280_handler, I2C_HandleTypeDef* hi2c);

#endif



