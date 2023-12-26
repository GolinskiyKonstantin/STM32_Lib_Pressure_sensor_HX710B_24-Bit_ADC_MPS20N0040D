/*
	
  ******************************************************************************
  * @file 			( фаил ):   HX710B.h
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

#ifndef _HX710B_H
#define _HX710B_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------*/

#include "main.h"

// Напряжение: 3,3-5 В
// Диапазон измеряемого давления: 0-40 кПа
// Сопротивление: 4...6 кОм
// АЦП: 24 бит

// Датчик давления MPS20N0040D представляет собой мостовую схему, в которой одним из мостовых 
// элементов является чувствительный к давлению резистор. 
// Исходное напряжение подается на мост в точках 2 и 5, а выходной сигнал измеряется в точках 1 или 6 и 4.

//=========  setup =========================================================

// пин HX710B_OUT настраиваем как вход с подтяжкой к питанию input pin ( input pull_up )
// пин HX710B_SCK настраиваем как выход


// данные для расчета глубины погружения трубки в воду ( измерение уровня воды )
#define FLUID_DENSITY									(997.0)			// плотность жидкости -> вода( 997 кг/м3)
#define AIR_DENSITY 									(1.225)			// плотность воздуха( 1,225 кг/м3)
#define ACCELERATION_OF_GRAVITY 			(9.81)			// ускорения свободного падения ( g ≈ 9,81 м/с2 )

// кол-во замеров
#define READ_TIMES			20
//==========================================================================



typedef enum:uint8_t{
	HX710B_NOT_READY,
	HX710B_READY
}HX710B_status;

typedef struct{
	GPIO_TypeDef* port;
	uint16_t pin;
}HX710B_pins;

typedef struct{
		HX710B_pins sck;		// Power Down and Serial Clock Input Pin
		HX710B_pins dout;		// Serial Data Output Pin
		uint8_t gain;				// (default -> 3)
		int32_t offset;			// used for tare weight
		float scale;				// used to return weight in grams, kg, ounces, whatever (default -> 1.0)
		float coeff;				// tuning factor
}HX710B_sensor;




uint8_t 	HX710B_is_ready(HX710B_sensor *sensor);
void 			HX710B_setGain(HX710B_sensor *sensor, uint8_t gain);
int32_t 	HX710B_read(HX710B_sensor *sensor);
void 			HX710B_wait_ready(HX710B_sensor *sensor, uint32_t delay_ms);
uint8_t 	HX710B_wait_ready_retry(HX710B_sensor *sensor, int32_t retries, uint32_t delay_ms);
uint8_t 	HX710B_wait_ready_timeout(HX710B_sensor *sensor, uint32_t timeout, uint32_t delay_ms);
int32_t 	HX710B_read_average(HX710B_sensor *sensor, uint8_t times);
float 		HX710B_pascal(HX710B_sensor *sensor);
float 		HX710B_atm(HX710B_sensor *sensor);
float 		HX710B_mmHg(HX710B_sensor *sensor);
float 		HX710B_psi(HX710B_sensor *sensor);
float 		HX710B_bar(HX710B_sensor *sensor);
double 		HX710B_get_value(HX710B_sensor *sensor, uint8_t times);
float 		HX710B_get_units(HX710B_sensor *sensor, uint8_t times);
void 			HX710B_tare(HX710B_sensor *sensor, uint8_t times);
void 			HX710B_set_scale(HX710B_sensor *sensor, float scale);
float 		HX710B_get_scale(HX710B_sensor *sensor);
void 			HX710B_set_offset(HX710B_sensor *sensor, int32_t offset);
int32_t 	HX710B_get_offset(HX710B_sensor *sensor);
void 			HX710B_set_coeff(HX710B_sensor *sensor, float coeff);
float 		HX710B_get_coeff(HX710B_sensor *sensor);
void 			HX710B_power_down(HX710B_sensor *sensor);
void 			HX710B_power_up(HX710B_sensor *sensor);
uint32_t 	HX710B_get_millimeters(HX710B_sensor *sensor);



//------------------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif

#endif	/*	_HX710B_H */

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
