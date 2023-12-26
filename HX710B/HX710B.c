/*

  ******************************************************************************
  * @file 			( фаил ):   HX710B.c
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

/* Includes ----------------------------------------------------------*/
#include "HX710B.h"



#define LSBFIRST 				0
#define MSBFIRST 				1




static void HX710B_delay(uint32_t ms)
{
	HAL_Delay ( ms );
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
static void HX710B_delay_us(uint32_t us)
{
  uint32_t delay = us;
  while (delay > 0)
  {
    delay--;
    __nop(); __nop(); __nop(); __nop();    
  }
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
static uint8_t shiftInSlow(HX710B_pins dataPin, HX710B_pins clockPin, uint8_t bitOrder)
{
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        HAL_GPIO_WritePin(clockPin.port, clockPin.pin, GPIO_PIN_SET );   //HIGH
        HX710B_delay_us(1);
        if(bitOrder == LSBFIRST){
            value |= HAL_GPIO_ReadPin(dataPin.port, dataPin.pin) << i;
				}
        else{
            value |= HAL_GPIO_ReadPin(dataPin.port, dataPin.pin) << (7 - i);
				}
        HAL_GPIO_WritePin(clockPin.port, clockPin.pin, GPIO_PIN_RESET );   //LOW
        HX710B_delay_us(1);
    }
    return value;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
HX710B_status HX710B_is_ready(HX710B_sensor *sensor)
{
	return HAL_GPIO_ReadPin(sensor->dout.port, sensor->dout.pin) == GPIO_PIN_RESET;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_setGain(HX710B_sensor *sensor, uint8_t gain)
{
	switch(gain)
	{
		case 128:		// channel A, gain factor 128
				sensor->gain = 1;
				break;
		case 64:		// channel A, gain factor 64
				sensor->gain = 3;
				break;
		case 32:		// channel B, gain factor 32
				sensor->gain = 2;
				break;
		default:
				break;
	}
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
int32_t HX710B_read(HX710B_sensor *sensor) 
{
	// Wait for the chip to become ready.
	HX710B_wait_ready(sensor, 0);

	// Define structures for reading data into.
	uint32_t value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	// Pulse the clock pin 24 times to read the data.
	data[2] = shiftInSlow(sensor->dout, sensor->sck, MSBFIRST);
	data[1] = shiftInSlow(sensor->dout, sensor->sck, MSBFIRST);
	data[0] = shiftInSlow(sensor->dout, sensor->sck, MSBFIRST);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (uint8_t i = 0; i < sensor->gain; i++) {
		HAL_GPIO_WritePin(sensor->sck.port, sensor->sck.pin, GPIO_PIN_SET );   //HIGH
		HX710B_delay_us(1);
		HAL_GPIO_WritePin(sensor->sck.port, sensor->sck.pin, GPIO_PIN_RESET );   //LOW
		HX710B_delay_us(1);
	}

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}
	
	// Construct a 32-bit signed integer
	value = ( (filler) << 24
						| (data[2]) << 16
						| (data[1]) << 8
						| (data[0]) );

	return (int32_t)value;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_wait_ready(HX710B_sensor *sensor, uint32_t delay_ms)
{
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (HX710B_NOT_READY == HX710B_is_ready(sensor)) {
		HX710B_delay(delay_ms);
	}
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
uint8_t HX710B_wait_ready_retry(HX710B_sensor *sensor, int32_t retries, uint32_t delay_ms)
{
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	int32_t count = 0;
	while (count < retries) {
		if (HX710B_is_ready(sensor) == HX710B_READY) {
			return 1;
		}
		HX710B_delay(delay_ms);
		count++;
	}
	return 0;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
uint8_t HX710B_wait_ready_timeout(HX710B_sensor *sensor, uint32_t timeout, uint32_t delay_ms)
{
	// Wait for the chip to become ready until timeout.
	uint32_t millisStarted = HAL_GetTick();
	while (HAL_GetTick() - millisStarted < timeout) {
		if (HX710B_is_ready(sensor) == HX710B_READY) {
			return 1;
		}
		HX710B_delay(delay_ms);
	}
	return 0;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
int32_t HX710B_read_average(HX710B_sensor *sensor, uint8_t times) 
{
	int32_t sum = 0;
	for (uint8_t i = 0; i < times; i++) {
		sum += HX710B_read(sensor);
		HX710B_delay(0);
	}
	return sum / times;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_pascal(HX710B_sensor *sensor)
{
		float value = (HX710B_read_average(sensor, READ_TIMES) - sensor->offset ) / sensor->coeff;
    return value;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_atm(HX710B_sensor *sensor)
{
    float value = HX710B_pascal(sensor) * 9.8692326671601E-6;
    return value;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_mmHg(HX710B_sensor *sensor)
{
    float value = HX710B_pascal(sensor) * 0.0075006376;
    return value;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_psi(HX710B_sensor *sensor)
{
    float value = HX710B_pascal(sensor) * 0.0001450377;
    return value;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_bar(HX710B_sensor *sensor)
{
    float value = HX710B_pascal(sensor) * 1.0E-5;
    return value;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
double HX710B_get_value(HX710B_sensor *sensor, uint8_t times)
{
	return HX710B_read_average(sensor, times) - sensor->offset;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_get_units(HX710B_sensor *sensor, uint8_t times)
{
	return HX710B_get_value(sensor, times) / sensor->scale;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_tare(HX710B_sensor *sensor, uint8_t times)
{
	double sum = HX710B_read_average(sensor, times);
	HX710B_set_offset(sensor, sum);
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_set_scale(HX710B_sensor *sensor, float scale) 
{
	sensor->scale = scale;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_get_scale(HX710B_sensor *sensor)
{
	return sensor->scale;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_set_offset(HX710B_sensor *sensor, int32_t offset)
{
	sensor->offset = offset;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
int32_t HX710B_get_offset(HX710B_sensor *sensor)
{
	return sensor->offset;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_set_coeff(HX710B_sensor *sensor, float coeff)
{
	sensor->coeff = coeff;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
float HX710B_get_coeff(HX710B_sensor *sensor)
{
	return sensor->coeff;
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_power_down(HX710B_sensor *sensor)
{
	HAL_GPIO_WritePin(sensor->sck.port, sensor->sck.pin, GPIO_PIN_RESET );   //LOW
	HAL_GPIO_WritePin(sensor->sck.port, sensor->sck.pin, GPIO_PIN_SET );   	//HIGH
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
void HX710B_power_up(HX710B_sensor *sensor)
{
	HAL_GPIO_WritePin(sensor->sck.port, sensor->sck.pin, GPIO_PIN_RESET );   //LOW
}
//--------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------
// P = (p2-p1)*g*h		// результат в паскалях Pa
// где p2 — плотность жидкости(кг/м3), p1 — плотность воздуха(кг/м3), h — высота жидкости в метрах (м), а g — ускорение свободного падения .
// плотности воды ( ρ2 = 997 кг/м3 ) и воздуха ( ρ1 = 1,225 кг/м3 ), а также ускорения свободного падения ( g ≈ 9,81 м/с2 ).
uint32_t HX710B_get_millimeters(HX710B_sensor *sensor)
{
	uint32_t millimeters = ( HX710B_pascal(sensor) / ((FLUID_DENSITY - AIR_DENSITY) * ACCELERATION_OF_GRAVITY) ) * 1000.0;
	return millimeters;
}
//--------------------------------------------------------------------------------------------


/************************ (C) COPYRIGHT GKP *****END OF FILE****/
