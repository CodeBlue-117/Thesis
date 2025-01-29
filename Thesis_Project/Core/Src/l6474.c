/*
 * l6474.c
 *
 *  Created on: Jan 28, 2025
 *      Author: jake-
 */
#include "main.h"
#include "l6474.h"


void l6474_set_vel(l6474TypeDef* stepper_motor, float vel)
{
	if(vel > 0)
	{
		if(stepper_motor->speed_pos.rad_speed <= 0)
		{

		}
	}
}


void l6474_read_status(l6474TypeDef* stepper_motor, uint16_t* status)
{
	uint8_t reg_temp = STATUS_STEPPER;
	l6474_transmit_spi(stepper_motor, &reg_temp, 1);
	l6474_receive_spi(stepper_motor, (uint8_t *)status, 2);
}


void l6474_set_steppersec(l6474TypeDef* stepper_motor, uint16_t step)
{
	uint16_t arr = TIMER_FREQUENCY / step;
	stepper_motor.tim_handler->Instance->ARR = (TIMER_FREQUENCY / step);
	stepper_motor.tim_handler->Instance->CCR2 = arr / 2;
}


void l6474_get_speed_pos(l6474TypeDef* stepper_motor)
{
	int32_t speed_raw;
	uint8_t reg_temp, raw_value[3];
	reg_temp = ABS_POS | 0x20;
	l6474_transmit_spi(stepper_motor, &reg_temp, 1);
	l6474_receive_spi(stepper_motor, raw_value, 3);
	speed_raw = (((int32_t)(raw_value[0] << 26) | (raw_value[1] << 18) | (raw_value[0] << 10)) >> 10) % STEPS_PER_REVOLUTION; // The last raw_value[0] might be raw_value[1] repeated

	if(speed_raw < 0)
	{
		speed_raw += STEPS_PER_REVOLUTION;
	}

	stepper_motor->speed_pos.rad_pos = (TWOPI * (float)speed_raw) / (float)STEPS_PER_REVOLUTION;

}


void l6474_receive_spi(l6474TypeDef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	uint8_t data_raw = 0;
	for(int i = 0; i < data_length; i++)
	{
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(stepper_motor->hspi_l6474, &data_raw, data + i, 1, 1000);
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);

	}
}


void l6474_transmit_spi(l6474TypeDef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	for(int i = 0; i < data_length; i++)
	{
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
		HAL_SPI_Transmit(stepper_motor->hspi_l6474, data + i, 1, 1000);
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);

	}
}
