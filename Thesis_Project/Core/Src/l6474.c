/*
 * l6474.c
 *
 *  Created on: Jan 28, 2025
 *      Author: jake-
 */
#include "main.h"
#include "l6474.h"
#include "stdio.h"


static void l6474_receive_spi(l6474TypeDef* stepper_motor, uint8_t* data, uint8_t data_length);
static void l6474_transmit_spi(l6474TypeDef* stepper_motor, uint8_t* data, uint8_t data_length);


// Enable the l6474 motor driver
void l6474_enable(l6474TypeDef* stepper_motor)
{
	uint8_t reg_temp;
	reg_temp = ENABLE_STEPPER;
	l6474_transmit_spi(stepper_motor, &reg_temp, 1);
}


// Disable the l6474 stepper driver
void l6474_disable(l6474TypeDef* stepper_motor)
{
	uint8_t reg_temp;
	reg_temp = DISABLE_STEPPER;
	l6474_transmit_spi(stepper_motor, &reg_temp, 1);
}


// l6474 motor driver initialization
void l6474_init(l6474TypeDef* stepper_motor)
{
	uint8_t reg_temp[4];
	stepper_motor->speed_pos.steps_per_rev = STEPS_PER_REVOLUTION;

	// reset the driver
	HAL_GPIO_WritePin(stepper_motor->gpio_rst_port, stepper_motor->gpio_rst_number, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(stepper_motor->gpio_rst_port, stepper_motor->gpio_rst_number, GPIO_PIN_SET);
	HAL_Delay(100);

	// Set the reference current
	reg_temp[0] = TVAL;
	reg_temp[1] = (uint8_t)(CURRENT_REFERENCE / 0.03125f) + 1;  // ToDo: this is set for a 350 mA stepper, need to change CURRENT REFERENCE for my stepper motor
	l6474_transmit_spi(stepper_motor, reg_temp, 2);

	// Set the max current
	reg_temp[0] = OCD_TH;
	reg_temp[1] = (uint8_t)(MAX_CURRENT / 0.375f) + 1; // ToDo: Change MAX_CURRENT for my stepper motor
	l6474_transmit_spi(stepper_motor, reg_temp, 2);

}


// Set the rotational velocity in radians per second (Example: setting vel = 6.28 = 1 rotation per second, 62.8 = 10 revolution per second)
void l6474_sel_vel(l6474TypeDef* stepper_motor, float vel)
{
	if(vel > 0)
	{
		if(stepper_motor->speed_pos.rad_speed <= 0)
		{
			HAL_GPIO_WritePin(stepper_motor -> gpio_dir_port,
								stepper_motor -> gpio_dir_number, GPIO_PIN_SET);
		}
		stepper_motor->speed_pos.rad_speed = vel;
		l6474_set_steppersec(stepper_motor, (uint16_t)(vel * STEPS_PER_REVOLUTION / TWOPI));
	}
	else
	{
		if(stepper_motor->speed_pos.rad_speed >= 0)
		{
			HAL_GPIO_WritePin(stepper_motor->gpio_dir_port,
					stepper_motor -> gpio_dir_number, GPIO_PIN_RESET);
		}
		stepper_motor->speed_pos.rad_speed = vel;
		vel = -vel;
		l6474_set_steppersec(stepper_motor, (uint16_t)(vel * STEPS_PER_REVOLUTION / TWOPI));
	}
}


// read the status of the l6474
void l6474_read_status(l6474TypeDef* stepper_motor, uint16_t* status)
{
	uint8_t reg_temp = STATUS_STEPPER;
	l6474_transmit_spi(stepper_motor, &reg_temp, 1);
	l6474_receive_spi(stepper_motor, (uint8_t *)status, 2);
}


// Set the number of steps per second that the stepper does (input param: step = number of steps per second)
void l6474_set_steppersec(l6474TypeDef* stepper_motor, uint16_t step)
{
	uint16_t arr = TIMER_FREQUENCY / step;
	stepper_motor->tim_handler->Instance->ARR = (TIMER_FREQUENCY / step);
	stepper_motor->tim_handler->Instance->CCR2 = arr / 2;
}


// Updates the stepper motor position in radians
void l6474_get_speed_pos(l6474TypeDef* stepper_motor)
{
	int32_t speed_raw;
	uint8_t reg_temp, raw_value[3];
	reg_temp = ABS_POS | 0x20;
	l6474_transmit_spi(stepper_motor, &reg_temp, 1);
	l6474_receive_spi(stepper_motor, raw_value, 3);
	speed_raw = (((int32_t)(raw_value[0] << 26) | (raw_value[1] << 18) | (raw_value[1] << 10)) >> 10) % STEPS_PER_REVOLUTION; // ToDo: The last raw_value[0] might be raw_value[1] repeated

	if(speed_raw < 0)
	{
		speed_raw += STEPS_PER_REVOLUTION;
	}

	stepper_motor->speed_pos.rad_pos = (TWOPI * (float)speed_raw) / (float)STEPS_PER_REVOLUTION;
}


// Receive data through SPI
void l6474_receive_spi(l6474TypeDef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	uint8_t data_raw = 0;
	for(int i = 0; i < data_length; i++)
	{
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(stepper_motor->hspi_l6474, &data_raw, data + i, 1, 1000);
		if(status != HAL_OK)
		{
			printf("Error receiving SPI\n\r");
			while(1);
		}

		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
	}
}


// Transmit data through SPI
void l6474_transmit_spi(l6474TypeDef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	for(int i = 0; i < data_length; i++)
	{
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
		HAL_StatusTypeDef status =  HAL_SPI_Transmit(stepper_motor->hspi_l6474, data + i, 1, 1000);
		if(status != HAL_OK)
		{
			printf("Error receiving SPI\n\r");
			while(1);
		}
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);

	}
}
