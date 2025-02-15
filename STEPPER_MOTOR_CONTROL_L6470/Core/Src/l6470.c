/*
 * l6470.c
 *
 *  Created on: Dec 1, 2024
 *      Author: yerke
 */
#include <l6470.h>



static void l6470_receive_spi(MotorSetTypedef* stepper_motor, uint8_t* data, uint8_t data_length);
static void l6470_set_param(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t *data, uint8_t data_length);
static void l6470_transmit_spi(MotorSetTypedef* stepper_motor, uint8_t* data, uint8_t data_length);
/*
 * @brief enable l6470 motor driver
 * @param stepper_motor: stepper motor handler
 */
void l6470_enable(MotorSetTypedef* stepper_motor)
{
	uint8_t reg_temp[NUMBER_OF_MOTORS];

	for(int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		reg_temp[i] = RESET_DEVICE;
	}

	l6470_transmit_spi(stepper_motor, &reg_temp, NUMBER_OF_MOTORS);
}
/*
 * @brief disable l6470 motor driver
 * @param stepper_motor: stepper motor handler
 */
void l6470_disable(MotorSetTypedef* stepper_motor)
{
	uint8_t reg_temp[NUMBER_OF_MOTORS];

	for(int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		reg_temp[i] = SOFT_STOP;
	}

	l6470_transmit_spi(stepper_motor, reg_temp, NUMBER_OF_MOTORS);
}

/*
 * @brief l6470 motor driver initialization
 * @param stepper_motor: stepper motor handler
 */
void l6470_init(MotorSetTypedef* stepper_motor)
{
	uint8_t reg_temp[4];

	for(int i = 0; i <= NUMBER_OF_MOTORS; i++)
	{
		stepper_motor ->motors[i].speed_pos.steps_per_rev = STEPS_PER_REVOLUTION;
	}
	// reset the driver
	HAL_GPIO_WritePin(stepper_motor -> gpio_rst_port, stepper_motor -> gpio_rst_number, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(stepper_motor ->gpio_rst_port, stepper_motor -> gpio_rst_number, GPIO_PIN_SET);
	HAL_Delay(100);
	l6470_disable(stepper_motor);

	// set KVAL values
	reg_temp[0] = (uint8_t)((uint16_t)KVAL_HOLD_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_HOLD, reg_temp, 1);
	reg_temp[0] = (uint8_t)((uint16_t)KVAL_RUN_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_RUN, reg_temp, 1);
	reg_temp[0] = (uint8_t)((uint16_t)KVAL_ACCDEC_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_ACC, reg_temp, 1);
	reg_temp[0] = (uint8_t)((uint16_t)KVAL_ACCDEC_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_ACC, reg_temp, 1);

	// set overcurrent threshold
	reg_temp[0] = (uint8_t)(MAX_CURRENT / 375) + 1;
	l6470_set_param(stepper_motor, OCD_TH, reg_temp, 1);

	// ToDo: set microstepping and disbale step clock mode

	// initialize the spi buffers
	stepper_motor -> spi_dma_busy = 0;
	stepper_motor -> spi_tx_count = 0;
	for(int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		stepper_motor->motors[i].stepper_id = i;
	}



}

/* @brief This function is to set rotational velocity at radians per angle
 * @param stepper motor Stepper motor handler
 * @param vel Velocity at radians per sec 62.8 - 2pi
 * @retval None
 */
void l6470_set_vel(MotorSetTypedef* stepper_motor, float* vel)
{
	uint32_t speed;
	for(int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		if(*(vel + i) > MAX_SPEED_RAD)
		{
			*(vel + i) = MAX_SPEED_RAD;
		}
		else if(*(vel + i) < -MAX_SPEED_RAD)
		{
			*(vel + i) = -MAX_SPEED_RAD;
		}
		if(*(vel + i) > 0)
		{
			stepper_motor -> spd_tx_buffer[i] = (0x51);
		}
		else
		{
			*(vel + i) = -(*(vel + i));
			stepper_motor -> spd_tx_buffer[i] = (0x50);
		}
		speed = (uint32_t)(*(vel + i) * STEPS_PER_REVOLUTION * 67.108864f / TWOPI);
		stepper_motor -> spd_tx_buffer[NUMBER_OF_MOTORS + i] = (uint8_t)(speed >> 16);
		stepper_motor -> spd_tx_buffer[NUMBER_OF_MOTORS * 2 + i] = (uint8_t)(speed >> 8);
		stepper_motor -> spd_tx_buffer[NUMBER_OF_MOTORS * 3 + i] = (uint8_t)(speed);
	}
	stepper_motor -> spi_tx_buffer_length = 4;
	l6470_transmit_spi_dma(stepper_motor);
}

/*
 * @brief updates the stepper motor position (radians)
 * @param stepper_motor: stepper motor handler
 */
void l6470_get_speed_pos(MotorSetTypedef* stepper_motor)
{
	int32_t speed_abs_raw;
	int32_t speed_raw;
	uint8_t *raw_value = &stepper_motor ->spd_rx_buffer[NUMBER_OF_MOTORS];
	while(stepper_motor -> spi_dma_busy);
	for(int i =0; i< NUMBER_OF_MOTORS; i++)
	{
		speed_abs_raw = (raw_value[i] << 16)|(raw_value[NUMBER_OF_MOTORS + i] << 8)
								|(raw_value[2 * NUMBER_OF_MOTORS + i]);
		if(speed_abs_raw &(1 << 21))
		{
			speed_abs_raw -= 2 * (1 << 21);
		}
		speed_raw = (speed_abs_raw) % (STEPS_PER_REVOLUTION * MICROSTEPPING);
		if(speed_raw < -(STEPS_PER_REVOLUTION * MICROSTEPPING) / 2)
		{
			speed_raw += (STEPS_PER_REVOLUTION * MICROSTEPPING);
		}
		else if(speed_raw > (STEPS_PER_REVOLUTION * MICROSTEPPING) / 2)
		{
			speed_raw -= (STEPS_PER_REVOLUTION * MICROSTEPPING);
		}
		stepper_motor->motors[i].speed_pos.rad_pos = speed_raw;
	}
	for(int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		stepper_motor -> spd_tx_buffer[i] = ABS_POS | 0x20;
		stepper_motor -> spd_tx_buffer[i + NUMBER_OF_MOTORS] = NOP;
		stepper_motor -> spd_tx_buffer[i + 2 * NUMBER_OF_MOTORS] = NOP;
		stepper_motor -> spd_tx_buffer[i + 3 * NUMBER_OF_MOTORS] = NOP;
		stepper_motor -> spd_tx_buffer[i + 4 * NUMBER_OF_MOTORS] = NOP;
	}
	stepper_motor -> spi_tx_buffer_length = 4;
	l6470_transmit_spi_dma(stepper_motor);
}
static void l6470_set_param(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t *data, uint8_t data_length)
{
	uint8_t data_raw[NUMBER_OF_MOTORS];
	for(int i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		data_raw[i] = param;
	}
	l6470_transmit_spi( stepper_motor, data_raw,NUMBER_OF_MOTORS);
	for(int i = 0; i < data_length; i++)
	{
		for(int j = 0; j < NUMBER_OF_MOTORS; j++)
		{
			data_raw[j] = *(data + i);
		}
		l6470_transmit_spi( stepper_motor, data_raw, NUMBER_OF_MOTORS);
	}
}

/*
 * @brief receiving data through spi
 * @param stepper_motor: stepper motor handler
 * @param data: data pointer
 * @param data_length: data length in bytes
 */
static void l6470_receive_spi(MotorSetTypedef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	uint8_t data_raw[data_length];
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(stepper_motor ->hspi_l6470, data_raw, data, data_length, 1000);
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
}
/*
 * @brief transmitting data through spi
 * @param stepper_motor: stepper motor handler
 * @param data: data pointer
 * @param data_length: data length in bytes
 */
static void l6470_transmit_spi(MotorSetTypedef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	uint8_t receive_data[40];
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(stepper_motor ->hspi_l6470, data, receive_data, data_length, 1000);
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
}

void l6470_transmit_spi_dma(MotorSetTypedef* stepper_motor)
{
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
	if(stepper_motor -> spi_tx_count == stepper_motor -> spi_tx_buffer_length)
	{
		stepper_motor -> spi_tx_count = 0;
		stepper_motor -> spi_dma_busy = 0;
	}
	else
	{
		HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);

		HAL_SPI_TransmitReceive_DMA(stepper_motor ->hspi_l6470, stepper_motor -> spd_tx_buffer +
				stepper_motor -> spi_tx_count * NUMBER_OF_MOTORS, stepper_motor -> spd_rx_buffer +
				stepper_motor -> spi_tx_count * NUMBER_OF_MOTORS, NUMBER_OF_MOTORS);
		stepper_motor -> spi_tx_count++;
		stepper_motor -> spi_dma_busy = 1;
	}
}

uint16_t l6470_get_status(MotorSetTypedef* stepper_motor)
{
	uint8_t tx_data = GET_STATUS;
	uint8_t rx_data[2] = {0};

	// transmit command
	l6470_transmit_spi(stepper_motor, &tx_data, 2);

	// receive status register (2 bytes)
	l6470_receive_spi(stepper_motor, rx_data, 2);

	// Combine bytes into a 16-bit status register
	return ((uint16_t)rx_data[0] << 8) | rx_data[1];

}




