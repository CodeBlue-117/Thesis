/*
 * l6470.c
 *
 *  Created on: Dec 1, 2024
 *      Author: Jake
 */
#include <l6470.h>
#include <stdio.h>
#include <string.h>



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

	l6470_transmit_spi(stepper_motor, reg_temp, NUMBER_OF_MOTORS);
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

	uint8_t reg_temp_1;
	uint8_t reg_temp_3[3] = {0, 0, 0};
	uint8_t reg_temp_4[4];

	// Set the number of steps per revolution for each motor
	for(int i = 0; i <= NUMBER_OF_MOTORS; i++)
	{
		stepper_motor ->motors[i].speed_pos.steps_per_rev = STEPS_PER_REVOLUTION;
	}

	// reset the driver
	HAL_GPIO_WritePin(stepper_motor -> gpio_rst_port, stepper_motor -> gpio_rst_number, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(stepper_motor ->gpio_rst_port, stepper_motor -> gpio_rst_number, GPIO_PIN_SET);
	HAL_Delay(100);

	// disable the driver
	l6470_disable(stepper_motor);

	// Enable all alarms
	reg_temp_1 = 0xFF;
	l6470_set_param(stepper_motor, ALARM_EN, &reg_temp_1, 1);

	// Set STEP_MODE Microstepping (The default is already set at 1/128 step, but we will issue the command anyways for testing)
	reg_temp_1 = (uint8_t)ONE_HUNDRED_TWENTY_EIGHTH_STEP;
	l6470_set_param(stepper_motor, STEP_MODE, &reg_temp_1, 1);

	// Set the default ABS position to 0
	// set all three bytes to zero for the 22 bit field (the upper 2 bits are ignored)
	l6470_set_param(stepper_motor, ABS_POS, reg_temp_3, 3);

	// Set the default EL position to 0
	// set all three bytes to zero for the 22 bit field (the upper 2 bits are ignored)
	l6470_set_param(stepper_motor, EL_POS, reg_temp_3, 3);

	// Set the level for Holding Current
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_HOLD_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_HOLD, reg_temp_4, 1);

	// Set the Current level for running at constant speed
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_RUN_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_RUN, reg_temp_4, 1);

	// Set the Current level for acceleration
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_ACCDEC_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_ACC, reg_temp_4, 1);

	// Set the current level for decceleration
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_ACCDEC_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_DEC , reg_temp_4, 1);

	// set overcurrent threshold
	reg_temp_4[0] = (uint8_t)(MAX_CURRENT / 375) + 1;
	l6470_set_param(stepper_motor, OCD_TH, reg_temp_4, 1);

	// ToDo: Set the Config register....

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
// Cleaner Code
void l6470_set_vel(MotorSetTypedef* stepper_motor, float* vel)
{
    uint32_t speed;

    for (int i = 0; i < NUMBER_OF_MOTORS; i++)
    {
        // Clamp velocity to the allowable range
        if (vel[i] > MAX_SPEED_RAD)
        {
            vel[i] = MAX_SPEED_RAD;
        }
        else if (vel[i] < -MAX_SPEED_RAD)
        {
            vel[i] = -MAX_SPEED_RAD;
        }

        // Determine motor direction and store the corresponding command
        if (vel[i] > 0)
        {
            stepper_motor->spd_tx_buffer[i] = 0x51; // Forward direction
        }
        else
        {
            vel[i] = -vel[i]; // Convert negative speed to positive magnitude
            stepper_motor->spd_tx_buffer[i] = 0x50; // Reverse direction
        }

        // Convert velocity to stepper motor speed format
        speed = (uint32_t)(vel[i] * STEPS_PER_REVOLUTION * 67.108864f / TWOPI);

        // Store speed data in the transmission buffer
        stepper_motor->spd_tx_buffer[NUMBER_OF_MOTORS + i]     = (uint8_t)(speed >> 16);
        stepper_motor->spd_tx_buffer[NUMBER_OF_MOTORS * 2 + i] = (uint8_t)(speed >> 8);
        stepper_motor->spd_tx_buffer[NUMBER_OF_MOTORS * 3 + i] = (uint8_t)(speed);
    }

    // Set SPI transmission buffer length and send data
    stepper_motor->spi_tx_buffer_length = 4 * NUMBER_OF_MOTORS; /////////////////////////////////////////////////////////////// was 4

    for(int i = 0; i < 16; i++)
    {
    	printf("stepper_motor->spd_tx_buffer[%d]: %02X\n\r", i,  stepper_motor->spd_tx_buffer[i]);
    }

    printf("stepper_motor->spi_tx_buffer_length: %d\n\r", stepper_motor->spi_tx_buffer_length);
    printf("spd_tx_buffer LENGTH: %d\n\r", NUMBER_OF_MOTORS * SPI_TX_BUFFER_LENGTH);
    printf("spi_tx_count: %d\n\r", stepper_motor->spi_tx_count);


    l6470_transmit_spi_dma(stepper_motor);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

//void l6470_set_single_motor_vel(MotorSetTypedef* stepper_motor, float vel)
//{
//    uint32_t speed;
//
//    // 1. Determine direction and set motor control byte
//    if (vel >= 0)
//    {
//        stepper_motor->spd_tx_buffer[0] = 0x51; // Clockwise (CW) direction
//    }
//    else
//    {
//        stepper_motor->spd_tx_buffer[0] = 0x50; // Counterclockwise (CCW) direction
//        vel = -vel; // Make velocity positive for speed calculation
//    }
//
//    // 2. Clamp velocity to max speed (MAX_SPEED_RAD)
//    if (vel > MAX_SPEED_RAD)
//    {
//        vel = MAX_SPEED_RAD;
//    }
//
//    // 3. Convert velocity to motor speed (step per second)
//    speed = (uint32_t)(vel * STEPS_PER_REVOLUTION * 67.108864f / TWOPI);
//
//    // 4. Store the speed value in the transmission buffer (3 bytes per motor)
//    stepper_motor->spd_tx_buffer[1] = (uint8_t)(speed >> 16);
//    stepper_motor->spd_tx_buffer[2] = (uint8_t)(speed >> 8);
//    stepper_motor->spd_tx_buffer[3] = (uint8_t)(speed);
//
//    // 5. Set SPI buffer length for one motor
//    stepper_motor->spi_tx_buffer_length = 4;
//
//    // 6. Transmit the data via SPI (assuming DMA is configured correctly)
//    l6470_transmit_spi_dma(stepper_motor);
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//void rotate_motor_individually(MotorSetTypedef* stepper_motor, uint8_t motor_num, float vel)
//{
//    uint32_t speed;
//
//    // Ensure velocity is within bounds
//    if (vel > MAX_SPEED_RAD)
//    {
//        vel = MAX_SPEED_RAD;
//    }
//    else if (vel < -MAX_SPEED_RAD)
//    {
//        vel = -MAX_SPEED_RAD;
//    }
//
//    // Clear buffer before modifying
//    memset(stepper_motor->spd_tx_buffer, 0, sizeof(stepper_motor->spd_tx_buffer));
//
//    // Motor 1
//    if (motor_num == 1)
//    {
//        // Set direction
//        if (vel > 0)
//        {
//            stepper_motor->spd_tx_buffer[0] = 0x51; // CW
//        }
//        else
//        {
//            vel = -vel;  // Convert to positive for speed calculation
//            stepper_motor->spd_tx_buffer[0] = 0x50; // CCW
//        }
//
//        // Compute speed
//        speed = (uint32_t)(vel * STEPS_PER_REVOLUTION * 67.108864f / TWOPI);
//
//        // Store speed bytes for motor 1
//        stepper_motor->spd_tx_buffer[3] = (uint8_t)(speed >> 16);
//        stepper_motor->spd_tx_buffer[4] = (uint8_t)(speed >> 8);
//        stepper_motor->spd_tx_buffer[5] = (uint8_t)(speed);
//    }
//
//    // Motor 2
//    else if (motor_num == 2)
//    {
//        // Set direction
//        if (vel > 0)
//        {
//            stepper_motor->spd_tx_buffer[1] = 0x51; // CW
//        }
//        else
//        {
//            vel = -vel;  // Convert to positive for speed calculation
//            stepper_motor->spd_tx_buffer[1] = 0x50; // CCW
//        }
//
//        // Compute speed
//        speed = (uint32_t)(vel * STEPS_PER_REVOLUTION * 67.108864f / TWOPI);
//
//        // Store speed bytes for motor 2
//        stepper_motor->spd_tx_buffer[6] = (uint8_t)(speed >> 16);
//        stepper_motor->spd_tx_buffer[7] = (uint8_t)(speed >> 8);
//        stepper_motor->spd_tx_buffer[8] = (uint8_t)(speed);
//    }
//
//    // Motor 3
//    else if (motor_num == 3)
//    {
//        // Set direction
//        if (vel > 0)
//        {
//            stepper_motor->spd_tx_buffer[2] = 0x51; // CW
//        }
//        else
//        {
//            vel = -vel;  // Convert to positive for speed calculation
//            stepper_motor->spd_tx_buffer[2] = 0x50; // CCW
//        }
//
//        // Compute speed
//        speed = (uint32_t)(vel * STEPS_PER_REVOLUTION * 67.108864f / TWOPI);
//
//        // Store speed bytes for motor 3
//        stepper_motor->spd_tx_buffer[9] = (uint8_t)(speed >> 16);
//        stepper_motor->spd_tx_buffer[10] = (uint8_t)(speed >> 8);
//        stepper_motor->spd_tx_buffer[11] = (uint8_t)(speed);
//    }
//
//    else // Invalid motor number, handle this case
//    {
//        printf("Invalid motor number\n\r");
//        while(1)
//        {
//            ; // Stay stuck here
//        }
//    }
//
//    // Set SPI transmission length to cover all motors' data (4 bytes per motor)
//    stepper_motor->spi_tx_buffer_length = 4; // 3 motors, 12 bytes total
//
//    // Transmit the data via SPI
//    l6470_transmit_spi_dma(stepper_motor);
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(stepper_motor ->hspi_l6470, data_raw, data, data_length, 1000);
	if(status != HAL_OK)
	{
		printf("SPI RECEIVE ERROR!!!!!\n\r");
		while(1)
		{
			;
		}
	}
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
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(stepper_motor ->hspi_l6470, data, receive_data, data_length, 1000);
	if(status != HAL_OK)
	{
		printf("SPI TRANSMIT ERROR!!!!!\n\r");
		while(1)
		{
			;
		}
	}
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

		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(stepper_motor ->hspi_l6470, stepper_motor -> spd_tx_buffer +
				stepper_motor -> spi_tx_count * NUMBER_OF_MOTORS, stepper_motor -> spd_rx_buffer +
				stepper_motor -> spi_tx_count * NUMBER_OF_MOTORS, NUMBER_OF_MOTORS);
		if(status != HAL_OK)
			{
				printf("SPI TRANSMIT DMA ERROR!!!!!\n\r");
				while(1)
				{
					;
				}
			}
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




