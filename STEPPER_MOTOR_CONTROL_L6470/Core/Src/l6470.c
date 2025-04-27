/*
 * l6470.c
 *
 *  Created on: Dec 1, 2024
 *      Author: Jake
 */
#include <l6470.h>
#include <stdio.h>
#include <string.h>

/*
 * @brief enable l6470 motor driver
 * @param stepper_motor: stepper motor handler
 */
void l6470_enable(MotorSetTypedef* stepper_motor)
{
	uint8_t reg_temp[stepper_motor->num_motors];

	for(int i = 0; i < stepper_motor->num_motors; i++)
	{
		reg_temp[i] = RESET_DEVICE;
		HAL_Delay(10);
	}

	l6470_transmit_spi(stepper_motor, reg_temp, stepper_motor->num_motors);
	HAL_Delay(10);
}
/*
 * @brief disable l6470 motor driver
 * @param stepper_motor: stepper motor handler
 */
void l6470_disable(MotorSetTypedef* stepper_motor)
{
	uint8_t reg_temp[stepper_motor->num_motors];

	for(int i = 0; i < stepper_motor->num_motors; i++)
	{
		reg_temp[i] = SOFT_STOP;
	}

	l6470_transmit_spi(stepper_motor, reg_temp, stepper_motor->num_motors);
	HAL_Delay(10);
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
	for(int i = 0; i < stepper_motor->num_motors; i++)
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

	HAL_Delay(10);

	// Set STEP_MODE Microstepping (The default is already set at 1/128 step, but we will issue the command anyways for testing)
	reg_temp_1 = (uint8_t)ONE_HUNDRED_TWENTY_EIGHTH_STEP;
	l6470_set_param(stepper_motor, STEP_MODE, &reg_temp_1, 1);

	HAL_Delay(10);

	// Set the default ABS position to 0
	// set all three bytes to zero for the 22 bit field (the upper 2 bits are ignored)
	l6470_set_param(stepper_motor, ABS_POS, reg_temp_3, 3);

	HAL_Delay(10);

	// Set the default EL position to 0
	// set all three bytes to zero for the 22 bit field (the upper 2 bits are ignored)
	l6470_set_param(stepper_motor, EL_POS, reg_temp_3, 3);

	HAL_Delay(10);

	// Set the level for Holding Current
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_HOLD_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_HOLD, reg_temp_4, 1);

	HAL_Delay(10);

	// Set the Current level for running at constant speed
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_RUN_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_RUN, reg_temp_4, 1);

	HAL_Delay(10);

	// Set the Current level for acceleration
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_ACCDEC_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_ACC, reg_temp_4, 1);

	HAL_Delay(10);

	// Set the current level for decceleration
	reg_temp_4[0] = (uint8_t)((uint16_t)KVAL_ACCDEC_PERCENT * 255 / 100);
	l6470_set_param(stepper_motor, KVAL_DEC , reg_temp_4, 1);

	HAL_Delay(10);

	// set overcurrent threshold
	reg_temp_4[0] = (uint8_t)(MAX_CURRENT / 375) + 1;
	l6470_set_param(stepper_motor, OCD_TH, reg_temp_4, 1);

	HAL_Delay(10);

	// Set the CONFIG register with max torque slew rate (POW_SR = 00 = 320 V/µs)
	uint8_t config_val[2] = { 0x2E, 0x88 };  // MSB first: 0x2888
	l6470_set_param(stepper_motor, CONFIG, config_val, 4);

	HAL_Delay(10);

	// ToDo: Set the Config register....

	// initialize the spi buffers
	stepper_motor -> spi_dma_busy = 0;
	stepper_motor -> spi_tx_count = 0;

	for(int i = 0; i < stepper_motor->num_motors; i++)
	{
		stepper_motor->motors[i].stepper_id = i;
	}
	HAL_Delay(10);

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

    for (int i = 0; i < stepper_motor->num_motors; i++)
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
        stepper_motor->spd_tx_buffer[stepper_motor->num_motors + i]     = (uint8_t)(speed >> 16);
        stepper_motor->spd_tx_buffer[stepper_motor->num_motors * 2 + i] = (uint8_t)(speed >> 8);
        stepper_motor->spd_tx_buffer[stepper_motor->num_motors * 3 + i] = (uint8_t)(speed);
    }

    // Set SPI transmission buffer length and send data
    stepper_motor->spi_tx_buffer_length = 4; // * stepper_motor->num_motors; /////////////////////////////////////////////////////////////// was 4

//    for(int i = 0; i < (4 * stepper_motor->num_motors); i++)
//    {
//    	printf("stepper_motor->spd_tx_buffer[%d]: %02X\n\r\n\r", i,  stepper_motor->spd_tx_buffer[i]);
//    }
//
//    printf("stepper_motor->spi_tx_buffer_length: %d\n\r", stepper_motor->spi_tx_buffer_length);
//	//    printf("spd_tx_buffer LENGTH: %d\n\r", MAX_NUMBER_OF_MOTORS * SPI_TX_BUFFER_LENGTH);
//	//    printf("spi_tx_count: %d\n\r", stepper_motor->spi_tx_count);


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
	uint8_t *raw_value = &stepper_motor ->spd_rx_buffer[stepper_motor->num_motors];
	while(stepper_motor -> spi_dma_busy);
	for(int i =0; i< stepper_motor->num_motors; i++)
	{
		speed_abs_raw = (raw_value[i] << 16)|(raw_value[stepper_motor->num_motors + i] << 8)
								|(raw_value[2 * stepper_motor->num_motors + i]);
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
	for(int i = 0; i < stepper_motor->num_motors; i++)
	{
		stepper_motor -> spd_tx_buffer[i] = ABS_POS | 0x20;
		stepper_motor -> spd_tx_buffer[i + stepper_motor->num_motors] = NOP;
		stepper_motor -> spd_tx_buffer[i + 2 * stepper_motor->num_motors] = NOP;
		stepper_motor -> spd_tx_buffer[i + 3 * stepper_motor->num_motors] = NOP;
		stepper_motor -> spd_tx_buffer[i + 4 * stepper_motor->num_motors] = NOP;
	}
	stepper_motor -> spi_tx_buffer_length = 4;
	l6470_transmit_spi_dma(stepper_motor);
}


void l6470_set_param(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t *data, uint8_t data_length)
{
	uint8_t data_raw[stepper_motor->num_motors];

	for(int i = 0; i < stepper_motor->num_motors; i++)
	{
		data_raw[i] = param;
	}

	l6470_transmit_spi( stepper_motor, data_raw, stepper_motor->num_motors);
	HAL_Delay(10);

	for(int i = 0; i < data_length; i++)
	{
		for(int j = 0; j < stepper_motor->num_motors; j++)
		{
			data_raw[j] = *(data + i);
		}
		l6470_transmit_spi( stepper_motor, data_raw, stepper_motor->num_motors);
		HAL_Delay(10);
	}
}
//
//void l6470_set_param(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t *data, uint8_t data_length)
//{
//
//	uint8_t tx_data[4] = { 0 };
//
//	tx_data[3] = param;
//
//	// Copy value bytes into tx_data (data_length can be 1-3 bytes)
//	for(int i = 0; i < data_length; i++)
//	{
//		tx_data[i + 1] = data[i];
//	}
//
//	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
//
//	l6470_transmit_spi(stepper_motor, tx_data, data_length);
//
//	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
//	HAL_Delay(10);
//}

uint16_t l6470_get_param_1_Byte(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t data_length)
{
    // tx_data: sending the command byte (0x20 | param) for both motors in the chain
//    uint8_t tx_data[4] = { 0x20 | param, 0x20 | param, 0x00, 0x00 };  // Command for both L6470s (daisy chain)
//    uint8_t tx_data[4] = { (0x20 | param), (0x20 | param), (0x20 | param), (0x20 | param) };
    uint8_t tx_data[2] = { 0x2B, 0x2B };
	//uint8_t tx_data[2] = { 0x00 };
	uint8_t rx_data[2] = { 0x00 };

    // Step 1: Send command to both L6470s in the daisy chain
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);  // Select the motor (CS low)

    // Step 2: Transmit the command and receive the response (2 for each motor)
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx_data, rx_data, 2, 1000);  // 4 bytes: 2 for command, 2 for response data
    if (status != HAL_OK)
    {
        printf("SPI TRANSMIT ERROR: %02X\n\r", status);
        return -1;  // Return error if SPI transmission fails
    }

    // Step 3: CS must be pulled high to let the L6470 process the command and prepare the response
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);  // Deselect the motor (CS high)

    HAL_Delay(5);

    tx_data[0] = 0x00;
    tx_data[1] = 0x00;


    HAL_Delay(5);

    // Step 4: After CS is high, we need to pull CS low again before reading the response from the devices
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);  // Select motor again (CS low)

    // Step 5: Receive the response from both motors (daisy chain will shift out the response data)
    status = HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx_data, rx_data, 2, 1000);  // 4 bytes: Read back the response data
    if (status != HAL_OK)
    {
        printf("SPI RECEIVE ERROR: %02X\n\r", status);
        return -1;  // Return error if SPI receive fails
    }

    // Step 6: CS is pulled high after receiving the response
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);  // Deselect the motor (CS high)

    // Step 7: Combine the received bytes into 16-bit values for both motors
    uint16_t motor1_value = ((uint16_t)rx_data[2] << 8) | rx_data[3];  // Response for motor 1 (MSB and LSB)
    // uint16_t motor2_value = ((uint16_t)rx_data[2] << 8) | rx_data[3];  // Response for motor 2 (MSB and LSB)
//
//    // Debug: Print out the values for both motors (for verification)
//    printf("Motor 1 PARAM: 0x%04X\n", motor1_value);
//    printf("Motor 2 PARAM: 0x%04X\n", motor2_value);

    // Step 8: Return the value for motor 1 (or motor 2, depending on which you need)
    return motor1_value;  // Or return motor2_value if you want the second motor's value
}
/*
 * @brief receiving data through spi
 * @param stepper_motor: stepper motor handler
 * @param data: data pointer
 * @param data_length: data length in bytes
 */
void l6470_receive_spi(MotorSetTypedef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	uint8_t data_raw[data_length]; // NOT USED
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(stepper_motor ->hspi_l6470, data_raw, data, data_length, 1000);
	if(status != HAL_OK)
	{
		printf("SPI RECEIVE ERROR: %02X\n\r", status);
	}

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
}
/*
 * @brief transmitting data through spi
 * @param stepper_motor: stepper motor handler
 * @param data: data pointer
 * @param data_length: data length in bytes
 */
void l6470_transmit_spi(MotorSetTypedef* stepper_motor, uint8_t* data, uint8_t data_length)
{
	uint8_t receive_data[data_length]; // NOT USED
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(stepper_motor ->hspi_l6470, data, receive_data, data_length, 1000);
	if(status != HAL_OK)
	{
		printf("SPI TRANSMIT ERROR: %02X\n\r", status);
	}

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
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

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
//		printf("\n\r\n\r");

////		printf("SPI Handle Address: %p\n\r", (void *)stepper_motor->hspi_l6470);
////		printf("TX Buffer Addr: %p\n\r", (void *)(stepper_motor->spd_tx_buffer + stepper_motor->spi_tx_count * stepper_motor->num_motors));
////		printf("RX Buffer Addr: %p\n\r", (void *)(stepper_motor->spd_rx_buffer + stepper_motor->spi_tx_count * stepper_motor->num_motors));
//		printf("spi_tx_count: %d\n\r", stepper_motor->spi_tx_count);
////		printf("num_motors (Size): %d\n\r", stepper_motor->num_motors);

//		// Optional: Print the first few bytes of TX/RX buffers for verification
//		for (int i = 0; i < stepper_motor->num_motors; i++)
//		{
//		    printf("TX[%d]: 0x%02X, RX[%d]: 0x%02X\n\r",
//		           i, stepper_motor->spd_tx_buffer[stepper_motor->spi_tx_count * stepper_motor->num_motors + i],
//		           i, stepper_motor->spd_rx_buffer[stepper_motor->spi_tx_count * stepper_motor->num_motors + i]);
//		}
//
//		printf("\n\r\n\r\n\r");
		//////////////////////////////////////////////////////////////////////////////////////////////////////////

		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
				stepper_motor ->hspi_l6470,
				stepper_motor -> spd_tx_buffer + stepper_motor -> spi_tx_count * stepper_motor->num_motors,
				stepper_motor -> spd_rx_buffer + stepper_motor -> spi_tx_count * stepper_motor->num_motors,
				stepper_motor->num_motors);

		if(status != HAL_OK)
		{
			printf("SPI TRANSMIT DMA ERROR: %02X\n\r", status);
		}

		stepper_motor -> spi_tx_count++;
		stepper_motor -> spi_dma_busy = 1;
	}

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
}

uint16_t l6470_get_status(MotorSetTypedef* stepper_motor)
{
	uint8_t tx_data = GET_STATUS;
	uint8_t rx_data[2] = {0};

	// transmit command
	l6470_transmit_spi(stepper_motor, &tx_data, 2);
	HAL_Delay(10);

	// receive status register (2 bytes)
	l6470_receive_spi(stepper_motor, rx_data, 2);
	HAL_Delay(10);

	// Combine bytes into a 16-bit status register
	return ((uint16_t)rx_data[0] << 8) | rx_data[1];

}




