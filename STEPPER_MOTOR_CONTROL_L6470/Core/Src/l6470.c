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

	uint32_t conf = l6470_get_param(stepper_motor, CONFIG, 2);  // ✅ Uses the argument passed in
	printf("CONFIG read in enable(): 0x%04lX\n\r", conf);
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
    uint8_t reg_temp_2[2];
    uint8_t reg_temp_3[3] = {0, 0, 0};

    // Set the number of steps per revolution for each motor
    for (int i = 0; i < stepper_motor->num_motors; i++)
    {
        stepper_motor->motors[i].speed_pos.steps_per_rev = STEPS_PER_REVOLUTION;
    }

    // Reset the driver TODO: verify that this is correct because red light turns on here
    HAL_GPIO_WritePin(stepper_motor->gpio_rst_port, stepper_motor->gpio_rst_number, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(stepper_motor->gpio_rst_port, stepper_motor->gpio_rst_number, GPIO_PIN_SET);
    HAL_Delay(100);

    // Disable the driver
    l6470_disable(stepper_motor);

    // Enable all alarms
    reg_temp_1 = 0xFF;
    l6470_set_param(stepper_motor, ALARM_EN, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set STEP_MODE to 1/128 microstepping
    reg_temp_1 = (uint8_t)ONE_HUNDRED_TWENTY_EIGHTH_STEP;
    l6470_set_param(stepper_motor, STEP_MODE, &reg_temp_1, 1);
    HAL_Delay(10);

    // Zero ABS_POS and EL_POS
    l6470_set_param(stepper_motor, ABS_POS, reg_temp_3, 3);
    HAL_Delay(10);
    l6470_set_param(stepper_motor, EL_POS, reg_temp_3, 3);
    HAL_Delay(10);

    // Set current levels
    reg_temp_1 = (uint8_t)(KVAL_HOLD_PERCENT * 255 / 100);
    l6470_set_param(stepper_motor, KVAL_HOLD, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = (uint8_t)(KVAL_RUN_PERCENT * 255 / 100);
    l6470_set_param(stepper_motor, KVAL_RUN, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = (uint8_t)(KVAL_ACCDEC_PERCENT * 255 / 100);
    l6470_set_param(stepper_motor, KVAL_ACC, &reg_temp_1, 1);
    HAL_Delay(10);
    l6470_set_param(stepper_motor, KVAL_DEC, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set BEMF compensation slopes
    reg_temp_1 = 0x19;  // ST_SLP = 0.038% s/step
    l6470_set_param(stepper_motor, ST_SLP, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = 0x29;  // FN_SLP_ACC = 0.063% s/step
    l6470_set_param(stepper_motor, FN_SLP_ACC, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = 0x29;  // FN_SLP_DEC = 0.063% s/step
    l6470_set_param(stepper_motor, FN_SLP_DEC, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set intersect speed to 0x0408 (≈61.5 steps/s)
    reg_temp_2[0] = 0x04;
    reg_temp_2[1] = 0x08;
    l6470_set_param(stepper_motor, INT_SPEED, reg_temp_2, 2);
    HAL_Delay(10);

    // Set overcurrent threshold for 1A (OCD_TH = 1)
    reg_temp_1 = 0x01;
    l6470_set_param(stepper_motor, OCD_TH, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set CONFIG register: 0x2E88 → internal oscillator, 2MHz, OC shutdown, slew rate = 320V/μs
    reg_temp_2[0] = 0x2E;
    reg_temp_2[1] = 0x80;
    l6470_set_param(stepper_motor, CONFIG, reg_temp_2, 2);
    HAL_Delay(10);

    // Initialize SPI buffers
    stepper_motor->spi_dma_busy = 0; // TODO: Unused?
    stepper_motor->spi_tx_count = 0; // TODO: Unused?

    for (int i = 0; i < stepper_motor->num_motors; i++)
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

        // printf("SPEED: %lu\n\r", speed);

        // Store speed data in the transmission buffer
        stepper_motor->spd_tx_buffer[stepper_motor->num_motors + i]     = (uint8_t)(speed >> 16);
        stepper_motor->spd_tx_buffer[stepper_motor->num_motors * 2 + i] = (uint8_t)(speed >> 8);
        stepper_motor->spd_tx_buffer[stepper_motor->num_motors * 3 + i] = (uint8_t)(speed);
    }

//    for (int i = 0; i < 4 * stepper_motor->num_motors; i++)
//    {
//        printf("TX[%d] = 0x%02X\n\r", i, stepper_motor->spd_tx_buffer[i]);
//    }

    l6470_transmit_spi(stepper_motor, stepper_motor->spd_tx_buffer, sizeof(stepper_motor->spd_tx_buffer));
    // l6470_transmit_spi_dma(stepper_motor);
}

void l6470_soft_stop(MotorSetTypedef* stepper_motor)
{
	uint8_t softStopCommand = 0xB0;

	uint8_t tx_buffer[2] = { softStopCommand, softStopCommand };

	l6470_transmit_spi(stepper_motor, tx_buffer, 2);

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

void l6470_set_param(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t *value, uint8_t length)
{
    if (length < 1 || length > 3)
    {
        printf("SET_PARAM: Invalid length: %d\n\r", length);
        return;
    }

    uint8_t tx[2] = { 0x00, param };  // Motor2 NOP, Motor1 CMD
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
    HAL_Delay(2);

    for (int i = 0; i < length; i++)
    {
        tx[0] = 0x00;  // TODO: Check to make sure this works with a single chip and double chip config
        tx[1] = value[i];  // MSB first
        rx[0] = rx[1] = 0;

        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
        HAL_Delay(2);
    }
}

uint32_t l6470_get_param(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t length)
{
    if (length < 1 || length > 3)
    {
        printf("GET_PARAM: Invalid length: %d\n\r", length);
        return 0xFFFFFFFF;
    }

    uint8_t tx[2] = { 0x00, 0x20 | param };
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
    HAL_Delay(2);

    uint32_t result = 0;
    for (int i = 0; i < length; i++)
    {
        tx[0] = tx[1] = 0;
        rx[0] = rx[1] = 0;

        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
        HAL_Delay(2);

        result = (result << 8) | rx[1];
        printf("Byte %d read: 0x%02X\n\r", i, rx[1]);
    }

    // printf("Raw result: 0x%06lX\n\r", result);
    return result;
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

	for (int i = 0; i < 4 * stepper_motor->num_motors; i+=2)
	{

		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);
		HAL_Delay(1);

		HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, &data[i], receive_data, 2, 1000);

		HAL_Delay(1);
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);
		HAL_Delay(1);

	}

}

void l6470_transmit_spi_dma(MotorSetTypedef* stepper_motor)
{

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_RESET);

	stepper_motor -> spi_dma_busy = 1;

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
			stepper_motor ->hspi_l6470,
			stepper_motor -> spd_tx_buffer,
			stepper_motor -> spd_rx_buffer,
			4 * stepper_motor->num_motors);

	if(status != HAL_OK)
	{
		printf("SPI TRANSMIT DMA ERROR: %02X\n\r", status);
	}

	stepper_motor -> spi_tx_count++;
	stepper_motor -> spi_dma_busy = 0;

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_number, GPIO_PIN_SET);

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




