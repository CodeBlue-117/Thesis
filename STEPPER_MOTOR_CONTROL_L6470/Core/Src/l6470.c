/*
 * l6470.c
 *
 *  Created on: Dec 1, 2024
 *      Author: Jake
 */
#include <l6470.h>
#include <stdio.h>
#include <string.h>

// DEFINITIONS
// Optional: widths for readability
#define L6470_LEN1 1
#define L6470_LEN2 2
#define L6470_LEN3 3

/*
 * @brief enable l6470 motor driver
 * @param stepper_motor: stepper motor handler
 */
void l6470_enable(MotorSetTypedef* stepper_motor)
{
    uint8_t tx[2] = { SOFT_STOP, SOFT_STOP };
    uint8_t rx[2] = { 0 };

    if (stepper_motor->num_motors == 1)
        tx[1] = 0x00; // Only Motor1 gets the command

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

}
/*
 * @brief disable l6470 motor driver
 * @param stepper_motor: stepper motor handler
 */
void l6470_disable(MotorSetTypedef* stepper_motor)
{

    uint8_t tx[2] = { HARD_HIZ, HARD_HIZ };
    uint8_t rx[2] = { 0 };

    if (stepper_motor->num_motors == 1)
        tx[1] = 0x00; // Only Motor1 gets the command


    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

}

void l6470_reset(MotorSetTypedef* stepper_motor)
{

    uint8_t tx[2] = { RESET_DEVICE, RESET_DEVICE };
    uint8_t rx[2] = { 0 };

    if (stepper_motor->num_motors == 1)
        tx[1] = 0x00; // Only Motor1 gets the command

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

}

void l6470_soft_stop(MotorSetTypedef* stepper_motor)
{

    uint8_t tx[2] = { SOFT_STOP, SOFT_STOP };
    uint8_t rx[2] = { 0 };

    if (stepper_motor->num_motors == 1)
        tx[1] = 0x00; // Only Motor1 gets the command

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(1);

}


void l6470_sync_daisy_chain(MotorSetTypedef *stepper_motor)
{
    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0x00, 0x00};

    for (int i = 0; i < 3; i++)  // Send at least 3 frames of NOPs
    {
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
        HAL_Delay(2);
    }

    // printf("SPI Daisy-chain sync complete\n\r");
}

/*
 * @brief l6470 motor driver initialization
 * @param stepper_motor: stepper motor handler
 */
void l6470_init_chip_1(MotorSetTypedef* stepper_motor)
{
    uint8_t reg_temp_1;
    uint8_t reg_temp_2[2];
    uint8_t reg_temp_3[3] = {0, 0, 0};

    // Set STEP_MODE to 1/128 microstepping
    reg_temp_1 = (uint8_t)ONE_HUNDRED_TWENTY_EIGHTH_STEP;
    l6470_set_param_chip_1(stepper_motor, STEP_MODE, &reg_temp_1, 1);
    HAL_Delay(10);

    // Zero ABS_POS and EL_POS
    l6470_set_param_chip_1(stepper_motor, ABS_POS, reg_temp_3, 3);
    HAL_Delay(10);

    l6470_set_param_chip_1(stepper_motor, EL_POS, reg_temp_3, 3);
    HAL_Delay(10);

    // Set max ACC and DEC: 0x0FFE = 4094 (59559 step/s²)
    reg_temp_2[0] = 0x0F;
    reg_temp_2[1] = 0xFE;
    l6470_set_param_chip_1(stepper_motor, ACC, reg_temp_2, 2);
    HAL_Delay(10);
    l6470_set_param_chip_1(stepper_motor, DEC, reg_temp_2, 2);
    HAL_Delay(10);

    // Set current levels
    reg_temp_1 = (uint8_t)(KVAL_HOLD_PERCENT * 255 / 100);
    l6470_set_param_chip_1(stepper_motor, KVAL_HOLD, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = (uint8_t)(KVAL_RUN_PERCENT * 255 / 100);
    l6470_set_param_chip_1(stepper_motor, KVAL_RUN, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = (uint8_t)(KVAL_ACCDEC_PERCENT * 255 / 100);
    l6470_set_param_chip_1(stepper_motor, KVAL_ACC, &reg_temp_1, 1);
    HAL_Delay(10);
    l6470_set_param_chip_1(stepper_motor, KVAL_DEC, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set BEMF compensation slopes
    reg_temp_1 = 0x19;  // ST_SLP = 0.038% s/step
    l6470_set_param_chip_1(stepper_motor, ST_SLP, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = 0x29;  // FN_SLP_ACC = 0.063% s/step
    l6470_set_param_chip_1(stepper_motor, FN_SLP_ACC, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = 0x29;  // FN_SLP_DEC = 0.063% s/step
    l6470_set_param_chip_1(stepper_motor, FN_SLP_DEC, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set intersect speed to 0x0408 (≈61.5 steps/s)
    reg_temp_2[0] = 0x04;
    reg_temp_2[1] = 0x08;
    l6470_set_param_chip_1(stepper_motor, INT_SPEED, reg_temp_2, 2);
    HAL_Delay(10);

    // Set overcurrent threshold for 1A (OCD_TH = 1)
    reg_temp_1 = 0x08; // was 0x01
    l6470_set_param_chip_1(stepper_motor, OCD_TH, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set CONFIG register: 0x2E88 → internal oscillator, 2MHz, OC shutdown, slew rate = 320V/μs
    reg_temp_2[0] = 0x2E;
    reg_temp_2[1] = 0x80;
    l6470_set_param_chip_1(stepper_motor, CONFIG, reg_temp_2, 2);
    HAL_Delay(10);

    // Initialize SPI buffers
    stepper_motor->spi_dma_busy = 0; // TODO: Unused?
    stepper_motor->spi_tx_count = 0; // TODO: Unused?

    HAL_Delay(10);
}

/*
 * @brief l6470 motor driver initialization
 * @param stepper_motor: stepper motor handler
 */
void l6470_init_chip_2(MotorSetTypedef* stepper_motor)
{
    uint8_t reg_temp_1;
    uint8_t reg_temp_2[2];
    uint8_t reg_temp_3[3] = {0, 0, 0};

    // Set STEP_MODE to 1/128 microstepping
    reg_temp_1 = (uint8_t)ONE_HUNDRED_TWENTY_EIGHTH_STEP;
    l6470_set_param_chip_2(stepper_motor, STEP_MODE, &reg_temp_1, 1);
    HAL_Delay(10);

    // Zero ABS_POS and EL_POS
    l6470_set_param_chip_2(stepper_motor, ABS_POS, reg_temp_3, 3);
    HAL_Delay(10);
    l6470_set_param_chip_2(stepper_motor, EL_POS, reg_temp_3, 3);
    HAL_Delay(10);

    // Set max ACC and DEC: 0x0FFE = 4094 (59559 step/s²)
    reg_temp_2[0] = 0x0F;
    reg_temp_2[1] = 0xFE;
    l6470_set_param_chip_2(stepper_motor, ACC, reg_temp_2, 2);
    HAL_Delay(10);
    l6470_set_param_chip_2(stepper_motor, DEC, reg_temp_2, 2);
    HAL_Delay(10);

    // Set current levels
    reg_temp_1 = (uint8_t)(KVAL_HOLD_PERCENT * 255 / 100);
    l6470_set_param_chip_2(stepper_motor, KVAL_HOLD, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = (uint8_t)(KVAL_RUN_PERCENT * 255 / 100);
    l6470_set_param_chip_2(stepper_motor, KVAL_RUN, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = (uint8_t)(KVAL_ACCDEC_PERCENT * 255 / 100);
    l6470_set_param_chip_2(stepper_motor, KVAL_ACC, &reg_temp_1, 1);
    HAL_Delay(10);
    l6470_set_param_chip_2(stepper_motor, KVAL_DEC, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set BEMF compensation slopes
    reg_temp_1 = 0x19;  // ST_SLP = 0.038% s/step
    l6470_set_param_chip_2(stepper_motor, ST_SLP, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = 0x29;  // FN_SLP_ACC = 0.063% s/step
    l6470_set_param_chip_2(stepper_motor, FN_SLP_ACC, &reg_temp_1, 1);
    HAL_Delay(10);

    reg_temp_1 = 0x29;  // FN_SLP_DEC = 0.063% s/step
    l6470_set_param_chip_2(stepper_motor, FN_SLP_DEC, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set intersect speed to 0x0408 (≈61.5 steps/s)
    reg_temp_2[0] = 0x04;
    reg_temp_2[1] = 0x08;
    l6470_set_param_chip_2(stepper_motor, INT_SPEED, reg_temp_2, 2);
    HAL_Delay(10);

    // Set overcurrent threshold for 1A (OCD_TH = 1)
    reg_temp_1 = 0x08; // was 0x01
    l6470_set_param_chip_2(stepper_motor, OCD_TH, &reg_temp_1, 1);
    HAL_Delay(10);

    // Set CONFIG register: 0x2E88 → internal oscillator, 2MHz, OC shutdown, slew rate = 320V/μs
    reg_temp_2[0] = 0x2E;
    reg_temp_2[1] = 0x80;
    l6470_set_param_chip_2(stepper_motor, CONFIG, reg_temp_2, 2);
    HAL_Delay(10);

    // Initialize SPI buffers
    stepper_motor->spi_dma_busy = 0; // TODO: Unused?
    stepper_motor->spi_tx_count = 0; // TODO: Unused?

    HAL_Delay(10);
}


/*
 * @brief transmitting data through spi
 * @param stepper_motor: stepper motor handler
 * @param data: data pointer
 * @param data_length: data length in bytes
 */
void l6470_transmit_spi(MotorSetTypedef* stepper_motor, uint8_t* data, uint8_t data_length)
{

	// TODO: Find the minimum delay necessary for this function to work
	uint8_t receive_data[data_length]; // NOT USED

	for (int i = 0; i < 4 * stepper_motor->num_motors; i+=2)
	{

		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
		// HAL_Delay(1);

		HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, &data[i], receive_data, 2, 1000);

		// HAL_Delay(1);
		HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
		// HAL_Delay(1);

	}

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
	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(stepper_motor ->hspi_l6470, data_raw, data, data_length, 1000);
	if(status != HAL_OK)
	{
		printf("SPI RECEIVE ERROR: %02X\n\r", status);
	}

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
}


void l6470_transmit_spi_dma(MotorSetTypedef* stepper_motor)
{

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);

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

	HAL_GPIO_WritePin(stepper_motor ->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);

}


/* @brief This function is to set rotational velocity at radians per angle
 * @param stepper motor Stepper motor handler
 * @param vel Velocity at radians per sec 62.8 - 2pi
 * @retval None
 */

// Convert angular velocity (rad/s) to L6470 SPEED register units
//
// vel[i] ............ Motor angular velocity in radians per second (rad/s).
// STEPS_PER_REVOLUTION = 200 for a standard 1.8° stepper.
//
// The L6470 SPEED register stores motor speed in units of steps/tick,
// where 1 tick = 250 ns and the value is represented as an unsigned 0.28 fixed-point number.
//
// Therefore:
//    steps_per_second = vel[i] * (STEPS_PER_REVOLUTION / (2π))
//
// To convert from steps/s → SPEED register value (steps/tick × 2^28):
//    SPEED = steps_per_second × tick × 2^28
//           = steps_per_second × (250e-9 s) × 2^28
//           = steps_per_second × 67.108864
//
// Combining everything yields:
//    SPEED = vel[i] * STEPS_PER_REVOLUTION * 67.108864 / (2π)
//
// The result must be an unsigned 20-bit integer (0–1,048,575).
// Any higher value is clipped internally by MAX_SPEED.
//
// Example:
//    vel = 31.4159 rad/s (≈ 5 rev/s)
//    SPEED = 31.4159 × 200 × 67.108864 / (2π) ≈ 67,000
//    -> roughly 5 revolutions per second.

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

    l6470_transmit_spi(stepper_motor, stepper_motor->spd_tx_buffer, sizeof(stepper_motor->spd_tx_buffer));
}

// Configure both motors on chip 1
void l6470_set_param_chip_1(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t *value, uint8_t length)
{
    if (length < 1 || length > 3)
    {
        printf("SET_PARAM: Invalid length: %d\n\r", length);
        return;
    }

    uint8_t tx[2] = { param, param };  // Motor2 CMD, Motor1 CMD
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    for (int i = 0; i < length; i++)
    {
        tx[0] = value[i];
        tx[1] = value[i];  // MSB first
        rx[0] = rx[1] = 0;

        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
        HAL_Delay(2);
    }
}

// Configure one motor on chip 2
void l6470_set_param_chip_2(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t *value, uint8_t length)
{
    if (length < 1 || length > 3)
    {
        printf("SET_PARAM: Invalid length: %d\n\r", length);
        return;
    }

    uint8_t tx[2] = { param, 0x00};
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    for (int i = 0; i < length; i++)
    {
        tx[0] = value[i]; // MSB first
        tx[1] = 0x00;
        rx[0] = rx[1] = 0;

        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
        HAL_Delay(2);
    }
}

// Get params from both motors on chip 1
void l6470_get_param_chip_1(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t length)
{
    if (length < 1 || length > 3)
    {
        printf("GET_PARAM: Invalid length: %d\n\r", length);
        return;
    }

    uint8_t tx[2] = { 0x20 | param, 0x20 | param };
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    uint32_t result_motor1 = 0;
    uint32_t result_motor2 = 0;

    for (int i = 0; i < length; i++)
    {
        tx[0] = tx[1] = 0;
        rx[0] = rx[1] = 0;

        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
        HAL_Delay(2);

        result_motor1 = (result_motor1 << 8) | rx[0];  // Motor1 (first in chain)
        result_motor2 = (result_motor2 << 8) | rx[1];  // Motor2 (second in chain)

        // printf("Byte %d - Motor1: 0x%04X  Motor2: 0x%04X\n\r", i, rx[0], rx[1]);
    }

     printf("Chip1: Motor1 result: 0x%06lX\n\r", result_motor1);
     printf("Chip1: Motor2 result: 0x%06lX\n\r", result_motor2);
}


// Get params from one motor on chip 2
uint32_t l6470_get_param_chip_2(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t length)
{
    if (length < 1 || length > 3)
    {
        printf("GET_PARAM: Invalid length: %d\n\r", length);
        return 0xFFFFFFFF;
    }

    uint8_t tx[2] = { 0x20 | param, 0x00 };
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    uint32_t result = 0;
    for (int i = 0; i < length; i++)
    {
        tx[0] = tx[1] = 0;
        rx[0] = rx[1] = 0;

        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
        HAL_Delay(2);

        result = (result << 8) | rx[0];
        // printf("Byte %d read: 0x%04X\n\r", i, rx[1]);
    }

    printf("Chip2: Motor1 result: 0x%06lX\n\r", result);
    return result;
}


void l6470_get_status(MotorSetTypedef* stepper_motor, uint16_t* m1_status, uint16_t* m2_status)
{
    uint8_t tx[2] = { 0xD0, 0xD0 }; // GET_STATUS command for both
    uint8_t rx[2] = { 0 };

    *m1_status = 0;
    *m2_status = 0;

    // Send GET_STATUS command
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
    HAL_Delay(1);
    HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    // Now read 2 bytes (MSB first) from each motor
    for (int i = 0; i < 2; i++)
    {
        tx[0] = 0;
        tx[1] = 0;
        rx[0] = 0;
        rx[1] = 0;

        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(stepper_motor->hspi_l6470, tx, rx, 2, 1000);
        HAL_Delay(1);
        HAL_GPIO_WritePin(stepper_motor->gpio_cs_port, stepper_motor->gpio_cs_pin, GPIO_PIN_SET);
        HAL_Delay(2);

        *m1_status = (*m1_status << 8) | rx[0];
        *m2_status = (*m2_status << 8) | rx[1];
    }

    if(stepper_motor->identifier == 1)
    {
        printf("Chip1: Motor1 STATUS = 0x%04X\n\r", *m1_status);
        printf("Chip1: Motor2 STATUS = 0x%04X\n\r", *m2_status);
    }
    else if(stepper_motor->identifier == 2)
    {
        printf("Chip2: Motor1 STATUS = 0x%04X\n\r", *m1_status);
        printf("Chip2: Motor2 STATUS = 0x%04X\n\r", *m2_status);
    }

}

//////////////////////////

void l6470_dump_params_chip1(MotorSetTypedef* stepper_motor)
{
    printf("\n=== Chip1 Registers ===\n\r");

    printf("STEP_MODE\n\r");
    l6470_get_param_chip_1(stepper_motor, STEP_MODE, L6470_LEN1);
    printf("\n\r");

    printf("ACC\n\r");
    l6470_get_param_chip_1(stepper_motor, ACC, L6470_LEN2);
    printf("\n\r");

    printf("DEC\n\r");
    l6470_get_param_chip_1(stepper_motor, DEC, L6470_LEN2);
    printf("\n\r");

    printf("MAX_SPEED\n\r");
    l6470_get_param_chip_1(stepper_motor, MAX_SPEED, L6470_LEN2);
    printf("\n\r");

    printf("KVAL_HOLD\n\r");
    l6470_get_param_chip_1(stepper_motor, KVAL_HOLD, L6470_LEN1);
    printf("\n\r");

    printf("KVAL_RUN\n\r");
    l6470_get_param_chip_1(stepper_motor, KVAL_RUN, L6470_LEN1);
    printf("\n\r");

    printf("KVAL_ACC\n\r");
    l6470_get_param_chip_1(stepper_motor, KVAL_ACC, L6470_LEN1);
    printf("\n\r");

    printf("KVAL_DEC\n\r");
    l6470_get_param_chip_1(stepper_motor, KVAL_DEC, L6470_LEN1);
    printf("\n\r");

    printf("ST_SLP\n\r");
    l6470_get_param_chip_1(stepper_motor, ST_SLP, L6470_LEN1);
    printf("\n\r");

    printf("FN_SLP_ACC\n\r");
    l6470_get_param_chip_1(stepper_motor, FN_SLP_ACC, L6470_LEN1);
    printf("\n\r");

    printf("FN_SLP_DEC\n\r");
    l6470_get_param_chip_1(stepper_motor, FN_SLP_DEC, L6470_LEN1);
    printf("\n\r");

    printf("OCD_TH\n\r");
    l6470_get_param_chip_1(stepper_motor, OCD_TH, L6470_LEN1);
    printf("\n\r");

    printf("CONFIG\n\r");
    l6470_get_param_chip_1(stepper_motor, CONFIG, L6470_LEN2);
    printf("\n\r");

    // From your init()
    printf("INT_SPEED\n\r");
    l6470_get_param_chip_1(stepper_motor, INT_SPEED, L6470_LEN2);
    printf("\n\r");

    printf("ABS_POS\n\r");
    l6470_get_param_chip_1(stepper_motor, ABS_POS, L6470_LEN3);
    printf("\n\r");

    printf("EL_POS\n\r");
    l6470_get_param_chip_1(stepper_motor, EL_POS, L6470_LEN2);
    printf("\n\r");

    // Added extras
    printf("MIN_SPEED\n\r");
    l6470_get_param_chip_1(stepper_motor, MIN_SPEED, L6470_LEN2);
    printf("\n\r");

    printf("ALARM_EN\n\r");
    l6470_get_param_chip_1(stepper_motor, ALARM_EN, L6470_LEN1);
    printf("\n\r");

    printf("K_THERM\n\r");
    l6470_get_param_chip_1(stepper_motor, K_THERM, L6470_LEN1);
    printf("\n\r");

    printf("STALL_TH\n\r");
    l6470_get_param_chip_1(stepper_motor, STALL_TH, L6470_LEN1);
    printf("\n\r");

    printf("ADC_OUT\n\r");
    l6470_get_param_chip_1(stepper_motor, ADC_OUT, L6470_LEN1);
    printf("\n\r");
}

///////////

void l6470_dump_params_chip2(MotorSetTypedef* stepper_motor)
{
    printf("\n=== Chip2 Registers ===\n\r");

    printf("STEP_MODE\n\r");
    l6470_get_param_chip_2(stepper_motor, STEP_MODE, L6470_LEN1);
    printf("\n\r");

    printf("ACC\n\r");
    l6470_get_param_chip_2(stepper_motor, ACC, L6470_LEN2);
    printf("\n\r");

    printf("DEC\n\r");
    l6470_get_param_chip_2(stepper_motor, DEC, L6470_LEN2);
    printf("\n\r");

    printf("MAX_SPEED\n\r");
    l6470_get_param_chip_2(stepper_motor, MAX_SPEED, L6470_LEN2);
    printf("\n\r");

    printf("KVAL_HOLD\n\r");
    l6470_get_param_chip_2(stepper_motor, KVAL_HOLD, L6470_LEN1);
    printf("\n\r");

    printf("KVAL_RUN\n\r");
    l6470_get_param_chip_2(stepper_motor, KVAL_RUN, L6470_LEN1);
    printf("\n\r");

    printf("KVAL_ACC\n\r");
    l6470_get_param_chip_2(stepper_motor, KVAL_ACC, L6470_LEN1);
    printf("\n\r");

    printf("KVAL_DEC\n\r");
    l6470_get_param_chip_2(stepper_motor, KVAL_DEC, L6470_LEN1);
    printf("\n\r");

    printf("ST_SLP\n\r");
    l6470_get_param_chip_2(stepper_motor, ST_SLP, L6470_LEN1);
    printf("\n\r");

    printf("FN_SLP_ACC\n\r");
    l6470_get_param_chip_2(stepper_motor, FN_SLP_ACC, L6470_LEN1);
    printf("\n\r");

    printf("FN_SLP_DEC\n\r");
    l6470_get_param_chip_2(stepper_motor, FN_SLP_DEC, L6470_LEN1);
    printf("\n\r");

    printf("OCD_TH\n\r");
    l6470_get_param_chip_2(stepper_motor, OCD_TH, L6470_LEN1);
    printf("\n\r");

    printf("CONFIG\n\r");
    l6470_get_param_chip_2(stepper_motor, CONFIG, L6470_LEN2);
    printf("\n\r");

    // From your init()
    printf("INT_SPEED\n\r");
    l6470_get_param_chip_2(stepper_motor, INT_SPEED, L6470_LEN2);
    printf("\n\r");

    printf("ABS_POS\n\r");
    l6470_get_param_chip_2(stepper_motor, ABS_POS, L6470_LEN3);
    printf("\n\r");

    printf("EL_POS\n\r");
    l6470_get_param_chip_2(stepper_motor, EL_POS, L6470_LEN2);
    printf("\n\r");

    // Added extras
    printf("MIN_SPEED\n\r");
    l6470_get_param_chip_2(stepper_motor, MIN_SPEED, L6470_LEN2);
    printf("\n\r");

    printf("ALARM_EN\n\r");
    l6470_get_param_chip_2(stepper_motor, ALARM_EN, L6470_LEN1);
    printf("\n\r");

    printf("K_THERM\n\r");
    l6470_get_param_chip_2(stepper_motor, K_THERM, L6470_LEN1);
    printf("\n\r");

    printf("STALL_TH\n\r");
    l6470_get_param_chip_2(stepper_motor, STALL_TH, L6470_LEN1);
    printf("\n\r");

    printf("ADC_OUT\n\r");
    l6470_get_param_chip_2(stepper_motor, ADC_OUT, L6470_LEN1);
    printf("\n\r");
}


////////////////////////////////////////////



