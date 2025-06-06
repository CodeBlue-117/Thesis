/*
 * l6474.h
 *
 *  Created on: Jan 28, 2025
 *      Author: jake-
 */

#ifndef INC_L6474_H_
#define INC_L6474_H_

#include "main.h"

#define MAX_ACCELERATION		60			// ??? Max acceleration per second ^ 2
#define MAX_SPEED				200			// ??? Max speed per second
#define MAX_CURRENT				0.5			// Max current of the stepper motor
#define CURRENT_REFERENCE		0.30		// Current reference of the stepper motor
#define STEPS_PER_REVOLUTION	3200	    // Steps per revolution 16 (microsteps per step) * 200 (steps per rev)
#define TWOPI					6.283185f   // Two Pi value

#define ABS_POS					0x01		// Current position
#define EL_POS					0x02		// Electrical position
#define MARK					0x03		// Mark position
#define TVAL					0x09		// Reference current
#define T_FAST					0x0E		// Fast decay/ Fall step time
#define TON_MIN					0x0F		// Minimum on time
#define TOFF_MIN				0x10		// Minimum off time
#define ADC_OUT					0x12		// ADC output
#define OCD_TH					0x13		// OCD threshold
#define STEP_MODE				0x16		// Step mode
#define ALARM_EN				0x17		// alarms enabled
#define CONFIG					0x18		// CONFIG REGISTER

#define ENABLE_STEPPER			0xB8
#define DISABLE_STEPPER			0xA8
#define STATUS_STEPPER			0xD0

#define TIMER_FREQUENCY			1000000

typedef struct
{
	float 		rad_pos;
	float 		rad_speed;
	float 		rad_speed_target;
	float 		max_accel;
	float 		max_vel;
	uint16_t 	steps_per_rev;
	uint16_t 	steps_per_rad;
}SpeedPosTypeDef;

typedef struct
{
	SPI_HandleTypeDef* 	hspi_l6474;
	TIM_HandleTypeDef* 	tim_handler;
	uint32_t 			tim_channel;
	GPIO_TypeDef*		gpio_dir_port;
	uint16_t 			gpio_dir_number;
	GPIO_TypeDef*		gpio_rst_port;
	uint16_t			gpio_rst_number;
	GPIO_TypeDef*		gpio_cs_port;
	uint16_t			gpio_cs_number;
	SpeedPosTypeDef		speed_pos;
}l6474TypeDef;

void l6474_enable(l6474TypeDef* stepper_motor);
void l6474_disable(l6474TypeDef* stepper_motor);
void l6474_init(l6474TypeDef* stepper_motor);
void l6474_sel_vel(l6474TypeDef* stepper_motor, float vel);
void l6474_set_steppersec(l6474TypeDef* stepper_motor, uint16_t step);
void l6474_get_speed_pos(l6474TypeDef* stepper_motor);
void l6474_read_status(l6474TypeDef* stepper_motor, uint16_t* status);



#endif /* INC_L6474_H_ */
