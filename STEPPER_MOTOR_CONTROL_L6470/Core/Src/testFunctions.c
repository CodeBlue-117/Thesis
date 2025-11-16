/*
 * testFunctions.c
 *
 *  Created on: Nov 15, 2025
 *      Author: jake-
 */

#include "main.h"

		  // Motor Speed test
		  // omni_drive(-angleX, -angleY, 0.0f, 0.0f);
		  // float speed = 10*M_PI;
		  // l6470_set_vel(&motor_set_1, &speed);



		  // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // OFF (How to use the LED)
		  // HAL_Delay(100); // was 100

/////////////////////////////////////////////////////////////////////

//		  static uint8_t pushButtonCallCount = 0;

//		  buttonFlag = false;
//
//  	  l6470_enable(&motor_set_1);
//  	  l6470_enable(&motor_set_2);
//
//		  switch(pushButtonCallCount)
//		  {
//		  		case 0:
//		  			pushButtonCallCount = 1;
//		  			forward_motion();
//		  			break;
//
//		  		case 1:
//		  			pushButtonCallCount = 2;
//		  			backward_motion();
//		  			break;
//
//		  		case 2:
//		  			pushButtonCallCount = 3;
//		  			left_motion();
//		  			break;
//
//		  		case 3:
//		  			pushButtonCallCount = 0;
//		  			right_motion();
//		  			break;
//		  }
//
//  			HAL_Delay(2000); // Duration of omni movement
//
//  			l6470_soft_stop(&motor_set_1);
//  			l6470_soft_stop(&motor_set_2);
//
//  			l6470_disable(&motor_set_1);
//  			l6470_disable(&motor_set_2);

//void forward_motion(void)
//{
//	omni_drive(0.0f, 6.0f, 0.0f, WHEEL_RADIUS); //12.0f is 2 rps // 24.0 works!!!!
//}
//
//void backward_motion(void)
//{
//	omni_drive(0.0f, -6.0f, 0.0f, WHEEL_RADIUS);
//}
//
//void left_motion(void)
//{
//	omni_drive(-6.0f, 0.0f, 0.0f, WHEEL_RADIUS);
//}
//
//void right_motion(void)
//{
//	omni_drive(6.0f, 0.0f, 0.0f, WHEEL_RADIUS);
//}
//
//
//// TODO: Review, test and fix this
//void accel(uint8_t start_time, uint8_t end_time, uint8_t start_vel, uint8_t end_vel) // time is in milliseconds, vel is in rad/s
//{
//	uint8_t delta_t = end_time - start_time;
//	uint8_t delta_v = end_vel - start_vel;
//
// 	vel_temp_1[0] = start_vel;
// 	vel_temp_1[1] = 0;
//
//	for(uint8_t i = start_time; i < end_time; i += delta_t)
//	{
//		 vel_temp_1[0] += delta_v;
//
//	  	 l6470_set_vel(&motor_set_1, vel_temp_1);
//	  	 HAL_Delay(delta_t);
//	}
//
// 	 l6470_soft_stop(&motor_set_1);
// 	 l6470_soft_stop(&motor_set_2);
//
// 	 l6470_disable(&motor_set_1);
// 	 l6470_disable(&motor_set_2);
//
//}
//
//// TODO: Review, test and fix this
//void accel_from_a(float acceleration_rad_s2, float initial_vel_rad_s, uint16_t duration_ms)
//{
//    const float steps_per_rad = 200.0f / (2.0f * M_PI);
//    const uint16_t step_interval_ms = 10;  // Chosen smallest time slice
//    uint16_t num_steps = duration_ms / step_interval_ms;
//
//    float current_vel = initial_vel_rad_s;
//
//    for (uint16_t i = 0; i < num_steps; i++)
//    {
//        current_vel += acceleration_rad_s2 * (step_interval_ms / 1000.0f);  // Convert ms to s
//        float vel_steps_per_sec = current_vel * steps_per_rad;
//        vel_temp_1[0] = (int32_t)vel_steps_per_sec;
//        vel_temp_1[1] = 0;
//
//        l6470_set_vel(&motor_set_1, vel_temp_1);
//        HAL_Delay(step_interval_ms);
//    }
//}
