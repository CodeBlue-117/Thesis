/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Jake Price.
  * All rights reserved.

  *
  ******************************************************************************
  */

// Use speeds 0 - 10PI
// NOTE: 10 PI is the highest achievable speed with one motor (5rps)

// MASTER TODO:

// TODO: Reduce delay to minimum in l6470_transmit_spi
// TODO: Test F/B/L/R with different accelerations
// TODO: Implement DMA


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "l6470.h"
#include <stdio.h>
#include "stm32f4xx_it.h"
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void forward_motion(void);
void backward_motion(void);
void left_motion(void);
void right_motion(void);
void l6470_get_param_chip_1(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t length);
void l6470_get_param_chip_2(MotorSetTypedef* stepper_motor, uint8_t param, uint8_t length);
void l6470_sync_daisy_chain(MotorSetTypedef *stepper_motor);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6000_ADDR 		(0x68 << 1) // 0xD0
#define PWR_MGMT_REG_1		(0x6B)
#define SIGNAL_PATH_REG 	(0x68)
#define CONFIG_REG			(0x1A)
#define ACCEL_CONFIG_REG	(0x1C)
#define DEBOUNCE_DELAY 	200  // 50ms debounce time
#define DEFAULT_DT	  	0.003f

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Tune these PID parameters
#define K_P_X 50.0f // Proportional constant for x-dir
#define K_P_Y 50.0f // proportional constant for y-dir
#define K_I_X 0.05f // 0.01f // Integral constant for x-dir
#define K_I_Y 0.05f //0.01f // Integral constant for y-dir
#define K_D_X 0.5f  // 5.0f
#define K_D_Y 0.5f  // 5.0f

// TODO: Increase the MAX VEL
#define MAX_CART_VEL 	0.5f // 0.9f  // m/s, tune for safety (v = rw => v m/s = (0.03m) * (10)*PI = 0.94 m/s)
#define MIN_CART_VEL 	-0.5f //-0.9f

// Tune the max integral???
#define MAX_INTEGRAL  	5.0f // anti-windup cap on integral, tune
#define MIN_INTEGRAL 	-5.0f

// TODO: Remove input POT filter until tested -> Model filter with random data and see what the output is.
#define POT_FC_HZ	  	15.0f // TODO: Tune this

// TODO: TUNE the DEADBAND
#define DEADBAND 	  	(0.25f * M_PI/180.0f)  // 0.5 degree for the dead band (no integral)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static uint32_t currentTime;  					// Get current system time
static uint32_t lastPressTime 		= 0; 		// Debounce for GPIO pushbutton
static bool buttonFlag 				= false;	// flag to START motors on button press interrupt
static bool stopNow 				= false;	// flag to SHUT OFF motors on button press interrupt
static bool prepareStop 			= false;	// Debounce for shutoff stopNow

float vel_temp_1[2];							// motor 2, 3
float vel_temp_2[2];							// motor 1 (second element, had to troubleshoot)
float pot_Y_voltage 				= 0.0f;     // Y POT VOLTAGE
float pot_X_voltage 				= 0.0f;		// X POT VOLTAGE

// JACOBIAN and INVERSE JACOBIAN for the transformation matrix mapping x and y to the three wheels (pi / 3) [120 deg] offset
const float J[3][3] 	= {{-1, 0.5, 0.5}, {0, 0.866, -0.866}, {-0.333, -0.333, -0.333}};
const float J_Inv[3][3] = {{0.667, 0, 1}, {-0.333, 0.577, 1}, {-0.333, -0.577, 1}};

// FIltered X,Y POT values TODO: Remove these until tested
//static float potX_filt = 0.0f;
//static float potY_filt = 0.0f;

uint16_t adc_buffer[2];  						// NOTE: adc_buffer[0] = Z-X pot, adc_buffer[1] = Z-Y pot
int16_t ax, ay, az;  							// IMU variables

typedef struct controlVariables 				// PID Control System Variables
{
	float prevCommandedCartVelocityX;
	float prevCommandedCartVelocityY;

	float curCommandedCartVelocityX;
	float curCommandedCartVelocityY;

	float prevThetaX;
	float prevThetaY;

	float curThetaX;
	float curThetaY;

	// Can also include Theta_dotX/Y for Derivative control

	float prevInputU_X;
	float prevInputU_Y;

	float curInputU_X;
	float curInputU_Y;

	float integralX;
	float integralY;

} controlVariables;

controlVariables myControlVariables = {0};

MotorSetTypedef motor_set_1 = { // TODO: Finish initializing the structs

		.identifier     = 1,
		.gpio_rst_port  = STEPPER_RST_GPIO_Port,
		.gpio_rst_pin 	= STEPPER_RST_Pin,
		.gpio_cs_port 	= STEPPER_SPI1_CS_GPIO_Port,
		.gpio_cs_pin 	= STEPPER_SPI1_CS_Pin,
		.hspi_l6470 	= &hspi1,
		.num_motors 	= 2,
		.spi_dma_busy 	= 0,

};

MotorSetTypedef motor_set_2 = {

		.identifier     = 2,
		.gpio_rst_port 	= STEPPER_RST_GPIO_Port,
		.gpio_rst_pin 	= STEPPER_RST_Pin,
		.gpio_cs_port 	= STEPPER_SPI2_CS_GPIO_Port,
		.gpio_cs_pin 	= STEPPER_SPI2_CS_Pin,
		.hspi_l6470 	= &hspi2,
		.num_motors		= 2, // This has to be two in order to get the motor 1 to work
		.spi_dma_busy 	= 0,

};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

// Redirect printf() to USART1
int __io_putchar(int ch) // Use UART 1 for FTDI cable
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == USER_BUTTON_Pin)
    {
        currentTime = HAL_GetTick();  // Get current system time

        // Check if the debounce period has passed
        if((currentTime - lastPressTime) >= DEBOUNCE_DELAY)
        {
            lastPressTime = currentTime; // Update last press time

            if(prepareStop)
            {
            	stopNow = true;
            }

            prepareStop = true;
            buttonFlag = buttonFlag ^ 1;
        }
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TODO: Do we use this?
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi == motor_set_1.hspi_l6470)
	{
		// printf("MOTOR SET 1 SPI COMPLETE\n\r");
	}

	else if(hspi == motor_set_2.hspi_l6470)
	{
		//printf("MOTOR SET 2 SPI COMPLETE\n\r");
	}
}

//static inline void update_pot_filter(float rawX, float rawY, float dt)
//{
//
//	// Computer alpha from cutoff
//	float tau = 1.0f / (2.0 * M_PI * POT_FC_HZ);
//	float alpha = dt / (tau + dt);
//
//	// First time filter initialization (no sudden jump)
//	static bool initialized = false;
//	if(!initialized)
//	{
//		potX_filt = rawX;
//		potY_filt = rawY;
//		initialized = true;
//	}
//
//	// exponential smoothing
//	potX_filt += alpha * (rawX - potX_filt);
//	potY_filt += alpha * (rawY - potY_filt);
//}

// TODO: Verify these delays are required
void omni_drive(float Vx, float Vy, float omega)
{

	float V_body[3] 	= {Vx, Vy, omega}; 	// Body velocities
	float v_wheel[3] 	= {0}; 				// Wheel linear velocities
	float w[3] 			= {0}; 				// Wheel angular velocities

	// Matrix multiplication
	for(int i = 0; i < 3; i++)
	{
		v_wheel[i] = 0.0f;
		for(int j = 0; j < 3; j++)
		{
			v_wheel[i] += J_Inv[i][j] * V_body[j];
		}
	}

	// --- Linear -> Angular velocities ---
	for(int j = 0; j < 3; j++)
	{
		w[j] = v_wheel[j] / WHEEL_RADIUS;
	}

	// Wheel mapping to motor sets
	float motor_set_1_speed[2] = {w[1], w[2]}; // Motor 3 and motor 1 on motor_set_1
	float motor_set_2_speed[2] = {0, w[0]};    // motor 2 on motor_set_2

	// Transmit velocities to motor driver
	l6470_set_vel(&motor_set_1, motor_set_1_speed);
	l6470_set_vel(&motor_set_2, motor_set_2_speed);

}

// Map voltages to degrees using linearization
static inline float mapVoltageToAngle(float v, float vMin, float vMax)
{
	if(v < vMin)
	{
		v = vMin;
	}
	if(v > vMax)
	{
		v = vMax;
	}

	float scale = (v - vMin) / (vMax - vMin); // Normalized
	scale = ((scale * 42.0f) - 21.0f); // [0,1] * 42 = [0, 42] - 21 = [-21,21] --> [-21 ... +21]
	scale *= (M_PI / 180.0f); // Convert degrees to radians
	return scale;

}

HAL_StatusTypeDef IMU_Write(uint16_t reg, uint8_t data)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c1, MPU6000_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
	return status;

}


HAL_StatusTypeDef IMU_Read(uint16_t reg, uint8_t *buf, uint8_t len)
{

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1, MPU6000_ADDR, reg, 1, buf, len, HAL_MAX_DELAY);
	return status;
}


uint8_t initializeIMU(void)
{
	  HAL_StatusTypeDef status;

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Read WhoAmI and verify it is 0x68
	  uint8_t reg = 0x75;
	  uint8_t receiveData = 0;

	  status = IMU_Read(reg, &receiveData, 1);
	  if(status != HAL_OK)
	  {
		  printf("Error reading WhoAmI register\n\r");
		  return 1;
	  }

	  HAL_Delay(100);

	  if(receiveData != 0x68)
	  {
		  printf("Error reading WhoAmI register\n\r");
		  return 1;
	  }

	  HAL_Delay(100);

	  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Device Reset
	  status = IMU_Write(PWR_MGMT_REG_1, 0x80); // 1000-0000
	  if(status != HAL_OK)
	  {
		  printf("Error reading WhoAmI register\n\r");
		  return 1;
	  }

	  HAL_Delay(100);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //Signal Path Reset
	  status = IMU_Write(SIGNAL_PATH_REG, 0x07); // Reset GYRO, ACCEL and TEMP 0000-0111 = 0x07
	  if(status != HAL_OK)
	  {
		  printf("Error resetting accel and gyro\n\r");
		  return 1;
	  }

	  HAL_Delay(100);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Wakeup and Clock Source
	  status = IMU_Write(PWR_MGMT_REG_1, 0x01); // Clock Source PLL from x-axis
	  if(status != HAL_OK)
	  {
		  printf("Error setting clock source\n\r");
		  return 1;
	  }

	  HAL_Delay(100);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Configure DLPF
	  status = IMU_Write(CONFIG_REG, 0x02); // DLPF in CONFIG REG set to 94Hz bandwidth and 3ms delay (try 0x01 for 184Hz BW and 2ms delay)
	  if(status != HAL_OK)
	  {
		  printf("Error setting clock source\n\r");
		  return 1;
	  }
	  HAL_Delay(100);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  // Configure Accel Full Scale Range
	  status = IMU_Write(ACCEL_CONFIG_REG, 0x00); // Configure Accel for full scale range +2g
	  if(status != HAL_OK)
	  {
		  printf("Error setting clock source\n\r");
		  return 1;
	  }
	  HAL_Delay(100);

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  return status;
}

uint8_t IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t buf[6];

	if(HAL_I2C_Mem_Read(&hi2c1, MPU6000_ADDR, 0x3B, 1, buf, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		return 1;
	}

	*ax = (int16_t)((buf[0] << 8) | buf[1]);
	*ay = (int16_t)((buf[2] << 8) | buf[3]);
	*az = (int16_t)((buf[4] << 8) | buf[5]);

	return 0;

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  	 HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);

	 // Reset L6470s (shared line)
	 HAL_GPIO_WritePin(STEPPER_RST_GPIO_Port, STEPPER_RST_Pin, GPIO_PIN_RESET);
	 HAL_Delay(100);
	 HAL_GPIO_WritePin(STEPPER_RST_GPIO_Port, STEPPER_RST_Pin, GPIO_PIN_SET);
	 HAL_Delay(100);

  	 // ***NOW*** do sync + init
  	 l6470_sync_daisy_chain(&motor_set_1);
  	 l6470_sync_daisy_chain(&motor_set_2);

  	 l6470_init_chip_1(&motor_set_1);
  	 l6470_init_chip_2(&motor_set_2);

  	 // uint16_t m1_stat, m2_stat;
  	 // l6470_get_status(&motor_set_1, &m1_stat, &m2_stat);
  	 // l6470_get_status(&motor_set_2, &m1_stat, &m2_stat);

  	 //	l6470_dump_params_chip1(&motor_set_1);
  	 //	l6470_dump_params_chip2(&motor_set_2);

  	 l6470_disable(&motor_set_1); // TODO: Always disable motors
  	 l6470_disable(&motor_set_2); // TODO: Always  disable motors

 	 // --- Enable motors in safe state (e.g. holding position, no motion) ---
	 l6470_enable(&motor_set_1);
	 l6470_enable(&motor_set_2);

	 uint8_t retVal = initializeIMU();
	 if(retVal == HAL_OK)
	 {
		 printf("IMU initialized!\n\r");
	 }
	 else
	 {
		 printf("IMU FAILED to initialize, retVal: %d\n\r", retVal);
	 }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //////////////////////////////////////////////////////////////////////

//	  if(IMU_ReadAccel(&ax, &ay, &az) == 0)
//	  {
//		  float xg = ax / 16384.0f;
//		  float yg = ay / 16384.0f;
//		  float zg = az / 16384.0f;
//
//		  printf("AX: %.2f, AY: %.2f, AZ %.2f\n\r", xg, yg, zg);
//
//		  HAL_Delay(50);
//	  }

	  /////////////////////////////////////////////////////////////////////
//
//	  static float lastTick = 0.0f;
//	  uint32_t nowTick = HAL_GetTick(); // TODO: Should we change nowTick to a float?
//	  float dt;
//
//	  if(lastTick == 0)
//	  {
//		  dt = DEFAULT_DT;
//	  }
//	  else
//	  {
//		  dt = (nowTick - lastTick) * 0.001f; // ms -> s
//	  }
//
//	  if(dt <= 0.0f)
//	  {
//		  dt = DEFAULT_DT;
//	  }
//
//	  lastTick = nowTick;

	  if(stopNow)
	  {

		  stopNow = false;

			l6470_soft_stop(&motor_set_1);
			l6470_soft_stop(&motor_set_2);

			l6470_disable(&motor_set_1);
			l6470_disable(&motor_set_2);

	  }

	  if(buttonFlag == true)
	  {

		  pot_Y_voltage = (3.3f * adc_buffer[0]) / 4095.0f; // Y - axis (forward/backward) angle
		  pot_X_voltage = (3.3f * adc_buffer[1]) / 4095.0f; // X -Axis (Left/Right) angle

		   printf("(VALOTAGE): Z-X: %.2f V\n\r", pot_X_voltage);
		   printf("(VOLTAGE): Z-Y: %.2f V\n\r", pot_Y_voltage);

		  // update_pot_filter(pot_X_voltage, pot_Y_voltage, dt); // TODO: REPLACE ALL INSTANCES OF potX_filt and potY_filt with pot_X_voltage, pot_Y_voltage

		  // printf("AFTER: Z-X: %.2f V\n\r", potX_filt);
		  // printf("AFTER: Z-Y: %.2f V\n\r", potY_filt);

		  // Parse X and Y voltages and convert them to angles asymmetrically, then to x,y values, then to Vx, Vy valuse
		  myControlVariables.curThetaX = mapVoltageToAngle(pot_X_voltage, X_MIN_V, X_MAX_V);
		  myControlVariables.curThetaY = mapVoltageToAngle(pot_Y_voltage, Y_MIN_V, Y_MAX_V);

		   printf("(ANGLE): Z-X: %.2f V\n\r", myControlVariables.curThetaX);
		   printf("(ANGLE): Z-Y: %.2f V\n\r", myControlVariables.curThetaY);

//		  // Deadband
//		  if(fabs(myControlVariables.curThetaX) < DEADBAND) // TODO: Tune the deadband
//		  {
//			  myControlVariables.curThetaX = 0.0f;
//			  myControlVariables.integralX = 0.0f;
//		  }
//		  if(fabs(myControlVariables.curThetaY) < DEADBAND) // TODO: Tune the deadband
//		  {
//			  myControlVariables.curThetaY = 0.0f;
//			  myControlVariables.integralY = 0.0f;
//		  }
//
//		  // --- Integral Control ---
//		  // Compute the integral / accumulation of error
//		  myControlVariables.integralX += 0.5 * (myControlVariables.curThetaX + myControlVariables.prevThetaX) * dt;
//		  myControlVariables.integralY += 0.5 * (myControlVariables.curThetaY + myControlVariables.prevThetaY) * dt;
//
//		  // anti-windup: clamp integrals  --> TODO: Later implement: if (velocity saturated) do not integrate
//		  if (myControlVariables.integralX > MAX_INTEGRAL) myControlVariables.integralX = MAX_INTEGRAL;
//		  if (myControlVariables.integralX < MIN_INTEGRAL) myControlVariables.integralX = MIN_INTEGRAL;
//		  if (myControlVariables.integralY > MAX_INTEGRAL) myControlVariables.integralY = MAX_INTEGRAL;
//		  if (myControlVariables.integralY < MIN_INTEGRAL) myControlVariables.integralY = MIN_INTEGRAL;
//
//		  // --- Derivative Control ---
//		  float theta_dotX = (myControlVariables.curThetaX -  myControlVariables.prevThetaX) / dt;
//		  float theta_dotY = (myControlVariables.curThetaY -  myControlVariables.prevThetaY) / dt;
//
//		  // u = Kp * cur_theta + Ki * 0.5 * [cur_theta + prev_theta] * Control_Loop_Time ---> the 0.5 factor in the second term comes from the trapezoid rule
//		  myControlVariables.curInputU_X = (K_P_X * myControlVariables.curThetaX) + (K_I_X * myControlVariables.integralX) + (K_D_X * theta_dotX); // Use negative to oppose the tilt
//		  myControlVariables.curInputU_Y = (K_P_Y * myControlVariables.curThetaY) + (K_I_Y * myControlVariables.integralY) + (K_D_Y * theta_dotY); // Use negative to oppose the tilt
//
//		  // Accel = (V2 - V1) / (CONTROL_LOOP_TIME) ---> V2 = Accel * CONTROL_LOOP_TIME + V1
//		  myControlVariables.curCommandedCartVelocityX = (myControlVariables.curInputU_X * dt) + myControlVariables.prevCommandedCartVelocityX;
//		  myControlVariables.curCommandedCartVelocityY = (myControlVariables.curInputU_Y * dt) + myControlVariables.prevCommandedCartVelocityY;
//
//		  // --- clamp velocities to safe range ---
//		  if (myControlVariables.curCommandedCartVelocityX > MAX_CART_VEL) myControlVariables.curCommandedCartVelocityX = MAX_CART_VEL;
//		  if (myControlVariables.curCommandedCartVelocityX < MIN_CART_VEL) myControlVariables.curCommandedCartVelocityX = MIN_CART_VEL;
//		  if (myControlVariables.curCommandedCartVelocityY > MAX_CART_VEL) myControlVariables.curCommandedCartVelocityY = MAX_CART_VEL;
//		  if (myControlVariables.curCommandedCartVelocityY < MIN_CART_VEL) myControlVariables.curCommandedCartVelocityY = MIN_CART_VEL;
//
//        // Send Commands to Motors
//		  omni_drive(myControlVariables.curCommandedCartVelocityX, myControlVariables.curCommandedCartVelocityY, 0.0f);
//
//		  // Update previous values
//		  myControlVariables.prevThetaX = myControlVariables.curThetaX;
//		  myControlVariables.prevThetaY = myControlVariables.curThetaY;
//
//		  // TODO: Do we need to save the previous inputs U?
//		  myControlVariables.prevInputU_X = myControlVariables.curInputU_X;
//		  myControlVariables.prevInputU_Y = myControlVariables.curInputU_Y;
//
//		  myControlVariables.prevCommandedCartVelocityX = myControlVariables.curCommandedCartVelocityX;
//		  myControlVariables.prevCommandedCartVelocityY = myControlVariables.curCommandedCartVelocityY;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 320;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEPPER_SPI1_CS_Pin|STEPPER_SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STEPPER_RST_GPIO_Port, STEPPER_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STEPPER_FLG_Pin */
  GPIO_InitStruct.Pin = STEPPER_FLG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STEPPER_FLG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEPPER_SPI1_CS_Pin STEPPER_SPI2_CS_Pin */
  GPIO_InitStruct.Pin = STEPPER_SPI1_CS_Pin|STEPPER_SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STEPPER_RST_Pin */
  GPIO_InitStruct.Pin = STEPPER_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEPPER_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  HAL_GPIO_WritePin(STEPPER_SPI2_CS_GPIO_Port, STEPPER_SPI1_CS_Pin, GPIO_PIN_SET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
