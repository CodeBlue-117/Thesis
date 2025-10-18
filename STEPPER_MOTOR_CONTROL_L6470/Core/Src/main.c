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

// NOTE: 10 PI is the highest achievable speed with one motor (5rps)

// MASTER TODO:

// TODO: Begin developing control system
// TODO: Configure software to read IMU data
// TODO: Reduce delay to minimum in l6470_transmit_spi
// TODO: Test F/B/L/R with different accelerations
// TODO: Combine Acceleration and omni direction functions
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

#define DEBOUNCE_DELAY 200  // 50ms debounce time

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define X_MIN_V	1.36
#define X_MAX_V 1.83
#define Y_MIN_V 1.39
#define Y_MAX_V 1.87

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

static uint32_t currentTime;  // Get current system time

float wheel_radius 		= 29.69; // each wheel has a radius of 44.25mm
float omniBody_radius 	= 88.9; // The omni body has a radius of 88.1mm

static uint32_t lastPressTime = 0; // Debounce for GPIO pushbutton
//static uint8_t pushButtonCallCount = 0;
static bool buttonFlag = false;

float vel_temp_1[2];		// motor 2, 3
float vel_temp_2[2];			// motor 1 (second element, had to troubleshoot)

const float J[3][3] = {{-1, 0.5, 0.5}, {0, 0.866, -0.866}, {-0.333, -0.333, -0.333}};
const float J_Inv[3][3] = {{-0.667, 0, -1}, {0.333, 0.577, -1}, {0.333, -0.577, -1}};

float pot_Y_voltage = 0.0f;
float pot_X_voltage = 0.0f;

static bool stopNow = false;
static bool prepareStop = false;

int8_t angleY = 0;
int8_t angleX = 0;

uint16_t adc_buffer[2];  // adc_buffer[0] = Z-X pot, adc_buffer[1] = Z-Y pot

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

// TODO: Verify these delays are required
void omni_drive(float Vx, float Vy, float omega, float r)
{

	float w[3] = {0}; // wheel velocities [w1, w2, w3]
	float V[3] = {Vx, Vy, omega};

	// Matrix multiplication
	for(int i = 0; i < 3; i++)
	{
		w[i] = 0.0f;
		for(int j = 0; j < 3; j++)
		{
			w[i] += J_Inv[i][j] * V[j];
		}
	}

	float motor_set_1_speed[2] = {w[1], w[2]}; // Motor 3 and motor 1 on motor_set_1
	float motor_set_2_speed[2] = {0, w[0]};    // motor 2 on motor_set_2

	// Transmit velocities to motor driver
	HAL_Delay(1); // Was 10
	l6470_set_vel(&motor_set_1, motor_set_1_speed);
	HAL_Delay(1); // Was 10
	l6470_set_vel(&motor_set_2, motor_set_2_speed);
	HAL_Delay(1); // Was 10

	// reset speeds to zero
	motor_set_1_speed[0] = 0;
	motor_set_1_speed[1] = 0;
	motor_set_2_speed[0] = 0;
	motor_set_2_speed[1] = 0;

}

void forward_motion(void)
{
	omni_drive(0.0f, 6.0f, 0.0f, wheel_radius); //12.0f is 2 rps // 24.0 works!!!!
}

void backward_motion(void)
{
	omni_drive(0.0f, -6.0f, 0.0f, wheel_radius);
}

void left_motion(void)
{
	omni_drive(-6.0f, 0.0f, 0.0f, wheel_radius);
}

void right_motion(void)
{
	omni_drive(6.0f, 0.0f, 0.0f, wheel_radius);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Review, test and fix this
void accel(uint8_t start_time, uint8_t end_time, uint8_t start_vel, uint8_t end_vel) // time is in milliseconds, vel is in rad/s
{
	uint8_t delta_t = end_time - start_time;
	uint8_t delta_v = end_vel - start_vel;

 	vel_temp_1[0] = start_vel;
 	vel_temp_1[1] = 0;

	for(uint8_t i = start_time; i < end_time; i += delta_t)
	{
		 vel_temp_1[0] += delta_v;

	  	 l6470_set_vel(&motor_set_1, vel_temp_1);
	  	 HAL_Delay(delta_t);
	}

 	 l6470_soft_stop(&motor_set_1);
 	 l6470_soft_stop(&motor_set_2);

 	 l6470_disable(&motor_set_1);
 	 l6470_disable(&motor_set_2);

}

///////////////////////////////////////////////////////////////////
// TODO: Review, test and fix this
void accel_from_a(float acceleration_rad_s2, float initial_vel_rad_s, uint16_t duration_ms)
{
    const float steps_per_rad = 200.0f / (2.0f * M_PI);
    const uint16_t step_interval_ms = 10;  // Chosen smallest time slice
    uint16_t num_steps = duration_ms / step_interval_ms;

    float current_vel = initial_vel_rad_s;

    for (uint16_t i = 0; i < num_steps; i++)
    {
        current_vel += acceleration_rad_s2 * (step_interval_ms / 1000.0f);  // Convert ms to s
        float vel_steps_per_sec = current_vel * steps_per_rad;
        vel_temp_1[0] = (int32_t)vel_steps_per_sec;
        vel_temp_1[1] = 0;

        l6470_set_vel(&motor_set_1, vel_temp_1);
        HAL_Delay(step_interval_ms);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Map voltages to degrees using linearization
static inline int8_t mapVoltageToAngle(float v, float vMin, float vMax)
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
	scale = (int8_t)((scale * 42.0f) - 21.0f); // [0,1] * 42 = [0, 42] - 21 = [-21,21] --> [-21 ... +21]
	return scale;

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

  	 // uint16_t m1_stat, m2_stat;

	 // Reset L6470s (shared line)
	 HAL_GPIO_WritePin(STEPPER_RST_GPIO_Port, STEPPER_RST_Pin, GPIO_PIN_RESET);
	 HAL_Delay(100);
	 HAL_GPIO_WritePin(STEPPER_RST_GPIO_Port, STEPPER_RST_Pin, GPIO_PIN_SET);
	 HAL_Delay(100);

  	 l6470_disable(&motor_set_1);
  	 l6470_disable(&motor_set_2);

  	 // ***NOW*** do sync + init
  	 l6470_sync_daisy_chain(&motor_set_1);
  	 l6470_sync_daisy_chain(&motor_set_2);

  	 l6470_init_chip_1(&motor_set_1);
  	 l6470_init_chip_2(&motor_set_2);

 	 // --- Enable motors in safe state (e.g. holding position, no motion) ---
 	 // l6470_enable(&motor_set_1);
 	 // l6470_enable(&motor_set_2);

  	 // l6470_get_status(&motor_set_1, &m1_stat, &m2_stat);
  	 // l6470_get_status(&motor_set_2, &m1_stat, &m2_stat);

  	 //	l6470_dump_params_chip1(&motor_set_1);
  	 //	l6470_dump_params_chip2(&motor_set_2);

  	 l6470_disable(&motor_set_1); // TODO: Always disable motors
  	 l6470_disable(&motor_set_2); // TODO: Always disable motors

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(stopNow)
	  {

		  stopNow = false;

			l6470_soft_stop(&motor_set_1);
			l6470_soft_stop(&motor_set_2);

			l6470_disable(&motor_set_1);
			l6470_disable(&motor_set_2);

	  }

	    ///////////////

	  if(buttonFlag == true)
	  {


		  // TODO: THE ORIENTATION IS SCREWED UP! IT RUNS AWAY FROM THE PENDULUM INSTEAD OF TRYING TO CATCH IT!!!!!

		  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		  	// HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // OFF (How to use the LED)
		  	//	  HAL_Delay(100); // was 100

		    pot_Y_voltage = (3.3f * adc_buffer[0]) / 4095.0f; // Y - axis (forward/backward) angle  //TODO: These need to be normalized with the Min and MAX speed input values for set_vel()
		    pot_X_voltage = (3.3f * adc_buffer[1]) / 4095.0f; // X -Axis (Left/Right) angle
		    //	    printf("Z-Y: %.2f V\n\r", pot1_voltage);
		    //	    printf("Z-X: %.2f V\n\r", pot2_voltage);

		    ///////////////
		    // Parse X and Y voltages and convert them to angles asymmetrically, then to x,y values, then to Vx, Vy valuse

		    angleY = mapVoltageToAngle(pot_Y_voltage, Y_MIN_V, Y_MAX_V);
		    angleX = mapVoltageToAngle(pot_X_voltage, X_MIN_V, X_MAX_V);

		    omni_drive(angleX, angleY, 0.0f, 0.0f);
		    HAL_Delay(50);


		  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//		  	buttonFlag = false;
//
//  			l6470_enable(&motor_set_1);
//  			l6470_enable(&motor_set_2);
//
//		  	switch(pushButtonCallCount)
//			{
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
//			}
//
//  			HAL_Delay(2000); // Duration of omni movement
//
//  			l6470_soft_stop(&motor_set_1);
//  			l6470_soft_stop(&motor_set_2);
//
//  			l6470_disable(&motor_set_1);
//  			l6470_disable(&motor_set_2);
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
