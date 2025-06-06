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

// ToDo: Check the Status register when the battery is plugged in and program is running to determine cause of red light


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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t status_step;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

bool buttonFlag = false;
uint8_t pushButtonCallCount = 0;

float wheel_radius 		= 44.25; // each wheel has a radius of 44.25mm ToDo: Update this
float omniBody_radius 	= 88.1; // The omni body has a radius of 88.1mm ToDo: Update this

float vel_temp_1[2];		// motor 2, 3
float vel_temp_2;			// motor 1

const float J[3][3] = {{-1, 0.5, 0.5}, {0, 0.866, -0.866}, {-0.333, -0.333, -0.333}};
const float J_Inv[3][3] = {{-0.667, 0, -1}, {0.333, 0.577, -1}, {0.333, -0.577, -1}};

static MotorSetTypedef motor_set_1 = {
		.gpio_rst_port = STEPPER_RST_GPIO_Port,
		.gpio_cs_number = STEPPER_SPI1_CS_Pin,
		.gpio_cs_port = STEPPER_SPI1_CS_GPIO_Port,
		.gpio_rst_number = STEPPER_RST_Pin,
		.num_motors = 2,
		.hspi_l6470 = &hspi1,
};

static MotorSetTypedef motor_set_2 = {
		.gpio_rst_port = STEPPER_RST_GPIO_Port,
		.gpio_cs_number = STEPPER_SPI2_CS_Pin,
		.gpio_cs_port = STEPPER_SPI2_CS_GPIO_Port,
		.gpio_rst_number = STEPPER_RST_Pin,

		.num_motors = 1,
		.hspi_l6470 = &hspi2,
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void forward_motion(void);
void backward_motion(void);
void left_motion(void);
void right_motion(void);

// Redirect printf() to USART2
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

#define DEBOUNCE_DELAY 50  // 50ms debounce time

uint32_t lastPressTime = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == B1_Pin)
    {
        uint32_t currentTime = HAL_GetTick();  // Get current system time

        // Check if the debounce period has passed
        if((currentTime - lastPressTime) >= DEBOUNCE_DELAY)
        {
            lastPressTime = currentTime; // Update last press time
            printf("USER PUSH BUTTON SELECTED!!!\n\r");

            buttonFlag = true;

        }
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi == motor_set_1.hspi_l6470)
	{
		printf("MOTOR SET 1 SPI COMPLETE\n\r");
	}

	else if(hspi == motor_set_2.hspi_l6470)
	{
		printf("MOTOR SET 2 SPI COMPLETE\n\r");
	}
}

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

	float vel_temp_1[2] = {w[2], w[0]}; // Motor 3 and motor 1 on motor_set_1
	float vel_temp_2 = w[1];				// motor 2 on motor_set_2

	printf("vel_temp_1[0]: %f\n\r", vel_temp_1[0]);
	printf("vel_temp_1[1]: %f\n\r", vel_temp_1[1]);
	printf("vel_temp_2: %f\n\r", vel_temp_2);

	// Transmit velocities to motor driver
	l6470_set_vel(&motor_set_1, vel_temp_1);
	HAL_Delay(5);
	l6470_set_vel(&motor_set_2, &vel_temp_2);
	HAL_Delay(5000);

}

void forward_motion(void)
{
	printf("Forward\n\r");
	HAL_Delay(10);
	omni_drive(0.0f, 6.0f, 0.0f, wheel_radius);
}

void backward_motion(void)
{
	printf("Backward\n\r");
	HAL_Delay(10);
	omni_drive(0.0f, -6.0f, 0.0f, wheel_radius);
}

void left_motion(void)
{
	printf("Left\n\r");
	HAL_Delay(10);
	omni_drive(-6.0f, 0.0f, 0.0f, wheel_radius);
}

void right_motion(void)
{
	printf("Right\n\r");
	HAL_Delay(10);
	omni_drive(6.0f, 0.0f, 0.0f, wheel_radius);
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
  /* USER CODE BEGIN 2 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  l6470_init(&motor_set_1);
  l6470_init(&motor_set_2);

  HAL_Delay(100);

  l6470_enable(&motor_set_1);
  l6470_enable(&motor_set_2);

  HAL_Delay(100);

///////////////////////////////////////////////////////////

  	  // testing Code (KEEP)
  	  // 6 = 1rps
    vel_temp_1[0] = 6; // motor 2
    vel_temp_1[1] = 6; // motor 3

    vel_temp_2 = 6; 	// motor 1 ----

    uint16_t stat = l6470_get_status(&motor_set_1);
    printf("\n\rM1 Status: 0x%04X\n\r", stat);

    l6470_set_vel(&motor_set_1, vel_temp_1);
    HAL_Delay(1000);  // ToDo: Do we need this delay?

//    l6470_set_vel(&motor_set_2, &vel_temp_2);
//    HAL_Delay(5000);

//    stat = l6470_get_status(&motor_set_2);
//    printf("M2 Status: 0x%04X\n\r", stat);

///////////////////////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(100);

	  if(buttonFlag == true)
	  {
		  	buttonFlag = false;

		  	if(pushButtonCallCount == 0)
			{
		  		  l6470_enable(&motor_set_1);
		  		  l6470_enable(&motor_set_2);

		  		  HAL_Delay(500);
		  		  pushButtonCallCount++;
		  		  forward_motion();

		  		  l6470_disable(&motor_set_1);
		  		  l6470_disable(&motor_set_2);
			}
			else if(pushButtonCallCount == 1)
			{
				  l6470_enable(&motor_set_1);
				  l6470_enable(&motor_set_2);

				  HAL_Delay(500);
				  pushButtonCallCount++;
				  backward_motion();

				  l6470_disable(&motor_set_1);
				  l6470_disable(&motor_set_2);

			}
			else if(pushButtonCallCount == 2)
			{
				  l6470_enable(&motor_set_1);
				  l6470_enable(&motor_set_2);

				  HAL_Delay(500);
				  pushButtonCallCount++;
				  left_motion();

				  l6470_disable(&motor_set_1);
				  l6470_disable(&motor_set_2);
			}
			else if(pushButtonCallCount == 3)
			{
				  l6470_enable(&motor_set_1);
				  l6470_enable(&motor_set_2);

				  HAL_Delay(500);
				  pushButtonCallCount++;
				  right_motion();

				  l6470_disable(&motor_set_1);
				  l6470_disable(&motor_set_2);
			}
			else
			{
				pushButtonCallCount = 0;
			}

	  }

	////////////////////////////////////////////////////////////////////////////////// OLD CODE FOR POSITION
	//	  l6470_get_speed_pos(&motor_set_1);
	//	  angular_position1 = motor_set_1.motors[0].speed_pos.rad_pos;
	//	  angular_position2 = motor_set_1.motors[1].speed_pos.rad_pos - motor_set_1.motors[0].speed_pos.rad_pos;
	//////////////////////////////////////////////////////////////////////////////////

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
  HAL_GPIO_WritePin(GPIOA, STEPPER_SPI1_CS_Pin|STEPPER_SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STEPPER_RST_GPIO_Port, STEPPER_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : STEPPER_RST_Pin */
  GPIO_InitStruct.Pin = STEPPER_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEPPER_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
