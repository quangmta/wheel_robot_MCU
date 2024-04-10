/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "commands.h"
#include "diff_controller.h"
#include "motor_driver.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SetPointInfo leftPID, rightPID;
//uint32_t encoderLeft = 0;
//uint32_t encoderRight = 0;
int speedLeft, speedRight;
uint8_t moving = 0;
uint8_t rx_buffer[30];
uint8_t tx_buffer[30];
uint8_t flag_value = 0;
uint8_t flag_moving = 0;

// A pair of varibles to help parse serial commands (thanks Fergs)
uint8_t rx_arg = 0;
uint8_t rx_index = 0;

// Variable to hold an input character
uint8_t rx_data;

// Variable to hold the current single-character command
uint8_t rx_cmd;

// Character arrays to hold the first and second arguments
uint8_t rx_argv1[20];
uint8_t rx_argv2[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		leftPID.Encoder = __HAL_TIM_GET_COUNTER(htim);
	} else if (htim == &htim3) {
		rightPID.Encoder = __HAL_TIM_GET_COUNTER(htim);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart1.Instance) {
//		if (!rx_index) {
//			for (int i = 0; i < 30; i++) {
//				rx_buffer[i] = '\0';
//			}
//		}
		if (rx_data == '\n' || rx_data == '\r' || rx_data == '/') {
			if (rx_arg == 1)
				rx_argv1[rx_index] = '\0';
			else if (rx_arg == 2)
				rx_argv2[rx_index] = '\0';
			flag_value = 1;
			rx_index = 0;
		} else if (rx_data == ' ') {
			if (rx_arg == 0)
				rx_arg = 1;
			else if (rx_arg == 1) {
				rx_argv1[rx_index] = '\0';
				rx_arg = 2;
				rx_index = 0;
			}
		} else {
			if (rx_arg == 0) {
				// The first arg is the single-letter command
				rx_cmd = rx_data;
			} else if (rx_arg == 1) {
				// Subsequent arguments can be more than one character
				rx_argv1[rx_index] = rx_data;
				rx_index++;
			} else if (rx_arg == 2) {
				rx_argv2[rx_index] = rx_data;
				rx_index++;
			}
		}

		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}
uint8_t calculate_crc8(uint8_t *pcBlock, uint8_t len) {
	uint8_t crc = 0xFF;
	uint8_t i;

	while (len--) {
		crc ^= *pcBlock++;

		for (i = 0; i < 8; i++)
			crc = crc & 0x80 ? (crc << 1) ^ CRC8_POLYNOMIAL : crc << 1;
	}

	return crc;
}
void resetCommand() {
	rx_cmd = '\0';
	memset(rx_argv1, 0, sizeof(rx_argv1));
	memset(rx_argv2, 0, sizeof(rx_argv2));
	rx_arg = 0;
	rx_index = 0;
}
void runCommand() {
//	char num[30];
//	uint8_t num_index = 0;
//	while (rxBuffer[num_index] != '\0') {
//		num[num_index] = rxBuffer[num_index + 1];
//		num_index++;
//	}
	int i = 0;
	char *p = (char*) rx_argv1;
	char *str;
	int pid_args[4];
	int rx_arg1 = atoi((char*) rx_argv1);
	int rx_arg2 = atoi((char*) rx_argv2);
	//sprintf((char*)tx_buffer,"%c %d %d\n",rx_cmd,rx_arg1,rx_arg2);
//	HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*)tx_buffer), HAL_MAX_DELAY);
	switch (rx_cmd) {
	case READ_ENCODERS:
//		sprintf((char*) tx_buffer, "%ld %ld\n", leftPID.Encoder,
//				rightPID.Encoder);
		tx_buffer[0] = (leftPID.Encoder >> 8) & 0xFF;
		tx_buffer[1] = leftPID.Encoder & 0xFF;

		tx_buffer[2] = (rightPID.Encoder >> 8) && 0xFF;
		tx_buffer[3] = rightPID.Encoder & 0xFF;

		tx_buffer[4] = calculate_crc8(tx_buffer, 4);

		HAL_UART_Transmit(&huart1, tx_buffer, 5, HAL_MAX_DELAY);
		break;
	case READ_SPEED:
//		sprintf((char*) tx_buffer, "%ld %ld\n", leftPID.speed, rightPID.speed);
		tx_buffer[0] = (leftPID.speed >> 8) && 0xFF;
		tx_buffer[1] = leftPID.speed && 0xFF;

		tx_buffer[2] = (rightPID.speed >> 8) && 0xFF;
		tx_buffer[3] = rightPID.speed && 0xFF;

		tx_buffer[4] = calculate_crc8(tx_buffer, 4);

		HAL_UART_Transmit(&huart1, tx_buffer, 5, HAL_MAX_DELAY);
		break;
	case RESET_ENCODERS:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_ALL, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_ALL, 0);
		break;
	case MOTOR_SPEEDS:
		if (rx_arg1 == 0 && rx_arg2 == 0) {
			flag_moving = 1;
//			setMotorSpeeds(0, 0);
//			resetPID(&leftPID);
//			resetPID(&rightPID);
//			moving = 0;
		} else
			moving = 1;
		leftPID.TargetTicksPerFrame = rx_arg1 * PID_TIME / 1000;
		rightPID.TargetTicksPerFrame = rx_arg2 * PID_TIME / 1000;
		break;
	case MOTOR_RAW_PWM:
		resetPID(&leftPID);
		resetPID(&rightPID);
		moving = 0;
		setMotorSpeeds(rx_arg1, rx_arg2);
		break;
	case UPDATE_PID:
		while (*(str = strtok_r(p, ":", &p)) != '\0') {
			pid_args[i] = atoi(str);
			i++;
		}
		if (i == 4) {
			leftPID.Kp = pid_args[0];
			leftPID.Ki = pid_args[1] * PID_TIME / 1000;
			leftPID.Kd = pid_args[2] * 1000 / PID_TIME;
			leftPID.Ko = pid_args[3];

			rightPID.Kp = pid_args[0];
			rightPID.Ki = pid_args[1] * PID_TIME / 1000;
			rightPID.Kd = pid_args[2] * 1000 / PID_TIME;
			rightPID.Ko = pid_args[3];

			HAL_UART_Transmit(&huart1, (uint8_t*) "PID'S SET!\n", 11,
			HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t*) "ERROR!\n", 7, HAL_MAX_DELAY);
		}
		break;
	case READ_PID:
		sprintf((char*) tx_buffer, "Left PID: %d %d %d %d\n", leftPID.Kp,
				leftPID.Ki, leftPID.Kd, leftPID.Ko);
		HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*) tx_buffer),
		HAL_MAX_DELAY);

		sprintf((char*) tx_buffer, "Right PID: %d %d %d %d\n", leftPID.Kp,
				leftPID.Ki, leftPID.Kd, leftPID.Ko);
		HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*) tx_buffer),
		HAL_MAX_DELAY);
		break;
	default:
		break;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
//	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
//	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

	initPID(&leftPID);
	initPID(&rightPID);
	HAL_Delay(500);
	resetPID(&leftPID);
	resetPID(&rightPID);

//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 500);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (flag_value) {
			runCommand();
			resetCommand();
			flag_value = 0;
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
	GPIO_InitStruct.Pin = IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
