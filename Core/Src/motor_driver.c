/*
 * motor_driver.c
 *
 *  Created on: Feb 13, 2024
 *      Author: Quang
 */
#include "main.h"
#include "motor_driver.h"
#include "diff_controller.h"
#include "commands.h"
#include "string.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

void initMotorController() {
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
}
void setMotorSpeed(int i, int spd) {
	uint8_t reverse = 0;
	if (spd < 0) {
		spd = -spd;
		reverse = 1;
	}
	if (spd > MAX_PWM)
		spd = MAX_PWM;

	if (i == LEFT) {
		if (reverse) {
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		} else {
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, spd);
	} else {
		if (reverse) {
			HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
		} else {
			HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, spd);
	}

}
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
	setMotorSpeed(LEFT, leftSpeed);
	setMotorSpeed(RIGHT, rightSpeed);
}
