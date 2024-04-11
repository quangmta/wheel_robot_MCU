/*
 * motor_driver.h
 *
 *  Created on: Feb 13, 2024
 *      Author: Quang
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif /* INC_MOTOR_DRIVER_H_ */
