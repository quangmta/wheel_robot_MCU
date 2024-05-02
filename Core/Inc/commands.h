/*
 * commands.h
 *
 *  Created on: Feb 13, 2024
 *      Author: Quang
 */

#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_

#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define RESET_ENCODERS 'r'
#define UPDATE_PID     'u'
#define READ_PID	   'p'
#define READ_SPEED	   's'
#define SEND_SPEED	   'v'
#define SEND_ENCODERS  'c'
#define LEFT            0
#define RIGHT           1
#define CRC8_POLYNOMIAL 0x31

#endif /* INC_COMMANDS_H_ */
