/*
 * diff_controller.h
 *
 *  Created on: Feb 13, 2024
 *      Author: Quang
 */

#ifndef INC_DIFF_CONTROLLER_H_
#define INC_DIFF_CONTROLLER_H_

#define MAX_PWM       1000
/* Run the PID loop at 30 times per second */
#define PID_TIME      30 // ms
#define SPEED_MAX 6300 // ticks/s
#define KP 26
#define KI 120
#define KD 1
#define KO 2
#include "stdint.h"
/* PID setpoint info For a Motor */
typedef struct {
	float TargetTicksPerFrame;    // target speed in ticks per frame
	uint32_t Encoder;                  // encoder count
	uint32_t PrevEnc;                  // last encoder count
	uint32_t speed;

	/*
	 * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
	 * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
	 */
	int Input;                // last input
	int PrevInput;
	//int PrevErr;                   // last error

	/*
	 * Using integrated term (ITerm) instead of integrated error (Ierror),
	 * to allow tuning changes,
	 * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
	 */
	//int Ierror;
	int ITerm;                    //integrated term

	int output;                    // last motor setting

	uint16_t Kp;
	uint16_t Ki;
	uint16_t Kd;
	uint16_t Ko;

} SetPointInfo;
void updateSpeed(SetPointInfo *pointPID);
void initPID(SetPointInfo *pointPID);
void resetPID(SetPointInfo * pointPID);
void doPID(SetPointInfo *pointPID);
void updatePID(SetPointInfo * leftPID, SetPointInfo * rightPID);

#endif /* INC_DIFF_CONTROLLER_H_ */
