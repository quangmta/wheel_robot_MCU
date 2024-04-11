/*
 * diff_controller.c
 *
 *  Created on: Feb 13, 2024
 *      Author: Quang
 */
#include "diff_controller.h"
#include "motor_driver.h"

extern uint8_t moving;
extern uint8_t flag_moving;
uint8_t count_flag_moving = 0;
void initPID(SetPointInfo *pointPID){
	pointPID->TargetTicksPerFrame = 0.0;
	pointPID->Encoder = 0.0;
	pointPID->PrevEnc = 0.0;
	pointPID->output = 0;
	pointPID->Input = 0;
	pointPID->PrevInput = 0;
	pointPID->ITerm = 0;
	pointPID->speed = 0;
	pointPID->Kp = KP;
	pointPID->Ki = KI*PID_TIME/1000;
	pointPID->Kd = KD*1000/PID_TIME;
	pointPID->Ko = KO;
}
void resetPID(SetPointInfo *pointPID) {
	pointPID->TargetTicksPerFrame = 0.0;
	pointPID->PrevEnc = pointPID->Encoder;
	pointPID->output = 0;
	pointPID->Input = 0;
	pointPID->PrevInput = 0;
	pointPID->ITerm = 0;
	pointPID->speed = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo *pointPID) {
	int Perror = pointPID->TargetTicksPerFrame - pointPID->PrevInput;

	/*
	 * Avoid derivative kick and allow tuning changes,
	 * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
	 * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
	 */
	//output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
	// p->PrevErr = Perror;
	pointPID->output = (pointPID->Kp * Perror + pointPID->ITerm
			- pointPID->Kd * (pointPID->Input - pointPID->PrevInput))
			/ pointPID->Ko;

	//output += pointPID->output;
	// Accumulate Integral error *or* Limit output.
	// Stop accumulating when output saturates
	if (pointPID->output >= MAX_PWM)
		pointPID->output = MAX_PWM;
	else if (pointPID->output <= -MAX_PWM)
		pointPID->output = -MAX_PWM;
	else
		/*
		 * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
		 */
		pointPID->ITerm += pointPID->Ki * Perror;
}
void updateSpeed(SetPointInfo *pointPID) {
	int input;

	//Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
	input = pointPID->Encoder - pointPID->PrevEnc;
	if ((pointPID->PrevEnc > 49152) && (pointPID->Encoder < 16384))
		input += 65536;
	else if ((pointPID->PrevEnc < 16384) && (pointPID->Encoder > 49152))
		input -= 65536;
	//Low-Pass filter
	input = 0.854 * input + 0.0728 * pointPID->Input
			+ 0.0728 * pointPID->PrevInput;

	pointPID->PrevEnc = pointPID->Encoder;
	pointPID->PrevInput = pointPID->Input;
	pointPID->Input = input;
	pointPID->speed = input * 1000 / PID_TIME;

}
/* Read the encoder values and call the PID routine */
void updatePID(SetPointInfo *leftPID, SetPointInfo *rightPID) {
	/* Read the encoders */
	updateSpeed(leftPID);
	updateSpeed(rightPID);

	if (flag_moving){
		count_flag_moving ++;
		if (count_flag_moving == 15){
			setMotorSpeeds(0, 0);
			resetPID(leftPID);
			resetPID(rightPID);
			moving = 0;
			count_flag_moving = 0;
			flag_moving = 0;
		}
	}

	/* If we're not moving there is nothing more to do */
	if (!moving) {
		/*
		 * Reset PIDs once, to prevent startup spikes,
		 * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
		 * PrevInput is considered a good proxy to detect
		 * whether reset has already happened
		 */
		if (leftPID->PrevInput != 0 || rightPID->PrevInput != 0) {
			resetPID(leftPID);
			resetPID(rightPID);
		}
		return;
	}
	/* Compute PID update for each motor */
	doPID(leftPID);
	doPID(rightPID);

	/* Set the motor speeds accordingly */
	setMotorSpeeds(leftPID->output, rightPID->output);
}

