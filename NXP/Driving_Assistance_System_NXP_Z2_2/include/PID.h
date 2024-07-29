/*
 * PID.h
 *
 *  Created on: Jul 10, 2024
 *      Author: ahmed
 */

#ifndef PID_H_
#define PID_H_

#include "Motor.h"
#include "EXTI_Encoder.h"
#include "pin_mux.h"
#include "stm1.h"
#include <math.h>

#define PI 3.14159265358979323846
#define WHEEL_DIAMETER 6.5 // 65mm in CM
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define ENCODER_CPR 492
#define PID_INTERVAL 0.001 // 100ms
// Encoder counts for each motor
//FR FL BR BL
volatile int32_t *encoder_counts = (int32_t *) 0x40080000;
//volatile int32_t encoder_counts[4] = { 0 };
volatile int32_t prev_encoder_counts[4] = { 0 };
volatile float motor_velocities[4] = { 0 };
volatile float pid_outputs[4] = { 0 };
volatile int32_t Motor_direction[4] = { 0 };
// PID constants
const float Kp = 5000 / 88;
const float Ki = 0.1;
float integral_errors[4] = { 0 };
uint32_t ticks; /* The number of ticks */
/* Motor speed */
uint32_t prev_speed = 30;


void PID_Init(void);

#endif /* PID_H_ */
