#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

#define SERVO_PWM_PIN_NAME       "PB12"
#define SERVO_MIN_PULSE_US       500U
#define SERVO_CENTER_PULSE_US    1500U
#define SERVO_MAX_PULSE_US       2500U
#define SERVO_SLOW_FORWARD_PULSE_US 1400U
#define SERVO_SLOW_REVERSE_PULSE_US 1600U
#define SERVO_STOP_PULSE_US      SERVO_CENTER_PULSE_US

void Servo_Init(void);
void Servo_SetPulseUs(uint32_t pulse_us);
uint32_t Servo_GetPulseUs(void);
void Servo_SetAngle(uint8_t angle);
uint8_t Servo_GetAngle(void);

#endif
