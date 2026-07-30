#ifndef __CONTROL_H
#define __CONTROL_H
#include "board.h"

#define EncoderMultiples  2
#define CONTROL_FREQUENCY 200
#define Black_WheelDiameter   0.065f
#define Perimeter       0.204203519f
#define MOTOR_GEAR_RATIO       28.0f
#define ENCODER_RESOLUTION     13.0f
#define Wheelspacing    0.2100f
#define PI              3.1415926f

#define PWM_MAX         7800

#define CONTROL_MODE_RUN     0
#define CONTROL_MODE_SEARCH  1

#define LEFT_ENCODER_SIGN    1.0f
#define RIGHT_ENCODER_SIGN   1.0f
#define LEFT_PWM_SIGN       -1
#define RIGHT_PWM_SIGN      -1

typedef struct
{
    float Current_Encoder;
    float Motor_Pwm;
    float Target_Encoder;
    float Velocity;
} Motor_parameter;

typedef struct
{
    int A;
    int B;
} Encoder;

extern float Move_X, Move_Z;
extern Encoder OriginalEncoder;
extern volatile Motor_parameter MotorA, MotorB;
extern float Voltage_Count, Voltage_All, Voltage;
extern float Velocity_KP, Velocity_KI;
extern int Run_Mode;
extern u8 Control_Work_Mode;
extern u8 Control_Work_Enable;
extern volatile u8 Control_Display_Number;

void Motor_SetPwm(int motor_left, int motor_right);
void Get_Velocity_From_Encoder(int Encoder1, int Encoder2);
void Get_Target_Encoder(float Vx, float Vz);
int Incremental_PI_Left(float Encoder, float Target);
int Incremental_PI_Right(float Encoder, float Target);
int myabs(int a);
void Key(void);

#endif
