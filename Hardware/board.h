#ifndef _BOARD_H_
#define _BOARD_H_
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"

/* ---- base types (must precede module includes that use u8/u16)（基础类型定义，必须放在使用 u8/u16 的模块头文件之前） ---- */
#define ABS(a)      (a>0 ? a:(-a))
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only（只读） */
typedef const int16_t sc16;  /*!< Read Only（只读） */
typedef const int8_t sc8;   /*!< Read Only（只读） */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only（只读） */
typedef __I int16_t vsc16;  /*!< Read Only（只读） */
typedef __I int8_t vsc8;   /*!< Read Only（只读） */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only（只读） */
typedef const uint16_t uc16;  /*!< Read Only（只读） */
typedef const uint8_t uc8;   /*!< Read Only（只读） */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only（只读） */
typedef __I uint16_t vuc16;  /*!< Read Only（只读） */
typedef __I uint8_t vuc8;   /*!< Read Only（只读） */

/* ---- module headers（模块头文件） ---- */
#include "oled.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "show.h"
#include "control.h"
#include "uart_callback.h"
#include "adc.h"
#include "IR_Module.h"
#include "mpu6050.h"
#include "servo.h"

// Enumeration of car types（小车类型枚举）
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car
} CarMode;

extern u8 Way_Angle;
extern u16 determine;
extern int Motor_Left,Motor_Right;
extern u8 Control_Work_Mode, Control_Work_Enable, Flag_Show;
extern int Middle_angle;
extern float Voltage,Angle_Balance,Gyro_Balance,Gyro_Turn;
extern u8 LD_Successful_Receive_flag;
extern int Temperature;
extern u32 Distance;
extern u8 Flag_follow,Flag_avoid,delay_50,delay_flag,PID_Send;
extern float Acceleration_Z;
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
extern float RC_Velocity,RC_Turn_Velocity,Move_X,Move_Y,Move_Z,PS2_ON_Flag;
extern u8 one_frame_data_success_flag,one_lap_data_success_flag;
extern int lap_count,PointDataProcess_count,test_once_flag,Dividing_point;
extern float Velocity_Left,Velocity_Right;
extern uint8_t  recv0_flag;
extern u16 test_num,show_cnt;
extern u8 Car_Mode;
extern volatile unsigned long tick_ms;
extern uint8_t RxPacket[36];
extern float Pitch, Roll, Yaw;
extern float GyroX, GyroY, GyroZ;
extern int motor_speed;

#define SysTickMAX_COUNT 0xFFFFFF
#define SysTickFre 80000000
#define SysTick_MS(x)  ((SysTickFre/1000U)*(uint32_t)(x))
#define SysTick_US(x)  ((SysTickFre/1000000U)*(uint32_t)(x))

uint32_t Systick_getTick(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void delay_1us(unsigned long __us);
void delay_1ms(unsigned long ms);
#endif  /* #ifndef _BOARD_H_ */
