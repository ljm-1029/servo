/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"

u8 Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;                 // motor PWM variables（电机 PWM 变量）
u8 PID_Send;                                 // delay/debug parameter variable（延时和调试参数变量）
float RC_Velocity = 200, RC_Turn_Velocity, Move_X, Move_Y, Move_Z, PS2_ON_Flag;
float Velocity_Left, Velocity_Right;         // wheel speed (mm/s)（轮速，单位 mm/s）
u16 test_num, show_cnt;
float Voltage = 0;
int motor_speed = 30;
extern u8 sb;

static void Startup_ReadyBeep(void)
{
    Buzzer_On();
    delay_ms(150);
    Buzzer_Off();
}

int main(void)
{
    SYSCFG_DL_init();
    Servo_Init();
    DL_GPIO_setDigitalInternalResistor(GPIO_UART_1_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);
    IR_Track_Init();

    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    /* K230 UART2: PA21 TX, PA22 RX, 115200 8N1. */
    K230Ball_Uart2RxIrqInit();


    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

    OLED_Init();
    if (MPU6050_Init())
    {
        Startup_ReadyBeep();
    }

    while (1)
    {
        MPU6050_Update();
        Voltage = Get_battery_volt();
        oled_show();
    }
}