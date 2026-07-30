#ifndef _IR_MODULE_H
#define _IR_MODULE_H

#include "ti_msp_dl_config.h"
#include "board.h"

extern uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;
extern uint32_t ir_s1_state, ir_s2_state, ir_s7_state, ir_s8_state;

#define IR_DH1_Read()   (DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0)
#define IR_DH2_Read()   (DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0)
#define IR_DH3_Read()   (DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0)
#define IR_DH4_Read()   (DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0)
#define IR_S1_Read()    (DL_GPIO_readPins(IR_S1_PORT, IR_S1_PIN_S1_PIN) ? 1 : 0)
#define IR_S2_Read()    (DL_GPIO_readPins(IR_S2_PORT, IR_S2_PIN_S2_PIN) ? 1 : 0)
#define IR_S7_Read()    (DL_GPIO_readPins(IR_S7_PORT, IR_S7_PIN_S7_PIN) ? 1 : 0)
#define IR_S8_Read()    (DL_GPIO_readPins(IR_S8_PORT, IR_S8_PIN_S8_PIN) ? 1 : 0)

void IR_Module_Read(void);
int IRDM_line_inspection(void);
void IR_Track_Init(void);
void IR_Track_Update(void);
uint8_t IR_Track_Read(void);
void IR_Track_LineInspection(void);

extern float TurnMaxAngle;
extern float BaseSpeed;
extern float ForwardLimit;
extern float GyroStraightK, GyroStraightLimit;
extern float base_speed_mm;
extern float turn_diff;
extern int Smooth;
extern int ir_sensor_state;

#endif