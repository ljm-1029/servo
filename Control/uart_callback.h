#ifndef _UART_CALLBACK_H_
#define _UART_CALLBACK_H_

#include "board.h"

typedef struct {
    uint8_t valid;
    uint8_t count;
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
    int16_t cx;
    int16_t cy;
    int16_t err_x;
    int16_t err_y;
    uint16_t area;
    uint8_t score;
    uint32_t frame_count;
} K230BallData;

void uart1_send_data(uint8_t data);
void uart1_send_SendArray(uint8_t *data, uint8_t len);
void uart2_send_data(uint8_t data);
void uart2_send_SendArray(uint8_t *data, uint8_t len);
void uart2_send_string(const char *str);

void K230Ball_Reset(void);
void K230Ball_OnRxByte(uint8_t data);
void K230Ball_Uart2RxIrqInit(void);

extern volatile int16_t  g_follow_x;
extern volatile int16_t  g_follow_y;
extern volatile uint8_t  g_data_ready;
extern volatile uint16_t g_follow_area;

extern volatile u16 g_vision_frame_count;
extern volatile u8  g_vision_frame_flag;

extern volatile K230BallData g_k230_ball;
extern volatile uint8_t g_k230_ball_new_frame;
extern volatile int16_t g_k230_ball_cm100;

extern uint8_t Packet[9];

#endif
