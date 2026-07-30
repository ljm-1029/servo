#include "ti_msp_dl_config.h"
#include <string.h>
#include "board.h"
#include "uart_callback.h"

uint8_t uart1_data;

void uart1_send_data(uint8_t data)
{
    while (DL_UART_isTXFIFOFull(UART_1_INST) == true);
    DL_UART_Main_transmitData(UART_1_INST, data);
}

void uart1_send_SendArray(uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        uart1_send_data(data[i]);
    }
}

void uart2_send_data(uint8_t data)
{
    while (DL_UART_isTXFIFOFull(UART_2_INST) == true);
    DL_UART_Main_transmitData(UART_2_INST, data);
}

void uart2_send_SendArray(uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        uart2_send_data(data[i]);
    }
}

void uart2_send_string(const char *str)
{
    while (*str != '\0')
    {
        uart2_send_data((uint8_t)*str++);
    }
}

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_IIDX_RX:
            uart1_data = DL_UART_Main_receiveData(UART_1_INST);
            (void)uart1_data;
            break;

        default:
            break;
    }
}

volatile int16_t  g_follow_x    = 0;
volatile int16_t  g_follow_y    = 0;
volatile uint16_t g_follow_area = 0;
volatile uint8_t  g_data_ready  = 0;
uint8_t res;
volatile u16 g_vision_frame_count = 0;
volatile u8  g_vision_frame_flag  = 0;
uint8_t Packet[9];

volatile K230BallData g_k230_ball;
volatile uint8_t g_k230_ball_new_frame = 0;
volatile int16_t g_k230_ball_cm100 = 0;

#define K230_BALL_FRAME_MAX_LEN       80
#define K230_CM_LINE_MAX_LEN          24
#define K230_BALL_IMAGE_CENTER_X      640
#define K230_BALL_IMAGE_CENTER_Y      480

static char s_k230_rx_frame[K230_BALL_FRAME_MAX_LEN];
static uint8_t s_k230_rx_index = 0;
static char s_k230_cm_line[K230_CM_LINE_MAX_LEN];
static uint8_t s_k230_cm_index = 0;

static uint8_t K230Ball_ParseInt(const char **pp, int32_t *out)
{
    const char *p = *pp;
    int32_t sign = 1;
    int32_t value = 0;
    uint8_t has_digit = 0;

    if (*p == '-')
    {
        sign = -1;
        p++;
    }

    while (*p >= '0' && *p <= '9')
    {
        has_digit = 1;
        value = value * 10 + (int32_t)(*p - '0');
        p++;
    }

    if (has_digit == 0)
    {
        return 0;
    }

    *out = value * sign;
    *pp = p;
    return 1;
}

static uint8_t K230Ball_ExpectChar(const char **pp, char ch)
{
    if (**pp != ch)
    {
        return 0;
    }

    (*pp)++;
    return 1;
}

static void K230Ball_SetNoTarget(void)
{
    g_k230_ball.valid = 0;
    g_k230_ball.count = 0;
    g_k230_ball.x = 0;
    g_k230_ball.y = 0;
    g_k230_ball.w = 0;
    g_k230_ball.h = 0;
    g_k230_ball.cx = 0;
    g_k230_ball.cy = 0;
    g_k230_ball.err_x = 0;
    g_k230_ball.err_y = 0;
    g_k230_ball.area = 0;
    g_k230_ball.score = 0;
    g_k230_ball_cm100 = 0;

    g_follow_x = 0;
    g_follow_y = 0;
    g_follow_area = 0;
    g_data_ready = 1;
    g_vision_frame_count++;
    g_vision_frame_flag = 1;
    g_k230_ball.frame_count++;
    g_k230_ball_new_frame = 1;
}

static uint8_t K230Ball_ParseCm100(const char *line, int16_t *out)
{
    const char *p = line;
    int32_t sign = 1;
    int32_t integer = 0;
    int32_t fraction = 0;
    uint8_t fraction_digits = 0;
    uint8_t has_digit = 0;

    while (*p == ' ' || *p == '\t')
    {
        p++;
    }

    if (*p == '-')
    {
        sign = -1;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    while (*p >= '0' && *p <= '9')
    {
        has_digit = 1;
        integer = integer * 10 + (int32_t)(*p - '0');
        p++;
    }

    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9' && fraction_digits < 2U)
        {
            fraction = fraction * 10 + (int32_t)(*p - '0');
            fraction_digits++;
            p++;
        }
    }

    if (!has_digit)
    {
        return 0;
    }

    while (fraction_digits < 2U)
    {
        fraction *= 10;
        fraction_digits++;
    }

    integer = sign * (integer * 100 + fraction);
    if (integer > 32767) integer = 32767;
    if (integer < -32768) integer = -32768;

    *out = (int16_t)integer;
    return 1;
}

static void K230Ball_SetCm100(int16_t cm100)
{
    if (cm100 <= -9000)
    {
        K230Ball_SetNoTarget();
        return;
    }

    g_k230_ball.valid = 1;
    g_k230_ball.count = 1;
    g_k230_ball.x = cm100;
    g_k230_ball.y = 0;
    g_k230_ball.w = 0;
    g_k230_ball.h = 0;
    g_k230_ball.cx = cm100;
    g_k230_ball.cy = 0;
    g_k230_ball.err_x = cm100;
    g_k230_ball.err_y = 0;
    g_k230_ball.area = 0;
    g_k230_ball.score = 100;
    g_k230_ball.frame_count++;
    g_k230_ball_new_frame = 1;
    g_k230_ball_cm100 = cm100;

    g_follow_x = cm100;
    g_follow_y = 0;
    g_follow_area = 0;
    g_data_ready = 1;
    g_vision_frame_count++;
    g_vision_frame_flag = 1;
}

static void K230Ball_ParseCmLine(const char *line)
{
    int16_t cm100;

    if (K230Ball_ParseCm100(line, &cm100))
    {
        K230Ball_SetCm100(cm100);
    }
}

static uint8_t K230Ball_ParseFrame(const char *frame)
{
    const char *p = frame;
    int32_t count = 0;
    int32_t x = 0;
    int32_t y = 0;
    int32_t w = 0;
    int32_t h = 0;
    int32_t cx = 0;
    int32_t cy = 0;
    int32_t score = 0;
    uint32_t area = 0;

    if (p[0] != '$' || p[1] != 'B' || p[2] != 'A' ||
        p[3] != 'L' || p[4] != 'L' || p[5] != ',')
    {
        return 0;
    }
    p += 6;

    if (!K230Ball_ParseInt(&p, &count))
    {
        return 0;
    }

    if (count <= 0)
    {
        if (*p != '#')
        {
            return 0;
        }
        K230Ball_SetNoTarget();
        return 1;
    }

    if (!K230Ball_ExpectChar(&p, ',') || !K230Ball_ParseInt(&p, &x)) return 0;
    if (!K230Ball_ExpectChar(&p, ',') || !K230Ball_ParseInt(&p, &y)) return 0;
    if (!K230Ball_ExpectChar(&p, ',') || !K230Ball_ParseInt(&p, &w)) return 0;
    if (!K230Ball_ExpectChar(&p, ',') || !K230Ball_ParseInt(&p, &h)) return 0;
    if (!K230Ball_ExpectChar(&p, ',') || !K230Ball_ParseInt(&p, &cx)) return 0;
    if (!K230Ball_ExpectChar(&p, ',') || !K230Ball_ParseInt(&p, &cy)) return 0;
    if (!K230Ball_ExpectChar(&p, ',') || !K230Ball_ParseInt(&p, &score)) return 0;
    if (*p != '#') return 0;

    area = (w > 0 && h > 0) ? ((uint32_t)w * (uint32_t)h) : 0;
    if (area > 65535U)
    {
        area = 65535U;
    }

    g_k230_ball.valid = 1;
    g_k230_ball.count = (count > 255) ? 255 : (uint8_t)count;
    g_k230_ball.x = (int16_t)x;
    g_k230_ball.y = (int16_t)y;
    g_k230_ball.w = (int16_t)w;
    g_k230_ball.h = (int16_t)h;
    g_k230_ball.cx = (int16_t)cx;
    g_k230_ball.cy = (int16_t)cy;
    g_k230_ball.err_x = (int16_t)(cx - K230_BALL_IMAGE_CENTER_X);
    g_k230_ball.err_y = (int16_t)(cy - K230_BALL_IMAGE_CENTER_Y);
    g_k230_ball.area = (uint16_t)area;
    g_k230_ball.score = (score > 255) ? 255 : (uint8_t)score;
    g_k230_ball.frame_count++;
    g_k230_ball_new_frame = 1;

    g_follow_x = g_k230_ball.err_x;
    g_follow_y = g_k230_ball.err_y;
    g_follow_area = g_k230_ball.area;
    g_data_ready = 1;
    g_vision_frame_count++;
    g_vision_frame_flag = 1;

    return 1;
}

void K230Ball_Reset(void)
{
    uint32_t frame_count = g_k230_ball.frame_count;

    memset((void *)&g_k230_ball, 0, sizeof(g_k230_ball));
    g_k230_ball.frame_count = frame_count;
    g_k230_ball_new_frame = 0;
    g_k230_ball_cm100 = 0;

    g_follow_x = 0;
    g_follow_y = 0;
    g_follow_area = 0;
    g_data_ready = 0;
    s_k230_rx_index = 0;
    s_k230_cm_index = 0;
}

void K230Ball_OnRxByte(uint8_t data)
{
    if (data == '$')
    {
        s_k230_rx_index = 0;
        s_k230_rx_frame[s_k230_rx_index++] = (char)data;
        s_k230_cm_index = 0;
        return;
    }

    if (s_k230_rx_index != 0)
    {
        if (s_k230_rx_index >= (K230_BALL_FRAME_MAX_LEN - 1))
        {
            s_k230_rx_index = 0;
            return;
        }

        s_k230_rx_frame[s_k230_rx_index++] = (char)data;

        if (data == '#')
        {
            s_k230_rx_frame[s_k230_rx_index] = '\0';
            (void)K230Ball_ParseFrame(s_k230_rx_frame);
            s_k230_rx_index = 0;
        }
        return;
    }

    if (data == '\n' || data == '\r')
    {
        if (s_k230_cm_index > 0U)
        {
            s_k230_cm_line[s_k230_cm_index] = '\0';
            K230Ball_ParseCmLine(s_k230_cm_line);
            s_k230_cm_index = 0;
        }
        return;
    }

    if (s_k230_cm_index >= (K230_CM_LINE_MAX_LEN - 1))
    {
        s_k230_cm_index = 0;
        return;
    }

    if ((data >= '0' && data <= '9') || data == '-' || data == '+' || data == '.' ||
        data == ' ' || data == '\t')
    {
        s_k230_cm_line[s_k230_cm_index++] = (char)data;
    }
    else
    {
        s_k230_cm_index = 0;
    }
}

void K230Ball_Uart2RxIrqInit(void)
{
    K230Ball_Reset();

    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    DL_UART_Main_disableDMAReceiveEvent(UART_2_INST, DL_UART_DMA_INTERRUPT_RX);
    DL_UART_Main_disableInterrupt(UART_2_INST, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    DL_UART_Main_setRXFIFOThreshold(UART_2_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(UART_2_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);

    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void UART_2_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_2_INST))
            {
                K230Ball_OnRxByte(DL_UART_Main_receiveData(UART_2_INST));
            }
            break;

        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            DL_UART_Main_clearInterruptStatus(UART_2_INST,
                DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
            while (!DL_UART_isRXFIFOEmpty(UART_2_INST))
            {
                K230Ball_OnRxByte(DL_UART_Main_receiveData(UART_2_INST));
            }
            break;

        default:
            break;
    }
}
