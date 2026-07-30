#include "show.h"

static int abs_int(int value)
{
    return value < 0 ? -value : value;
}

static int speed_to_mmps(float value)
{
    if (value >= 0.0f) return (int)(value * 1000.0f + 0.5f);
    else return (int)(value * 1000.0f - 0.5f);
}

static void OLED_ShowSignedInt(u8 x, u8 y, int value, u8 len)
{
    if (value < 0) OLED_ShowString(x, y, "-");
    else OLED_ShowString(x, y, "+");

    OLED_ShowNumber(x + 8, y, abs_int(value), len, 12);
}
static int float_to_deci(float value)
{
    if (value >= 0.0f) return (int)(value * 10.0f + 0.5f);
    else return (int)(value * 10.0f - 0.5f);
}

static void OLED_ShowSignedDeci(u8 x, u8 y, float value, u8 len)
{
    int deci_value = float_to_deci(value);
    int deci_abs = abs_int(deci_value);

    if (deci_value < 0) OLED_ShowString(x, y, "-");
    else OLED_ShowString(x, y, "+");

    OLED_ShowNumber(x + 8, y, deci_abs / 10, len, 12);
    OLED_ShowString(x + 8 + len * 8, y, ".");
    OLED_ShowNumber(x + 16 + len * 8, y, deci_abs % 10, 1, 12);
}

static void OLED_ShowSignedCm100(u8 x, u8 y, int cm100)
{
    int cm_abs = abs_int(cm100);

    if (cm100 < 0) OLED_ShowString(x, y, "-");
    else OLED_ShowString(x, y, "+");

    OLED_ShowNumber(x + 8, y, cm_abs / 100, 2, 12);
    OLED_ShowString(x + 24, y, ".");
    OLED_ShowNumber(x + 32, y, cm_abs % 100, 2, 12);
    OLED_ShowString(x + 48, y, "cm");
}


static int gyro_to_dps_int(float value)
{
    if (value >= 0.0f) return (int)(value + 0.5f);
    else return (int)(value - 0.5f);
}
static void OLED_ShowIRState(u8 x, u8 y)
{
    IR_Module_Read();
    OLED_ShowNumber(x, y, ir_s1_state ? 0 : 1, 1, 12);
    OLED_ShowNumber(x + 8, y, ir_s2_state ? 0 : 1, 1, 12);
    OLED_ShowNumber(x + 16, y, ir_dh1_state ? 0 : 1, 1, 12);
    OLED_ShowNumber(x + 24, y, ir_dh2_state ? 0 : 1, 1, 12);
    OLED_ShowNumber(x + 32, y, ir_dh3_state ? 0 : 1, 1, 12);
    OLED_ShowNumber(x + 40, y, ir_dh4_state ? 0 : 1, 1, 12);
    OLED_ShowNumber(x + 48, y, ir_s7_state ? 0 : 1, 1, 12);
    OLED_ShowNumber(x + 56, y, ir_s8_state ? 0 : 1, 1, 12);
}

void oled_show(void)
{
    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));

    OLED_ShowIRState(0, 0);
    if (Control_Work_Mode == CONTROL_MODE_RUN) OLED_ShowString(72, 0, "RUN");
    else OLED_ShowString(72, 0, "SEA");
    OLED_ShowNumber(104, 0, Control_Display_Number, 1, 12);

    OLED_ShowString(0, 12, "L");
    OLED_ShowString(12, 12, "T");
    OLED_ShowSignedInt(24, 12, speed_to_mmps(MotorA.Target_Encoder), 3);
    OLED_ShowString(64, 12, "A");
    OLED_ShowSignedInt(76, 12, speed_to_mmps(MotorA.Current_Encoder), 3);

    OLED_ShowString(0, 24, "R");
    OLED_ShowString(12, 24, "T");
    OLED_ShowSignedInt(24, 24, speed_to_mmps(MotorB.Target_Encoder), 3);
    OLED_ShowString(64, 24, "A");
    OLED_ShowSignedInt(76, 24, speed_to_mmps(MotorB.Current_Encoder), 3);

    OLED_ShowString(0, 36, "BALL");
    OLED_ShowSignedCm100(40, 36, g_k230_ball_cm100);
    OLED_Refresh_Gram();
}

void APP_Show(void)
{
    static u8 flag;
    int Encoder_Left_Show, Encoder_Right_Show, Voltage_Show;

    Voltage_Show = (Voltage - 1000) * 2 / 3;
    if (Voltage_Show < 0) Voltage_Show = 0;
    if (Voltage_Show > 100) Voltage_Show = 100;

    Encoder_Right_Show = Velocity_Right * 1.1f;
    if (Encoder_Right_Show < 0) Encoder_Right_Show = -Encoder_Right_Show;
    Encoder_Left_Show = Velocity_Left * 1.1f;
    if (Encoder_Left_Show < 0) Encoder_Left_Show = -Encoder_Left_Show;

    flag = !flag;
    if (PID_Send == 1)
    {
        printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d:}$",
               (int)Velocity_KP,
               (int)Velocity_KI,
               (int)BaseSpeed,
               0,
               (int)TurnMaxAngle,
               0,
               0,
               (int)ForwardLimit,
               0);
        PID_Send = 0;
    }
    else if (flag == 0)
    {
        printf("{A%d:%d:%d:%d}$", Encoder_Left_Show, Encoder_Right_Show, Voltage_Show, 0);
    }
    else
    {
        printf("{B%d:%d:%d}$", 0, 0, 0);
    }
}
