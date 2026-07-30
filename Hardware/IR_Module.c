#include "IR_Module.h"

uint32_t ir_dh1_state = 1;
uint32_t ir_dh2_state = 1;
uint32_t ir_dh3_state = 1;
uint32_t ir_dh4_state = 1;
uint32_t ir_s1_state = 1;
uint32_t ir_s2_state = 1;
uint32_t ir_s7_state = 1;
uint32_t ir_s8_state = 1;

float BaseSpeed = 350.0f;             // 正常循迹基础速度，单位 mm/s；速度越大越快，但弯道越容易冲出
float GyroStraightK = 0.25f;          // 直线陀螺仪阻尼系数；直线摆尾可小幅增大，进弯变钝或反向摆动就减小/改负
float GyroStraightLimit = 6.0f;       // 陀螺仪单次最大修正量，防止姿态噪声造成大幅摆动
float GyroStraightApplyLimit = 55.0f; // 小偏差范围内启用直线平滑；值过小压不住摆尾，过大会让进弯变钝

float TurnMaxAngle = 180.0f;          // 最大转向修正量；弯道转不过去可增大，转向太猛或摆动可减小
float LineTurnDirection = 1.0f;       // 循迹转向方向；如果压线后反向修正，改为 -1.0f

float ForwardLimit = 320.0f;         // 根据偏差降速的范围；值越小，轻微偏差时降速越明显

//大偏差判定
float BigErrorThreshold = 110.0f;     // 大偏差阈值；提高后避免第 1/6 路一触发就猛降速
float BigErrorTurnK = 1.00f;          // 大偏差转向放大倍数；急弯转不过去可增大，甩头严重可减小
float BigErrorSpeedRate = 0.75f;      // 大偏差降速比例；弯道冲出去可减小，弯道太慢可增大
float LineDampingK = 0.0f;            // 预留阻尼系数；当前使用转向限斜率抑制离散传感器跳变

float base_speed_mm = 0.0f;
float turn_diff = 0.0f;
int Smooth = 0;
int ir_sensor_state = 15;

#define TRACK_TUNE_SPEED_ONLY 0        // 1: 只调速度环，不循迹；速度环稳定后改为 0 再调循迹环
#define TRACK_TUNE_SPEED_MM   320.0f   // 速度环测试目标速度，单位 mm/s

#define SMOOTH_PERIOD       100       // 启动/恢复时的平滑周期，数值越大加速越慢
#define SMOOTH_MIN_RATE     0.6f      // 平滑开始时的最低速度比例
#define TRACK_MIN_SPEED_MM  280.0f    // 弯道或大偏差时允许的最低循迹速度，单位 mm/s
#define TRACK_LOST_SPEED_MM 280.0f    // 丢线时保持的低速，单位 mm/s
#define CENTER_LEFT_STATE   0x10      // 只有中间偏左传感器 DH2 识别到黑线
#define CENTER_RIGHT_STATE  0x08      // 只有中间偏右传感器 DH3 识别到黑线
#define CENTER_BOTH_STATE   0x18      // 中间两个传感器 DH2、DH3 同时识别到黑线
#define CENTER_LINE_ERROR   4.0f      // 中间单边压线时的小修正；方向正确后用于及时拉回中心
#define TRACK_SENSOR_MASK   0x7E      // 循迹只使用中间 6 个传感器，屏蔽 S1/S8 避免扫到赛道字母干扰
#define STRAIGHT_FILTER_ALPHA 0.70f   // 直线小偏差滤波系数；越小越稳但响应越慢，1.0 表示不滤波
#define TURN_OUTPUT_STEP_MM 3.0f      // 每个控制周期转向输出最大变化量，直线摆尾时减小，弯道反应慢时增大

#define TRACK_1_STATE       0x40      // 6 路中的第 1 路：左侧救线
#define TRACK_2_STATE       0x20      // 6 路中的第 2 路：左弯主工作区
#define TRACK_3_STATE       0x10      // 6 路中的第 3 路：直线左中心
#define TRACK_4_STATE       0x08      // 6 路中的第 4 路：直线右中心
#define TRACK_5_STATE       0x04      // 6 路中的第 5 路：右弯主工作区
#define TRACK_6_STATE       0x02      // 6 路中的第 6 路：右侧救线
#define TRACK_LEFT_CURVE_STATE  (TRACK_2_STATE | TRACK_3_STATE)
#define TRACK_RIGHT_CURVE_STATE (TRACK_4_STATE | TRACK_5_STATE)

static float abs_float(float value)
{
    return value < 0.0f ? -value : value;
}

static float limit_float(float value, float max, float min)
{
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

static int sensor_black_count(int sensor_state)
{
    int i;
    int count = 0;

    for(i = 0; i < 8; i++)
    {
        if(sensor_state & (1 << i)) count++;
    }

    return count;
}

static float sensor_weighted_error(int sensor_state)
{
    static const float weight[8] = {-80.0f, -50.0f, -20.0f, 0.0f, 0.0f, 20.0f, 50.0f, 80.0f};
    int i;
    int count = 0;
    float sum = 0.0f;

    for(i = 0; i < 8; i++)
    {
        if(sensor_state & (0x80 >> i))
        {
            sum += weight[i];
            count++;
        }
    }

    if(count == 0) return 0.0f;
    return sum / (float)count;
}

static float sensor_track_error(int sensor_state)
{
    switch(sensor_state)
    {
        case CENTER_BOTH_STATE:
            return 0.0f;

        case CENTER_LEFT_STATE:
            return -CENTER_LINE_ERROR;

        case CENTER_RIGHT_STATE:
            return CENTER_LINE_ERROR;

        case TRACK_LEFT_CURVE_STATE:
            return -30.0f;

        case TRACK_RIGHT_CURVE_STATE:
            return 30.0f;

        case TRACK_2_STATE:
            return -45.0f;

        case TRACK_5_STATE:
            return 45.0f;

        case (TRACK_1_STATE | TRACK_2_STATE):
            return -70.0f;

        case (TRACK_5_STATE | TRACK_6_STATE):
            return 70.0f;

        case TRACK_1_STATE:
            return -85.0f;

        case TRACK_6_STATE:
            return 85.0f;

        default:
            return sensor_weighted_error(sensor_state);
    }
}

static float smooth_speed_rate(void)
{
    float rate;

    if(Smooth <= 0) return 1.0f;
    if(Smooth > SMOOTH_PERIOD) Smooth = SMOOTH_PERIOD;

    rate = SMOOTH_MIN_RATE +
           (1.0f - SMOOTH_MIN_RATE) *
           (float)(SMOOTH_PERIOD - Smooth) / (float)SMOOTH_PERIOD;

    Smooth--;
    return rate;
}

static float smooth_straight_turn_diff(float target)
{
    static uint8_t filter_ready = 0;
    static float filtered = 0.0f;

    if(!filter_ready)
    {
        filtered = target;
        filter_ready = 1;
        return filtered;
    }

    filtered += STRAIGHT_FILTER_ALPHA * (target - filtered);

    return filtered;
}

void IR_Module_Read(void)
{
    ir_s1_state = IR_S1_Read();
    ir_s2_state = IR_S2_Read();
    ir_dh1_state = IR_DH1_Read();
    ir_dh2_state = IR_DH2_Read();
    ir_dh3_state = IR_DH3_Read();
    ir_dh4_state = IR_DH4_Read();
    ir_s7_state = IR_S7_Read();
    ir_s8_state = IR_S8_Read();
}

static int IR_Module_GetSensorState(void)
{
    IR_Module_Read();

    return ((ir_s1_state  ? 0 : 1) << 7) |
           ((ir_s2_state  ? 0 : 1) << 6) |
           ((ir_dh1_state ? 0 : 1) << 5) |
           ((ir_dh2_state ? 0 : 1) << 4) |
           ((ir_dh3_state ? 0 : 1) << 3) |
           ((ir_dh4_state ? 0 : 1) << 2) |
           ((ir_s7_state  ? 0 : 1) << 1) |
            (ir_s8_state  ? 0 : 1);
}

void IR_Track_Init(void)
{
    IR_Module_Read();
}

void IR_Track_Update(void)
{
    IR_Module_Read();
}

uint8_t IR_Track_Read(void)
{
    return (uint8_t)IR_Module_GetSensorState();
}

int IRDM_line_inspection(void)
{
#if TRACK_TUNE_SPEED_ONLY
    base_speed_mm = TRACK_TUNE_SPEED_MM;
    turn_diff = 0.0f;
    ir_sensor_state = IR_Track_Read() & TRACK_SENSOR_MASK;

    MotorA.Target_Encoder = TRACK_TUNE_SPEED_MM * 0.001f;
    MotorB.Target_Encoder = TRACK_TUNE_SPEED_MM * 0.001f;

    return 0;
#else
    static float last_turn_diff = 0.0f;
    float left_motor_speed;
    float right_motor_speed;
    float abs_turn;
    float smooth_rate;
    float turn_output;
    static float last_turn_output = 0.0f;
    int sensor_state;
    int black_count;

    sensor_state = IR_Track_Read() & TRACK_SENSOR_MASK;
    ir_sensor_state = sensor_state;

    black_count = sensor_black_count(sensor_state);

    if(black_count == 0)
    {
        turn_diff = last_turn_diff;
    }
    else
    {
        turn_diff = limit_float(sensor_weighted_error(sensor_state), TurnMaxAngle, -TurnMaxAngle);
        last_turn_diff = turn_diff;
    }

    if(black_count > 0 && black_count <= 3 && abs_float(turn_diff) <= GyroStraightApplyLimit)
    {
        float gyro_comp = limit_float(-mpu6050_data.gyro_z * GyroStraightK,
                                      GyroStraightLimit, -GyroStraightLimit);
        turn_diff += gyro_comp;
    }

    if(black_count > 0 && abs_float(turn_diff) <= GyroStraightApplyLimit)
    {
        turn_diff = smooth_straight_turn_diff(turn_diff);
    }
    last_turn_diff = turn_diff;

    abs_turn = abs_float(turn_diff);
    if(black_count == 0)
    {
        base_speed_mm = TRACK_LOST_SPEED_MM;
    }
    else if(abs_turn < ForwardLimit)
    {
        base_speed_mm = BaseSpeed - ((BaseSpeed - TRACK_MIN_SPEED_MM) * (abs_turn / ForwardLimit));
    }
    else
    {
        base_speed_mm = TRACK_MIN_SPEED_MM;
    }

    smooth_rate = smooth_speed_rate();
    base_speed_mm *= smooth_rate;

    turn_output = LineTurnDirection * turn_diff;
    if(abs_turn > BigErrorThreshold)
    {
        turn_output *= BigErrorTurnK;
        base_speed_mm *= BigErrorSpeedRate;
    }

    turn_output = last_turn_output +
                  limit_float(turn_output - last_turn_output,
                              TURN_OUTPUT_STEP_MM, -TURN_OUTPUT_STEP_MM);
    last_turn_output = turn_output;

    left_motor_speed  = 0.001f * (base_speed_mm - turn_output);
    right_motor_speed = 0.001f * (base_speed_mm + turn_output);

    MotorA.Target_Encoder = left_motor_speed;
    MotorB.Target_Encoder = right_motor_speed;

    return 0;
#endif
}

void IR_Track_LineInspection(void)
{
    (void)IRDM_line_inspection();
}
