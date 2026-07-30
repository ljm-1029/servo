#include "control.h"
#include "IR_Module.h"

u8 Control_Work_Mode = CONTROL_MODE_SEARCH;
u8 Control_Work_Enable = 0;
volatile u8 Control_Display_Number = 0;
Encoder OriginalEncoder;
volatile Motor_parameter MotorA;
volatile Motor_parameter MotorB;

float Velocity_KP = 1050.0f;
float Velocity_KI = 180.0f;

#define VELOCITY_PI_DEADBAND 0.010f

#define SERVO_CENTER_ANGLE    93U
#define SERVO_RIGHT_MAX_ANGLE 113U
#define SERVO_LEFT_MAX_ANGLE  73U
#define BALL_TARGET_CM100     0
#define BALL_DEADBAND_CM100   50
#define BALL_STOP_SPEED_CM100 8
#define BALL_CONTROL_TICKS    4U
#define BALL_NO_FRAME_TICKS   80U
#define BALL_TASK_POS_CM100   500
#define BALL_TASK_NEG_CM100   (-500)
#define BALL_TASK_TIMEOUT_TICKS ((uint16_t)(CONTROL_FREQUENCY * 5U))
#define BALL_SERVO_KP_NUM     5
#define BALL_SERVO_KP_DEN     100
#define BALL_SERVO_KD_NUM     500
#define BALL_SERVO_KD_DEN     100
#define BALL_MIN_CORRECT_ANGLE10 50
#define BALL_HOLD_TRIM_TICKS  40U
#define BUTTON_DEBOUNCE_TICKS 4U
#define BUZZER_BEEP_TICKS     40U

static uint16_t Buzzer_Beep_Ticks = 0;

typedef struct
{
    uint8_t stable_level;
    uint8_t last_sample;
    uint8_t stable_count;
} ButtonDebounce_t;

typedef struct
{
    uint8_t control_ticks;
    uint8_t no_frame_ticks;
    uint8_t have_last_pos;
    uint16_t hold_trim_ticks;
    uint32_t last_frame_count;
    int16_t last_pos_cm100;
    int16_t filtered_pos_cm100;
    int16_t hold_angle10;
    int16_t last_error_cm100;
    int16_t last_speed_cm100;
} BallServoController_t;

typedef enum
{
    BALL_PM5_IDLE = 0,
    BALL_PM5_GO_POS,
    BALL_PM5_GO_NEG,
    BALL_PM5_HOLD_NEG,
    BALL_PM5_TIMEOUT_HOLD
} BallPlusMinus5State_t;

static BallServoController_t Ball_Position_Controller = {
    .no_frame_ticks = BALL_NO_FRAME_TICKS,
    .hold_angle10 = (int16_t)SERVO_CENTER_ANGLE * 10
};
static BallServoController_t Ball_PlusMinus5_Controller = {
    .no_frame_ticks = BALL_NO_FRAME_TICKS,
    .hold_angle10 = (int16_t)SERVO_CENTER_ANGLE * 10
};
static BallPlusMinus5State_t Ball_PlusMinus5_State = BALL_PM5_IDLE;
static uint16_t Ball_PlusMinus5_Ticks = 0;

static void Control_SetDisplayNumber(u8 number)
{
    Control_Display_Number = (Control_Display_Number == number) ? 0U : number;

    if (Control_Display_Number == 2U)
    {
        Buzzer_On();
        Buzzer_Beep_Ticks = BUZZER_BEEP_TICKS;
    }
}

static uint8_t ButtonDebounce_RisingEdge(ButtonDebounce_t *button, uint8_t sample)
{
    if (sample == button->last_sample)
    {
        if (button->stable_count < BUTTON_DEBOUNCE_TICKS)
        {
            button->stable_count++;
        }
    }
    else
    {
        button->last_sample = sample;
        button->stable_count = 0;
    }

    if (button->stable_count >= BUTTON_DEBOUNCE_TICKS && sample != button->stable_level)
    {
        button->stable_level = sample;
        return sample ? 1U : 0U;
    }

    return 0U;
}

static void Buzzer_BeepTask(void)
{
    if (Buzzer_Beep_Ticks > 0U)
    {
        Buzzer_Beep_Ticks--;
        if (Buzzer_Beep_Ticks == 0U)
        {
            Buzzer_Off();
        }
    }
}

static void BallServoController_Reset(BallServoController_t *controller)
{
    controller->control_ticks = 0;
    controller->no_frame_ticks = BALL_NO_FRAME_TICKS;
    controller->have_last_pos = 0;
    controller->hold_trim_ticks = 0;
    controller->last_frame_count = 0;
    controller->last_pos_cm100 = 0;
    controller->filtered_pos_cm100 = 0;
    controller->hold_angle10 = (int16_t)SERVO_CENTER_ANGLE * 10;
    controller->last_error_cm100 = 0;
    controller->last_speed_cm100 = 0;
}

static uint8_t BallServoController_Update(BallServoController_t *controller,
                                          int16_t target_cm100,
                                          uint8_t allow_hold_trim)
{
    int16_t current_pos_cm100;
    int16_t error_cm100;
    int16_t speed_cm100;
    int32_t servo_delta10;
    int32_t servo_angle10;
    uint8_t servo_angle;

    controller->control_ticks++;
    if (controller->control_ticks < BALL_CONTROL_TICKS)
    {
        return 0;
    }
    controller->control_ticks = 0;

    if (g_k230_ball.frame_count != controller->last_frame_count)
    {
        controller->last_frame_count = g_k230_ball.frame_count;
        controller->no_frame_ticks = 0;
    }
    else if (controller->no_frame_ticks < BALL_NO_FRAME_TICKS)
    {
        controller->no_frame_ticks++;
    }

    if (!g_k230_ball.valid || controller->no_frame_ticks >= BALL_NO_FRAME_TICKS)
    {
        Servo_SetAngle((uint8_t)(controller->hold_angle10 / 10));
        controller->have_last_pos = 0;
        controller->hold_trim_ticks = 0;
        return 0;
    }

    current_pos_cm100 = g_k230_ball_cm100;
    if (!controller->have_last_pos)
    {
        controller->filtered_pos_cm100 = current_pos_cm100;
        controller->last_pos_cm100 = current_pos_cm100;
        controller->have_last_pos = 1;
    }
    else
    {
        controller->filtered_pos_cm100 =
            (int16_t)(((int32_t)controller->filtered_pos_cm100 * 3 + current_pos_cm100) / 4);
    }

    error_cm100 = controller->filtered_pos_cm100 - target_cm100;
    speed_cm100 = controller->filtered_pos_cm100 - controller->last_pos_cm100;
    controller->last_pos_cm100 = controller->filtered_pos_cm100;
    controller->last_error_cm100 = error_cm100;
    controller->last_speed_cm100 = speed_cm100;

    if (myabs(error_cm100) <= BALL_DEADBAND_CM100 &&
        myabs(speed_cm100) <= BALL_STOP_SPEED_CM100 &&
        allow_hold_trim)
    {
        servo_angle10 = controller->hold_angle10;
        controller->hold_trim_ticks++;
        if (controller->hold_trim_ticks >= BALL_HOLD_TRIM_TICKS)
        {
            controller->hold_trim_ticks = 0;
            if (error_cm100 > 20 &&
                controller->hold_angle10 < ((int16_t)SERVO_RIGHT_MAX_ANGLE * 10))
            {
                controller->hold_angle10++;
            }
            else if (error_cm100 < -20 &&
                     controller->hold_angle10 > ((int16_t)SERVO_LEFT_MAX_ANGLE * 10))
            {
                controller->hold_angle10--;
            }
        }
    }
    else
    {
        controller->hold_trim_ticks = 0;
        servo_delta10 =
            ((int32_t)BALL_SERVO_KP_NUM * error_cm100 * 10) / BALL_SERVO_KP_DEN +
            ((int32_t)BALL_SERVO_KD_NUM * speed_cm100 * 10) / BALL_SERVO_KD_DEN;
        if (myabs(error_cm100) > BALL_DEADBAND_CM100 &&
            servo_delta10 > -BALL_MIN_CORRECT_ANGLE10 &&
            servo_delta10 < BALL_MIN_CORRECT_ANGLE10)
        {
            servo_delta10 = (error_cm100 > 0) ? BALL_MIN_CORRECT_ANGLE10 : -BALL_MIN_CORRECT_ANGLE10;
        }
        servo_angle10 = controller->hold_angle10 + servo_delta10;
    }

    if (servo_angle10 > ((int32_t)SERVO_RIGHT_MAX_ANGLE * 10))
    {
        servo_angle10 = (int32_t)SERVO_RIGHT_MAX_ANGLE * 10;
    }
    if (servo_angle10 < ((int32_t)SERVO_LEFT_MAX_ANGLE * 10))
    {
        servo_angle10 = (int32_t)SERVO_LEFT_MAX_ANGLE * 10;
    }

    servo_angle = (uint8_t)((servo_angle10 + 5) / 10);
    Servo_SetAngle(servo_angle);
    return 1;
}

static void Ball_PositionControlStart(void)
{
    BallServoController_Reset(&Ball_Position_Controller);
}

static void Ball_PositionControlStop(void)
{
    BallServoController_Reset(&Ball_Position_Controller);
    Servo_SetAngle(SERVO_CENTER_ANGLE);
}

static void Ball_PositionControlTask(void)
{
    (void)BallServoController_Update(&Ball_Position_Controller, BALL_TARGET_CM100, 1U);
}

static void Ball_PlusMinus5Start(void)
{
    BallServoController_Reset(&Ball_PlusMinus5_Controller);
    Ball_PlusMinus5_State = BALL_PM5_GO_POS;
    Ball_PlusMinus5_Ticks = 0;
}

static void Ball_PlusMinus5Stop(void)
{
    BallServoController_Reset(&Ball_PlusMinus5_Controller);
    Ball_PlusMinus5_State = BALL_PM5_IDLE;
    Ball_PlusMinus5_Ticks = 0;
    Servo_SetAngle(SERVO_CENTER_ANGLE);
}

static void Ball_PlusMinus5Task(void)
{
    uint8_t updated;
    int16_t target_cm100;

    if (Ball_PlusMinus5_State == BALL_PM5_IDLE)
    {
        Ball_PlusMinus5Start();
    }

    if (Ball_PlusMinus5_Ticks < BALL_TASK_TIMEOUT_TICKS)
    {
        Ball_PlusMinus5_Ticks++;
    }

    if (Ball_PlusMinus5_Ticks >= BALL_TASK_TIMEOUT_TICKS &&
        Ball_PlusMinus5_State != BALL_PM5_HOLD_NEG)
    {
        Ball_PlusMinus5_State = BALL_PM5_TIMEOUT_HOLD;
    }

    if (Ball_PlusMinus5_State == BALL_PM5_GO_POS)
    {
        target_cm100 = BALL_TASK_POS_CM100;
        updated = BallServoController_Update(&Ball_PlusMinus5_Controller, target_cm100, 0U);
        if (updated &&
            myabs(Ball_PlusMinus5_Controller.last_error_cm100) <= BALL_DEADBAND_CM100)
        {
            Ball_PlusMinus5_State = BALL_PM5_GO_NEG;
        }
    }
    else
    {
        target_cm100 = BALL_TASK_NEG_CM100;
        updated = BallServoController_Update(&Ball_PlusMinus5_Controller, target_cm100, 1U);
        if (updated &&
            Ball_PlusMinus5_State == BALL_PM5_GO_NEG &&
            myabs(Ball_PlusMinus5_Controller.last_error_cm100) <= BALL_DEADBAND_CM100 &&
            myabs(Ball_PlusMinus5_Controller.last_speed_cm100) <= BALL_STOP_SPEED_CM100)
        {
            Ball_PlusMinus5_State = BALL_PM5_HOLD_NEG;
        }
    }
}

static void Ball_ControlIdleTask(void)
{
    Ball_PositionControlStop();
    Ball_PlusMinus5Stop();
}
static int limit_int(int value, int max, int min)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float limit_float(float value, float max, float min)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float abs_float_local(float value)
{
    return value < 0.0f ? -value : value;
}

void Motor_SetPwm(int motor_left, int motor_right)
{
    Motor_Left  = limit_int(motor_left, PWM_MAX, -PWM_MAX);
    Motor_Right = limit_int(motor_right, PWM_MAX, -PWM_MAX);

    if (Control_Work_Mode == CONTROL_MODE_SEARCH || !Control_Work_Enable)
    {
        Set_PWM(0, 0);
        return;
    }

    Set_PWM(Motor_Left, Motor_Right);
}

void Get_Velocity_From_Encoder(int Encoder1, int Encoder2)
{
    float Encoder_A_pr, Encoder_B_pr;

    OriginalEncoder.A = -Encoder1;
    OriginalEncoder.B = -Encoder2;
    Encoder_A_pr = OriginalEncoder.A * LEFT_ENCODER_SIGN;
    Encoder_B_pr = -OriginalEncoder.B * RIGHT_ENCODER_SIGN;

    MotorA.Current_Encoder = Encoder_A_pr * CONTROL_FREQUENCY * Perimeter /
                             (EncoderMultiples * ENCODER_RESOLUTION * MOTOR_GEAR_RATIO);
    MotorB.Current_Encoder = Encoder_B_pr * CONTROL_FREQUENCY * Perimeter /
                             (EncoderMultiples * ENCODER_RESOLUTION * MOTOR_GEAR_RATIO);
}

void Get_Target_Encoder(float Vx, float Vz)
{
    if (Vx < 0) Vz = -Vz;

    MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f;
    MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f;
}

int Incremental_PI_Left(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;

    Bias = Target - Encoder;
    if (Control_Work_Mode == CONTROL_MODE_SEARCH || !Control_Work_Enable)
    {
        Pwm = 0.0f;
        Last_bias = 0.0f;
        return 0;
    }
    if (abs_float_local(Bias) < VELOCITY_PI_DEADBAND)
    {
        Last_bias = 0.0f;
        Pwm = limit_float(Pwm, (float)PWM_MAX, (float)-PWM_MAX);
        return (int)Pwm;
    }
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    Pwm = limit_float(Pwm, (float)PWM_MAX, (float)-PWM_MAX);
    Last_bias = Bias;
    return (int)Pwm;
}

int Incremental_PI_Right(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;

    Bias = Target - Encoder;
    if (Control_Work_Mode == CONTROL_MODE_SEARCH || !Control_Work_Enable)
    {
        Pwm = 0.0f;
        Last_bias = 0.0f;
        return 0;
    }
    if (abs_float_local(Bias) < VELOCITY_PI_DEADBAND)
    {
        Last_bias = 0.0f;
        Pwm = limit_float(Pwm, (float)PWM_MAX, (float)-PWM_MAX);
        return (int)Pwm;
    }
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    Pwm = limit_float(Pwm, (float)PWM_MAX, (float)-PWM_MAX);
    Last_bias = Bias;
    return (int)Pwm;
}

int myabs(int a)
{
    return a < 0 ? -a : a;
}

static void Control_StopWork(void)
{
    Control_Work_Enable = 0;
    Set_PWM(0, 0);
}

static void Control_StartWork(void)
{
    Control_Work_Enable = 1;
}

static void Control_ToggleWork(void)
{
    if (Control_Work_Enable)
    {
        Control_StopWork();
    }
    else
    {
        Control_StartWork();
    }
}

static void Control_SwitchMode(void)
{
    Control_StopWork();
    Control_Work_Mode = (Control_Work_Mode == CONTROL_MODE_SEARCH) ? CONTROL_MODE_RUN : CONTROL_MODE_SEARCH;
}

void Key(void)
{
    static ButtonDebounce_t key2_button = {0, 0, 0};
    static ButtonDebounce_t key3_button = {0, 0, 0};
    UserKeyState_t key_state;

    key_state = key_scan(200);

    if (key_state == USEKEY_single_click)
    {
        Control_SetDisplayNumber(1U);
        Control_ToggleWork();
        return;
    }

    if (key_state == USEKEY_double_click)
    {
        return;
    }

    if (key_state == USEKEY_long_click)
    {
        Control_SwitchMode();
        return;
    }

    if (ButtonDebounce_RisingEdge(&key2_button, controlKey2Value()))
    {
        Control_SetDisplayNumber(2U);
        Control_StopWork();
        if (Control_Display_Number == 2U)
        {
            Ball_PlusMinus5Start();
        }
        else
        {
            Ball_PlusMinus5Stop();
        }
    }

    if (ButtonDebounce_RisingEdge(&key3_button, controlKey3Value()))
    {
        Control_SetDisplayNumber(3U);
        Control_StopWork();
        if (Control_Display_Number == 3U)
        {
            Ball_PositionControlStart();
        }
        else
        {
            Ball_PositionControlStop();
        }
    }
}
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if (DL_TIMER_IIDX_ZERO)
        {
            Key();
            Buzzer_BeepTask();
            if (Control_Display_Number == 2U)
            {
                Ball_PlusMinus5Task();
            }
            else if (Control_Display_Number == 3U)
            {
                Ball_PositionControlTask();
            }
            else
            {
                Ball_ControlIdleTask();
            }
            LED_Flash(100);

            Get_Velocity_From_Encoder(-Get_Encoder_countA, -Get_Encoder_countB);
            Get_Encoder_countA = Get_Encoder_countB = 0;

            IRDM_line_inspection();

            MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder, MotorA.Target_Encoder);
            MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder, MotorB.Target_Encoder);

            if (Control_Work_Mode == CONTROL_MODE_SEARCH)
            {
                Motor_SetPwm(0, 0);
            }
            else
            {
                Motor_SetPwm(LEFT_PWM_SIGN * (int)MotorA.Motor_Pwm,
                             RIGHT_PWM_SIGN * (int)MotorB.Motor_Pwm);
            }
        }
    }
}
