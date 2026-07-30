#include "board.h"
#include "servo.h"

#define SERVO_PWM_INST           TIMA0
#define SERVO_PWM_PERIOD_US      20000U
#define SERVO_PWM_CC_INDEX       DL_TIMER_CC_2_INDEX
#define SERVO_PWM_CC_DIR         DL_TIMER_CC2_OUTPUT
#define SERVO_PWM_CC_OUT_INDEX   DL_TIMERA_CAPTURE_COMPARE_2_INDEX

static uint8_t servo_angle;
static uint32_t servo_pulse_us;

static const DL_TimerA_ClockConfig servoClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 9U,
};

static const DL_TimerA_PWMConfig servoPwmConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = SERVO_PWM_PERIOD_US,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

static uint32_t Servo_AngleToPulseUs(uint8_t angle)
{
    uint32_t pulse_range;

    if (angle > 180U) angle = 180U;

    pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    return SERVO_MIN_PULSE_US + ((uint32_t) angle * pulse_range + 90U) / 180U;
}

void Servo_SetPulseUs(uint32_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;
    servo_pulse_us = pulse_us;

    DL_TimerA_setCaptureCompareValue(SERVO_PWM_INST, SERVO_PWM_PERIOD_US - pulse_us, SERVO_PWM_CC_INDEX);
}

uint32_t Servo_GetPulseUs(void)
{
    return servo_pulse_us;
}

void Servo_SetAngle(uint8_t angle)
{
    if (angle > 180U) angle = 180U;
    servo_angle = angle;
    Servo_SetPulseUs(Servo_AngleToPulseUs(angle));
}

uint8_t Servo_GetAngle(void)
{
    return servo_angle;
}

void Servo_Init(void)
{
    DL_TimerA_reset(SERVO_PWM_INST);
    DL_TimerA_enablePower(SERVO_PWM_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM29, IOMUX_PINCM29_PF_TIMA0_CCP2);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_12);

    DL_TimerA_setClockConfig(SERVO_PWM_INST, (DL_TimerA_ClockConfig *) &servoClockConfig);
    DL_TimerA_initPWMMode(SERVO_PWM_INST, (DL_TimerA_PWMConfig *) &servoPwmConfig);

    DL_TimerA_setCaptureCompareOutCtl(SERVO_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        SERVO_PWM_CC_OUT_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(SERVO_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        SERVO_PWM_CC_OUT_INDEX);
    DL_TimerA_setCCPDirection(SERVO_PWM_INST, SERVO_PWM_CC_DIR);

    Servo_SetAngle(93U);
}
