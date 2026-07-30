#include "key.h"

uint8_t keyValue(void)
{
    return (DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) & KEY_key_PIN) > 0 ? 0 : 1;
}

uint8_t controlKey1Value(void)
{
    return keyValue();
}

uint8_t controlKey2Value(void)
{
    return (DL_GPIO_readPins(CONTROL_KEY2_PORT, CONTROL_KEY2_KEY2_PIN) & CONTROL_KEY2_KEY2_PIN) > 0 ? 0 : 1;
}

uint8_t controlKey3Value(void)
{
    return (DL_GPIO_readPins(CONTROL_KEY3_PORT, CONTROL_KEY3_KEY3_PIN) & CONTROL_KEY3_KEY3_PIN) > 0 ? 0 : 1;
}

uint8_t userKeyValue(void)
{
    return (DL_GPIO_readPins(USER_KEY_PORT, USER_KEY_PIN_0_PIN) & USER_KEY_PIN_0_PIN) > 0 ? 0 : 1;
}

void Buzzer_On(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_PIN);
}

void Buzzer_Off(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_PIN);
}

void Buzzer_Set(uint8_t on)
{
    if (on)
    {
        Buzzer_On();
    }
    else
    {
        Buzzer_Off();
    }
}

void Buzzer_Toggle(void)
{
    DL_GPIO_togglePins(BUZZER_PORT, BUZZER_PIN_PIN);
}

static UserKeyState_t key_scan_value(uint16_t freq, uint8_t pressed)
{
    static uint16_t time_core       = 0;
    static uint16_t long_press_time = 0;
    static uint8_t  press_flag      = 0;
    static uint8_t  check_once      = 0;

    float Count_time = (1.0f / (float)freq) * 1000.0f;

    if (check_once)
    {
        press_flag      = 0;
        time_core       = 0;
        long_press_time = 0;
    }
    if (check_once && pressed == 0) check_once = 0;

    if (pressed == 1 && check_once == 0)
    {
        press_flag = 1;
        long_press_time++;
    }
    if (long_press_time > (uint16_t)(500.0f / Count_time))
    {
        check_once = 1;
        return USEKEY_long_click;
    }
    if (press_flag && pressed == 0)
    {
        time_core++;
    }
    if (press_flag && (time_core > (uint16_t)(50.0f / Count_time) &&
                       time_core < (uint16_t)(300.0f / Count_time)))
    {
        if (pressed == 1)
        {
            check_once = 1;
            return USEKEY_double_click;
        }
    }
    else if (press_flag && time_core > (uint16_t)(300.0f / Count_time))
    {
        check_once = 1;
        return USEKEY_single_click;
    }
    return USEKEY_stateless;
}

UserKeyState_t key_scan(uint16_t freq)
{
    return key_scan_value(freq, keyValue());
}

UserKeyState_t user_key_scan(uint16_t freq)
{
    static uint16_t time_core       = 0;
    static uint16_t long_press_time = 0;
    static uint8_t  press_flag      = 0;
    static uint8_t  check_once      = 0;
    uint8_t pressed = userKeyValue();

    float Count_time = (1.0f / (float)freq) * 1000.0f;

    if (check_once)
    {
        press_flag      = 0;
        time_core       = 0;
        long_press_time = 0;
    }
    if (check_once && pressed == 0) check_once = 0;

    if (pressed == 1 && check_once == 0)
    {
        press_flag = 1;
        long_press_time++;
    }
    if (long_press_time > (uint16_t)(500.0f / Count_time))
    {
        check_once = 1;
        return USEKEY_long_click;
    }
    if (press_flag && pressed == 0)
    {
        time_core++;
    }
    if (press_flag && (time_core > (uint16_t)(50.0f / Count_time) &&
                       time_core < (uint16_t)(300.0f / Count_time)))
    {
        if (pressed == 1)
        {
            check_once = 1;
            return USEKEY_double_click;
        }
    }
    else if (press_flag && time_core > (uint16_t)(300.0f / Count_time))
    {
        check_once = 1;
        return USEKEY_single_click;
    }
    return USEKEY_stateless;
}