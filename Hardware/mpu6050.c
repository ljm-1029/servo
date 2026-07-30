#include "mpu6050.h"
#include "board.h"

#define MPU6050_ADDR        0x68U
#define MPU6050_ADDR_WRITE  ((MPU6050_ADDR << 1) | 0U)
#define MPU6050_ADDR_READ   ((MPU6050_ADDR << 1) | 1U)

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_GYRO_XOUT_H  0x43U
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_WHO_AM_I     0x75U

#define MPU6050_GYRO_SCALE       131.0f
#define MPU6050_GYRO_CAL_SAMPLES 300U
#define MPU6050_GYRO_DEADBAND    0.35f
#define MPU6050_GYRO_LPF_ALPHA   0.25f

volatile MPU6050_Data_t mpu6050_data;
volatile uint8_t mpu6050_online = 0;
volatile uint16_t mpu6050_update_count = 0;
volatile uint8_t mpu6050_who_am_i = 0;

volatile uint8_t mpu6050_gyro_calibrated = 0;
volatile float mpu6050_gyro_x_bias = 0.0f;
volatile float mpu6050_gyro_y_bias = 0.0f;
volatile float mpu6050_gyro_z_bias = 0.0f;

static float s_gyro_x_filtered = 0.0f;
static float s_gyro_y_filtered = 0.0f;
static float s_gyro_z_filtered = 0.0f;
static uint8_t s_gyro_filter_ready = 0;

static void MPU6050_I2C_PinInit(void)
{
    DL_GPIO_initDigitalOutputFeatures(MPU6050_SCL_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MPU6050_SDA_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_setPins(MPU6050_SCL_PORT, MPU6050_SCL_SCL_PIN);
    DL_GPIO_setPins(MPU6050_SDA_PORT, MPU6050_SDA_SDA_PIN);
    DL_GPIO_enableOutput(MPU6050_SCL_PORT, MPU6050_SCL_SCL_PIN);
    DL_GPIO_disableOutput(MPU6050_SDA_PORT, MPU6050_SDA_SDA_PIN);
}

static void MPU6050_I2C_Delay(void)
{
    delay_us(2);
}

static void MPU6050_SCL_Low(void)
{
    DL_GPIO_enableOutput(MPU6050_SCL_PORT, MPU6050_SCL_SCL_PIN);
    DL_GPIO_clearPins(MPU6050_SCL_PORT, MPU6050_SCL_SCL_PIN);
}

static void MPU6050_SCL_High(void)
{
    DL_GPIO_setPins(MPU6050_SCL_PORT, MPU6050_SCL_SCL_PIN);
    DL_GPIO_enableOutput(MPU6050_SCL_PORT, MPU6050_SCL_SCL_PIN);
}

static void MPU6050_SDA_Low(void)
{
    DL_GPIO_enableOutput(MPU6050_SDA_PORT, MPU6050_SDA_SDA_PIN);
    DL_GPIO_clearPins(MPU6050_SDA_PORT, MPU6050_SDA_SDA_PIN);
}

static void MPU6050_SDA_Release(void)
{
    DL_GPIO_setPins(MPU6050_SDA_PORT, MPU6050_SDA_SDA_PIN);
    DL_GPIO_disableOutput(MPU6050_SDA_PORT, MPU6050_SDA_SDA_PIN);
}

static uint8_t MPU6050_SDA_Read(void)
{
    return (DL_GPIO_readPins(MPU6050_SDA_PORT, MPU6050_SDA_SDA_PIN) != 0U) ? 1U : 0U;
}

static void MPU6050_I2C_Start(void)
{
    MPU6050_SDA_Release();
    MPU6050_SCL_High();
    MPU6050_I2C_Delay();
    MPU6050_SDA_Low();
    MPU6050_I2C_Delay();
    MPU6050_SCL_Low();
}

static void MPU6050_I2C_Stop(void)
{
    MPU6050_SCL_Low();
    MPU6050_SDA_Low();
    MPU6050_I2C_Delay();
    MPU6050_SCL_High();
    MPU6050_I2C_Delay();
    MPU6050_SDA_Release();
    MPU6050_I2C_Delay();
}

static uint8_t MPU6050_I2C_WriteByte(uint8_t data)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8U; i++)
    {
        MPU6050_SCL_Low();
        if ((data & 0x80U) != 0U) MPU6050_SDA_Release();
        else MPU6050_SDA_Low();
        MPU6050_I2C_Delay();
        MPU6050_SCL_High();
        MPU6050_I2C_Delay();
        data <<= 1;
    }

    MPU6050_SCL_Low();
    MPU6050_SDA_Release();
    MPU6050_I2C_Delay();
    MPU6050_SCL_High();
    MPU6050_I2C_Delay();
    ack = MPU6050_SDA_Read();
    MPU6050_SCL_Low();

    return ack == 0U;
}

static uint8_t MPU6050_I2C_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    MPU6050_SDA_Release();
    for (i = 0; i < 8U; i++)
    {
        data <<= 1;
        MPU6050_SCL_Low();
        MPU6050_I2C_Delay();
        MPU6050_SCL_High();
        MPU6050_I2C_Delay();
        if (MPU6050_SDA_Read()) data |= 1U;
    }

    MPU6050_SCL_Low();
    if (ack) MPU6050_SDA_Low();
    else MPU6050_SDA_Release();
    MPU6050_I2C_Delay();
    MPU6050_SCL_High();
    MPU6050_I2C_Delay();
    MPU6050_SCL_Low();
    MPU6050_SDA_Release();

    return data;
}

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t ok = 1U;

    MPU6050_I2C_Start();
    ok &= MPU6050_I2C_WriteByte(MPU6050_ADDR_WRITE);
    ok &= MPU6050_I2C_WriteByte(reg);
    ok &= MPU6050_I2C_WriteByte(data);
    MPU6050_I2C_Stop();

    return ok;
}

static uint8_t MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t ok = 1U;

    MPU6050_I2C_Start();
    ok &= MPU6050_I2C_WriteByte(MPU6050_ADDR_WRITE);
    ok &= MPU6050_I2C_WriteByte(reg);
    MPU6050_I2C_Start();
    ok &= MPU6050_I2C_WriteByte(MPU6050_ADDR_READ);

    if (ok)
    {
        for (i = 0; i < len; i++)
        {
            buf[i] = MPU6050_I2C_ReadByte((i + 1U) < len);
        }
    }

    MPU6050_I2C_Stop();
    return ok;
}

uint8_t MPU6050_ReadID(void)
{
    uint8_t id = 0;
    if (MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &id, 1U) == 0U) return 0;
    return id;
}

static float MPU6050_AbsFloat(float value)
{
    return value < 0.0f ? -value : value;
}

static float MPU6050_ApplyDeadband(float value)
{
    return (MPU6050_AbsFloat(value) < MPU6050_GYRO_DEADBAND) ? 0.0f : value;
}

static float MPU6050_LowPass(float previous, float current)
{
    return previous + MPU6050_GYRO_LPF_ALPHA * (current - previous);
}

static uint8_t MPU6050_ReadRawGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    if (MPU6050_ReadRegs(MPU6050_REG_GYRO_XOUT_H, buf, sizeof(buf)) == 0U)
    {
        return 0;
    }

    *gx = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *gy = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *gz = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    return 1;
}

uint8_t MPU6050_CalibrateGyro(void)
{
    uint16_t i;
    int16_t gx, gy, gz;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    mpu6050_gyro_calibrated = 0;

    for (i = 0; i < MPU6050_GYRO_CAL_SAMPLES; i++)
    {
        if (MPU6050_ReadRawGyro(&gx, &gy, &gz) == 0U)
        {
            return 0;
        }

        sum_x += (float)gx;
        sum_y += (float)gy;
        sum_z += (float)gz;
        delay_ms(2);
    }

    mpu6050_gyro_x_bias = sum_x / (float)MPU6050_GYRO_CAL_SAMPLES;
    mpu6050_gyro_y_bias = sum_y / (float)MPU6050_GYRO_CAL_SAMPLES;
    mpu6050_gyro_z_bias = sum_z / (float)MPU6050_GYRO_CAL_SAMPLES;

    s_gyro_x_filtered = 0.0f;
    s_gyro_y_filtered = 0.0f;
    s_gyro_z_filtered = 0.0f;
    s_gyro_filter_ready = 0;
    mpu6050_gyro_calibrated = 1;

    return 1;
}

uint8_t MPU6050_Init(void)
{
    uint8_t id;

    MPU6050_I2C_PinInit();
    delay_ms(50);

    id = MPU6050_ReadID();
    mpu6050_who_am_i = id;
    if (id != MPU6050_ADDR)
    {
        mpu6050_online = 0;
        return 0;
    }

    if (!MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00U)) return 0;
    delay_ms(10);
    if (!MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07U)) return 0;
    if (!MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x06U)) return 0;
    if (!MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00U)) return 0;
    if (!MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00U)) return 0;

    if (MPU6050_CalibrateGyro() == 0U) return 0;

    mpu6050_online = 1;
    return MPU6050_Update();
}

uint8_t MPU6050_Update(void)
{
    uint8_t buf[14];
    int16_t ax, ay, az, temp, gx, gy, gz;

    if (MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf)) == 0U)
    {
        mpu6050_online = 0;
        return 0;
    }

    ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    temp = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
    gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    mpu6050_data.accel_x_raw = ax;
    mpu6050_data.accel_y_raw = ay;
    mpu6050_data.accel_z_raw = az;
    mpu6050_data.temp_raw = temp;
    mpu6050_data.gyro_x_raw = gx;
    mpu6050_data.gyro_y_raw = gy;
    mpu6050_data.gyro_z_raw = gz;

    mpu6050_data.accel_x_g = (float)ax / 16384.0f;
    mpu6050_data.accel_y_g = (float)ay / 16384.0f;
    mpu6050_data.accel_z_g = (float)az / 16384.0f;

    if (mpu6050_gyro_calibrated)
    {
        float gyro_x = MPU6050_ApplyDeadband(((float)gx - mpu6050_gyro_x_bias) / MPU6050_GYRO_SCALE);
        float gyro_y = MPU6050_ApplyDeadband(((float)gy - mpu6050_gyro_y_bias) / MPU6050_GYRO_SCALE);
        float gyro_z = MPU6050_ApplyDeadband(((float)gz - mpu6050_gyro_z_bias) / MPU6050_GYRO_SCALE);

        if (s_gyro_filter_ready == 0U)
        {
            s_gyro_x_filtered = gyro_x;
            s_gyro_y_filtered = gyro_y;
            s_gyro_z_filtered = gyro_z;
            s_gyro_filter_ready = 1U;
        }
        else
        {
            s_gyro_x_filtered = MPU6050_LowPass(s_gyro_x_filtered, gyro_x);
            s_gyro_y_filtered = MPU6050_LowPass(s_gyro_y_filtered, gyro_y);
            s_gyro_z_filtered = MPU6050_LowPass(s_gyro_z_filtered, gyro_z);
        }

        mpu6050_data.gyro_x = s_gyro_x_filtered;
        mpu6050_data.gyro_y = s_gyro_y_filtered;
        mpu6050_data.gyro_z = s_gyro_z_filtered;
    }
    else
    {
        mpu6050_data.gyro_x = (float)gx / MPU6050_GYRO_SCALE;
        mpu6050_data.gyro_y = (float)gy / MPU6050_GYRO_SCALE;
        mpu6050_data.gyro_z = (float)gz / MPU6050_GYRO_SCALE;
    }

    mpu6050_data.temperature_c = 36.53f + ((float)temp / 340.0f);

    mpu6050_online = 1;
    mpu6050_update_count++;
    return 1;
}
