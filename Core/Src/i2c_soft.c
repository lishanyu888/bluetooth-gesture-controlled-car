#include "i2c_soft.h"

#include "delay.h"

#define SOFT_I2C_DELAY_US 5U

static void SoftI2C_SDA_High(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET);
}

static void SoftI2C_SDA_Low(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET);
}

static void SoftI2C_SCL_High(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
}

static void SoftI2C_SCL_Low(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
}

static GPIO_PinState SoftI2C_ReadSDA(void)
{
    return HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN);
}

static void SoftI2C_Delay(void)
{
    Delay_Us(SOFT_I2C_DELAY_US);
}

static void SoftI2C_Start(void)
{
    SoftI2C_SDA_High();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SDA_Low();
    SoftI2C_Delay();
    SoftI2C_SCL_Low();
}

static void SoftI2C_Stop(void)
{
    SoftI2C_SDA_Low();
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SDA_High();
    SoftI2C_Delay();
}

static uint8_t SoftI2C_WaitAck(void)
{
    uint8_t timeout = 0U;

    SoftI2C_SDA_High();
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();

    while (SoftI2C_ReadSDA() == GPIO_PIN_SET) {
        timeout++;
        if (timeout > 20U) {
            SoftI2C_Stop();
            return 1U;
        }
        SoftI2C_Delay();
    }

    SoftI2C_SCL_Low();
    return 0U;
}

static void SoftI2C_SendAck(void)
{
    SoftI2C_SDA_Low();
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SCL_Low();
    SoftI2C_SDA_High();
}

static void SoftI2C_SendNack(void)
{
    SoftI2C_SDA_High();
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SCL_Low();
}

static void SoftI2C_SendByte(uint8_t data)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        SoftI2C_SCL_Low();
        if ((data & 0x80U) != 0U) {
            SoftI2C_SDA_High();
        } else {
            SoftI2C_SDA_Low();
        }
        data <<= 1;
        SoftI2C_Delay();
        SoftI2C_SCL_High();
        SoftI2C_Delay();
    }
    SoftI2C_SCL_Low();
}

static uint8_t SoftI2C_ReadByteInternal(void)
{
    uint8_t i;
    uint8_t data = 0U;

    SoftI2C_SDA_High();

    for (i = 0U; i < 8U; i++) {
        data <<= 1;
        SoftI2C_SCL_Low();
        SoftI2C_Delay();
        SoftI2C_SCL_High();
        if (SoftI2C_ReadSDA() == GPIO_PIN_SET) {
            data |= 0x01U;
        }
        SoftI2C_Delay();
    }
    SoftI2C_SCL_Low();

    return data;
}

void SoftI2C_Init(void)
{
    Delay_Init();
    SoftI2C_SDA_High();
    SoftI2C_SCL_High();
    Delay_Ms(1U);
}

void SoftI2C_Recover(void)
{
    uint8_t i;

    SoftI2C_SDA_High();
    for (i = 0U; i < 9U; i++) {
        SoftI2C_SCL_Low();
        SoftI2C_Delay();
        SoftI2C_SCL_High();
        SoftI2C_Delay();
    }
    SoftI2C_Stop();
}

uint8_t SoftI2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    SoftI2C_Start();
    SoftI2C_SendByte((uint8_t)(dev_addr << 1));
    if (SoftI2C_WaitAck() != 0U) {
        return 1U;
    }

    SoftI2C_SendByte(reg_addr);
    if (SoftI2C_WaitAck() != 0U) {
        return 2U;
    }

    SoftI2C_SendByte(data);
    if (SoftI2C_WaitAck() != 0U) {
        return 3U;
    }

    SoftI2C_Stop();
    return 0U;
}

uint8_t SoftI2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return SoftI2C_ReadBytes(dev_addr, reg_addr, data, 1U);
}

uint8_t SoftI2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if ((data == NULL) || (len == 0U)) {
        return 4U;
    }

    SoftI2C_Start();
    SoftI2C_SendByte((uint8_t)(dev_addr << 1));
    if (SoftI2C_WaitAck() != 0U) {
        return 1U;
    }

    SoftI2C_SendByte(reg_addr);
    if (SoftI2C_WaitAck() != 0U) {
        return 2U;
    }

    SoftI2C_Start();
    SoftI2C_SendByte((uint8_t)((dev_addr << 1) | 0x01U));
    if (SoftI2C_WaitAck() != 0U) {
        return 3U;
    }

    for (i = 0U; i < len; i++) {
        data[i] = SoftI2C_ReadByteInternal();
        if (i < (len - 1U)) {
            SoftI2C_SendAck();
        } else {
            SoftI2C_SendNack();
        }
    }

    SoftI2C_Stop();
    return 0U;
}
