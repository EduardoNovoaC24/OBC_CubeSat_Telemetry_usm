#include "mpu6050.h"

HAL_StatusTypeDef MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *whoami)
{
    return HAL_I2C_Mem_Read(
        hi2c,
        MPU6050_ADDR,
        MPU6050_REG_WHO_AM_I,
        I2C_MEMADD_SIZE_8BIT,
        whoami,
        1,
        100
    );
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t whoami = 0;
    uint8_t data = 0;

    if (MPU6050_ReadWhoAmI(hi2c, &whoami) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (whoami != 0x68)
    {
        return HAL_ERROR;
    }

    data = 0x00;

    return HAL_I2C_Mem_Write(
        hi2c,
        MPU6050_ADDR,
        MPU6050_REG_PWR_MGMT1,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );
}

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *data)
{
    uint8_t buffer[14];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c,
        MPU6050_ADDR,
        MPU6050_REG_ACCEL_X_H,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        14,
        100
    );

    if (status != HAL_OK)
    {
        return status;
    }

    data->ax   = (int16_t)(buffer[0]  << 8 | buffer[1]);
    data->ay   = (int16_t)(buffer[2]  << 8 | buffer[3]);
    data->az   = (int16_t)(buffer[4]  << 8 | buffer[5]);
    data->temp = (int16_t)(buffer[6]  << 8 | buffer[7]);
    data->gx   = (int16_t)(buffer[8]  << 8 | buffer[9]);
    data->gy   = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->gz   = (int16_t)(buffer[12] << 8 | buffer[13]);

    return HAL_OK;
}
