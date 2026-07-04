#include "hmc5883l.h"

static float HMC5883L_SelectAxis(const HMC5883L_CalData_t *data, uint8_t axis)
{
    switch (axis) {
        case HMC5883L_AXIS_X:
            return data->x;
        case HMC5883L_AXIS_Y:
            return data->y;
        case HMC5883L_AXIS_Z:
            return data->z;
        default:
            return 0.0f;
    }
}

HAL_StatusTypeDef HMC5883L_ReadId(I2C_HandleTypeDef *hi2c, uint8_t id[3])
{
    if (id == 0) {
        return HAL_ERROR;
    }
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC5883L_REG_ID_A,
                                             I2C_MEMADD_SIZE_8BIT, &id[0], 1, 100);
    if (ret != HAL_OK) return ret;

    ret = HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC5883L_REG_ID_B,
                           I2C_MEMADD_SIZE_8BIT, &id[1], 1, 100);
    if (ret != HAL_OK) return ret;

    return HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC5883L_REG_ID_C,
                            I2C_MEMADD_SIZE_8BIT, &id[2], 1, 100);
}

HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t id[3] = {0};
    uint8_t data;

    ret = HMC5883L_ReadId(hi2c, id);
    if ((ret != HAL_OK) || (id[0] != 'H') || (id[1] != '4') || (id[2] != '3')) {
        return HAL_ERROR;
    }

    /* Config A: 1 muestra, 15 Hz, bias normal. */
    data = HMC5883L_CONFIG_A_1AVG_15HZ_NORMAL;
    ret = HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC5883L_REG_CONFIG_A,
                            I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    /* Config B: +/-1.3 Ga, 0.92 mG/LSB. */
    data = HMC5883L_CONFIG_B_RANGE_1_3GA;
    ret = HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC5883L_REG_CONFIG_B,
                            I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    /* Mode: continuous measurement */
    data = HMC5883L_MODE_CONTINUOUS;
    ret = HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC5883L_REG_MODE,
                            I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (ret != HAL_OK) return ret;

    /* Primera lectura dummy para sacar el sensor del estado inicial */
    HAL_Delay(10);
    uint8_t dummy[6];
    HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB,
                     I2C_MEMADD_SIZE_8BIT, dummy, 6, 100);
    HAL_Delay(15);

    return HAL_OK;
}

HAL_StatusTypeDef HMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c,
                                    HMC5883L_RawData_t *data)
{
    uint8_t status = 0;
    const uint32_t start = HAL_GetTick();

    /* Esperar DRDY (bit 0 del registro STATUS) */
    while ((status & HMC5883L_STATUS_READY) == 0U)
    {
        HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC5883L_REG_STATUS,
                                                 I2C_MEMADD_SIZE_8BIT, &status, 1, 10);
        if (ret != HAL_OK)
            return ret;
        if ((HAL_GetTick() - start) > 20U)
            return HAL_TIMEOUT;
    }

    uint8_t buf[6];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB,
        I2C_MEMADD_SIZE_8BIT, buf, 6, 100);

    if (ret != HAL_OK) return ret;

    /* Orden HMC5883L: X_MSB X_LSB Z_MSB Z_LSB Y_MSB Y_LSB */
    data->x = (int16_t)((buf[0] << 8) | buf[1]);
    data->z = (int16_t)((buf[2] << 8) | buf[3]);
    data->y = (int16_t)((buf[4] << 8) | buf[5]);

    /* -4096 indica overflow/saturación en HMC5883L; no es una muestra válida. */
    if ((data->x == HMC5883L_OVERFLOW_VALUE) ||
        (data->y == HMC5883L_OVERFLOW_VALUE) ||
        (data->z == HMC5883L_OVERFLOW_VALUE)) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HMC5883L_ApplyCalibration(const HMC5883L_RawData_t *raw,
                               HMC5883L_CalData_t *cal)
{
    if (!raw || !cal) return;
    cal->x = (float)raw->x * HMC5883L_MG_PER_LSB_1_3GA;
    cal->y = (float)raw->y * HMC5883L_MG_PER_LSB_1_3GA;
    cal->z = (float)raw->z * HMC5883L_MG_PER_LSB_1_3GA;
}

void HMC5883L_RotateToBody(const HMC5883L_CalData_t *chip,
                           HMC5883L_CalData_t *body)
{
    if (!chip || !body) return;

    body->x = HMC5883L_BODY_X_SIGN * HMC5883L_SelectAxis(chip, HMC5883L_BODY_X_AXIS);
    body->y = HMC5883L_BODY_Y_SIGN * HMC5883L_SelectAxis(chip, HMC5883L_BODY_Y_AXIS);
    body->z = HMC5883L_BODY_Z_SIGN * HMC5883L_SelectAxis(chip, HMC5883L_BODY_Z_AXIS);
}
