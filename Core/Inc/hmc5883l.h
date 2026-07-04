#ifndef HMC5883L_H
#define HMC5883L_H

#include "stm32l4xx_hal.h"

/* Dirección I2C del HMC5883L (fija en hardware) */
#define HMC5883L_ADDR        (0x1E << 1)   /* HAL usa dirección desplazada 1 bit */

/* Registros internos */
#define HMC5883L_REG_CONFIG_A   0x00
#define HMC5883L_REG_CONFIG_B   0x01
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA_X_MSB 0x03       /* Orden: X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB */
#define HMC5883L_REG_STATUS     0x09
#define HMC5883L_REG_ID_A       0x0A       /* Debe leer 'H' */
#define HMC5883L_REG_ID_B       0x0B       /* Debe leer '4' */
#define HMC5883L_REG_ID_C       0x0C       /* Debe leer '3' */

/* Configuración base equivalente a jarzebski/Arduino-HMC5883L:
 * 1 muestra, 15 Hz, bias normal, rango +/-1.3 Ga, continuo. */
#define HMC5883L_CONFIG_A_1AVG_15HZ_NORMAL 0x10
#define HMC5883L_CONFIG_B_RANGE_1_3GA      0x20
#define HMC5883L_MODE_CONTINUOUS           0x00
#define HMC5883L_STATUS_LOCK               0x02
#define HMC5883L_STATUS_READY              0x01
#define HMC5883L_OVERFLOW_VALUE           -4096

/* Rango +/-1.3 Ga: Jarzebski normaliza con 0.92 mG/LSB. */
#define HMC5883L_MG_PER_LSB_1_3GA          0.92f

/* Conversion directa RAW -> mG. No se aplica compensacion de bias hard-iron. */

/* ============================================================
 * Montaje HMC5883L -> cuerpo
 *
 * El MPU ya esta validado; esta matriz debe ajustarse SOLO para
 * que el magnetometro quede en el mismo marco de cuerpo:
 *   X_body adelante, Y_body izquierda, Z_body arriba.
 *
 * Si al apuntar al sur queda norte: usar heading offset 180 en main.c.
 * Si este/oeste quedan invertidos: cambiar HMC5883L_BODY_Y_SIGN.
 * Si norte/sur quedan invertidos: cambiar HMC5883L_BODY_X_SIGN.
 * Si un giro horizontal mueve poco o raro: intercambiar BODY_X_AXIS/BODY_Y_AXIS.
 * ============================================================ */
#define HMC5883L_AXIS_X      0U
#define HMC5883L_AXIS_Y      1U
#define HMC5883L_AXIS_Z      2U

#define HMC5883L_BODY_X_AXIS HMC5883L_AXIS_X
#define HMC5883L_BODY_X_SIGN (+1.0f)
#define HMC5883L_BODY_Y_AXIS HMC5883L_AXIS_Y
#define HMC5883L_BODY_Y_SIGN (-1.0f)
#define HMC5883L_BODY_Z_AXIS HMC5883L_AXIS_Z
#define HMC5883L_BODY_Z_SIGN (+1.0f)

typedef struct
{
    int16_t x;   /* campo magnético eje X [LSB] */
    int16_t y;   /* campo magnético eje Y [LSB] */
    int16_t z;   /* campo magnético eje Z [LSB] */
} HMC5883L_RawData_t;

typedef struct
{
    float x;     /* campo magnético calibrado eje X [mG] */
    float y;     /* campo magnético calibrado eje Y [mG] */
    float z;     /* campo magnético calibrado eje Z [mG] */
} HMC5883L_CalData_t;

HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HMC5883L_ReadRaw(I2C_HandleTypeDef *hi2c, HMC5883L_RawData_t *data);
HAL_StatusTypeDef HMC5883L_ReadId(I2C_HandleTypeDef *hi2c, uint8_t id[3]);

/* Convierte RAW -> mG sin compensar bias del magnetometro */
void HMC5883L_ApplyCalibration(const HMC5883L_RawData_t *raw,
                               HMC5883L_CalData_t *cal);
void HMC5883L_RotateToBody(const HMC5883L_CalData_t *chip,
                           HMC5883L_CalData_t *body);

#endif /* HMC5883L_H */
