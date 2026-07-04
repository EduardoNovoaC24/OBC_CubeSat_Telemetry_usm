#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <math.h>

/* ── Dirección I2C ──────────────────────────────────────────────────────────*/
#define MPU6050_ADDR            (0x68 << 1)

/* ── Registros internos ─────────────────────────────────────────────────────*/
#define MPU6050_REG_WHO_AM_I    0x75
#define MPU6050_REG_PWR_MGMT1  0x6B
#define MPU6050_REG_ACCEL_X_H  0x3B
#define MPU6050_REG_GYRO_X_H   0x43

/* ── Escalas de conversión ───────────────────────────────────────────────────
 * Acelerómetro: ±2g     → 16384 LSB/g
 * Giróscopo:    ±250°/s →   131 LSB/(°/s)                                   */
#define MPU6050_ACCEL_SCALE     16384.0f
#define MPU6050_GYRO_SCALE        131.0f
#define MPU6050_TEMP_SCALE        340.0f
#define MPU6050_TEMP_OFFSET        36.53f

/* ── Bias de calibración estática ────────────────────────────────────────────
 * Calibrados en posición definitiva: de canto, lado largo arriba.
 * Muestras: 2078  |  σ_accel < 0.005g
 *
 * SOLO se corrige el offset del sensor (error del chip), NO la gravedad.
 * La gravedad la maneja Fusion AHRS internamente.
 *
 * Cómo se obtienen:
 *   bias_accel = promedio_medido - valor_esperado
 *
 *   Posición de calibración: sensor de canto, chip_X apunta DOWN (-Z mundo)
 *   → chip_X debería medir -1g  → bias_AX = promedio(ax_g) - (-1.0) NO
 *   → chip_X debería medir +1g  → bias_AX = promedio(ax_g) -  1.0
 *     promedio(ax_g) = 17400/16384 = 1.0620g
 *     bias_AX = 1.0620 - 1.0000 = +0.0620g  ← solo el offset del chip
 *
 *   chip_Y debería medir 0g:
 *     promedio(ay_g) = -550/16384 = -0.0336g
 *     bias_AY = -0.0336 - 0.0 = -0.0336g
 *
 *   chip_Z debería medir 0g:
 *     promedio(az_g) = -250/16384 = -0.0153g
 *     bias_AZ = -0.0153 - 0.0 = -0.0153g
 *
 * Ajuste residual acelerómetro 2026-06-23:
 *   captura multicara incompleta, ajuste robusto de esfera sobre telemetría
 *   post-calibración: body_bias ≈ (-0.0006, -0.0077, +0.0365) g.
 *   Se aplica solo como offset preliminar; una calibración final requiere las
 *   6 caras limpias para resolver bias y escala por eje.
 *
 * Para el giróscopo: en reposo debe medir 0°/s → bias = promedio directo.
 * Ajuste residual 2026-06-23: 506 muestras quietas filtradas desde telemetría
 * post-calibración; se compensa el promedio remanente en marco de cuerpo.    */
#define CALIB_BIAS_AX           (+0.061374f)  /* [g]   offset chip_X        */
#define CALIB_BIAS_AY           (-0.025854f)  /* [g]   offset chip_Y        */
#define CALIB_BIAS_AZ           (+0.021235f)  /* [g]   offset chip_Z        */
#define CALIB_BIAS_GX           (-1.367046f)  /* [°/s]                      */
#define CALIB_BIAS_GY           (-2.302856f)  /* [°/s]                      */
#define CALIB_BIAS_GZ           (+0.960243f)  /* [°/s]                      */

/* ── Estructura de datos RAW ─────────────────────────────────────────────────*/
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6050_RawData_t;

/* ── Estructura de datos calibrados ──────────────────────────────────────────
 * Marco de cuerpo usado por la GUI/eCompass:
 * X adelante, Y izquierda, Z arriba.                                          */
typedef struct
{
    float ax_w;
    float ay_w;
    float az_w;
    float gx_w;
    float gy_w;
    float gz_w;
    float temp_c;
    float accel_norm;
    float gyro_norm;
} MPU6050_CalibratedData_t;

/* ── Prototipos ──────────────────────────────────────────────────────────────*/
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *whoami);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *data);

/* ── Calibración inline ──────────────────────────────────────────────────────
 *
 * Pasos:
 *   1. LSB → unidades físicas [g] y [°/s]
 *   2. Restar bias de offset del chip (NO incluye componente de gravedad)
 *   3. Rotar ejes chip → marco de cuerpo
 *
 * Matriz de rotación para montaje actual con chip hacia abajo:
 *   X_body =  chip_X
 *   Y_body = -chip_Y
 *   Z_body =  chip_Z
 *
 * Con bias solo de offset, en reposo:
 *   chip_Z ≈ -1g con chip hacia abajo → az_w ≈ -1.000g
 *
 * Fusion recibe [0, 0, -1g] → gravedad apunta en -Z → correcto.
 * Fusion inicializa la orientación desde ahí sin conflicto con FusionBias.   */
static inline void MPU6050_ApplyCalibration(
    const MPU6050_RawData_t     *raw,
    MPU6050_CalibratedData_t    *cal)
{
    /* 1. LSB → física */
    float ax_g = (float)raw->ax / MPU6050_ACCEL_SCALE;
    float ay_g = (float)raw->ay / MPU6050_ACCEL_SCALE;
    float az_g = (float)raw->az / MPU6050_ACCEL_SCALE;
    float gx_d = (float)raw->gx / MPU6050_GYRO_SCALE;
    float gy_d = (float)raw->gy / MPU6050_GYRO_SCALE;
    float gz_d = (float)raw->gz / MPU6050_GYRO_SCALE;

    /* 2. Restar bias de offset (solo error del chip, no gravedad) */
    float ax_c = ax_g - CALIB_BIAS_AX;
    float ay_c = ay_g - CALIB_BIAS_AY;
    float az_c = az_g - CALIB_BIAS_AZ;
    float gx_c = gx_d - CALIB_BIAS_GX;
    float gy_c = gy_d - CALIB_BIAS_GY;
    float gz_c = gz_d - CALIB_BIAS_GZ;

    /* 3. Rotar al marco de cuerpo: chip hacia abajo, Z vertical real */
    cal->ax_w =  ax_c;   /* X_body =  chip_X */
    cal->ay_w = -ay_c;   /* Y_body = -chip_Y */
    cal->az_w =  az_c;   /* Z_body =  chip_Z */

    cal->gx_w =  gx_c;
    cal->gy_w = -gy_c;
    cal->gz_w =  gz_c;

    /* 4. Temperatura */
    cal->temp_c = (float)raw->temp / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;

    /* 5. Magnitudes */
    cal->accel_norm = sqrtf(cal->ax_w * cal->ax_w +
                             cal->ay_w * cal->ay_w +
                             cal->az_w * cal->az_w);
    cal->gyro_norm  = sqrtf(cal->gx_w * cal->gx_w +
                             cal->gy_w * cal->gy_w +
                             cal->gz_w * cal->gz_w);
}

#endif /* INC_MPU6050_H_ */
