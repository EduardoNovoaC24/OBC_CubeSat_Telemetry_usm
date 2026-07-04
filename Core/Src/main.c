/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Telemetria IMU/magnetometro con FreeRTOS y SX1262
  ******************************************************************************
  * Task_SensorFusion produce quaternion/eCompass y Task_RadioTest transmite
  * el ultimo payload LoRaWAN por SX1262. UART2 (USB, 115200) deja logs.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "radio_test.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "FusionAhrs.h"
#include "FusionBias.h"
#include "lorawan_crypto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

#define UART_TX_TIMEOUT_MS   100U
#define LORAWAN_FPORT        44U
#define LORAWAN_MAX_APP_LEN  51U
#define QUAT_UART_PERIOD_MS  20U     /* ~igual al periodo del lazo (vTaskDelay 20ms) → streaming fluido por cable */
#define LORAWAN_TX_PERIOD_MS 10000U
#define MAG_READ_PERIOD_MS   80U     /* HMC5883L a 15 Hz: muestra nueva cada ~67 ms */
#define MAG_DIAG_PERIOD_MS   500U
#define FUSION_USE_MAGNETOMETER 0U
#define FUSION_GAIN             0.50f
#define FUSION_SAMPLE_RATE_HZ   50.0f
#define FUSION_BIAS_STATIONARY_THRESHOLD_DPS 3.0f
#define FUSION_BIAS_STATIONARY_PERIOD_S      3.0f
#define ECOMPASS_ACCEL_DOWN_SIGN (-1.0f) /* Con chip hacia abajo, AZ plano debe quedar cerca de -1g. */
#define ECOMPASS_HEADING_SIGN     1.0f
#define ECOMPASS_HEADING_OFFSET_DEG 0.0f

TaskHandle_t radioTestTaskHandle;
TaskHandle_t sensorFusionTaskHandle;
static SemaphoreHandle_t uartMutexHandle;
static SemaphoreHandle_t telemetryMutexHandle;

typedef struct
{
    char payload[LORAWAN_MAX_APP_LEN + 1U];
    uint32_t updated_ms;
    bool valid;
} TelemetrySnapshot_t;

static TelemetrySnapshot_t telemetry_snapshot = {
    .payload = "QERR,NO_DATA",
    .updated_ms = 0U,
    .valid = false,
};

/* TTN ABP session for end-device stm32-obc-usm.
 * ABP uplinks use DevAddr + session keys; DevEUI is only registry identity. */
static const uint32_t LORAWAN_DEV_ADDR = 0x260D99FD;
static const uint8_t LORAWAN_NWK_SKEY[16] = {
    0x45, 0xEF, 0x6B, 0xB6, 0x4E, 0xCD, 0x79, 0x34,
    0xDF, 0x6D, 0x40, 0x16, 0x57, 0xB0, 0x8F, 0xCD
};
static const uint8_t LORAWAN_APP_SKEY[16] = {
    0x84, 0xE4, 0xEC, 0xD9, 0x06, 0x5F, 0x17, 0xA3,
    0xB0, 0xE2, 0x03, 0xF3, 0x23, 0x28, 0x35, 0xAC
};
static uint32_t lorawan_fcnt_up = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void Task_RadioTest(void *argument);
void Task_SensorFusion(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "STACK OVERFLOW: %s\r\n", pcTaskName);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    __disable_irq();
    while (1) {}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  char dbg[64];
  snprintf(dbg, sizeof(dbg), "PCLK1=%lu Hz\r\n", pclk1);
  HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), 100);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  uartMutexHandle = xSemaphoreCreateMutex();
  telemetryMutexHandle = xSemaphoreCreateMutex();
  if ((uartMutexHandle == NULL) || (telemetryMutexHandle == NULL)) {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (xTaskCreate(Task_RadioTest, "RADIOTEST", 1024, NULL, osPriorityNormal, &radioTestTaskHandle) != pdPASS) {
    Error_Handler();
  }
  if (xTaskCreate(Task_SensorFusion, "SENSOR", 1024, NULL, osPriorityAboveNormal, &sensorFusionTaskHandle) != pdPASS) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; /* ~1.25 MHz, validado para este cableado de protoboard */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SX1262_NRESET_GPIO_Port, SX1262_NRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  /* NSS arranca en alto (deseleccionado) — en bajo deja al SX1262
   * seleccionado antes de que el SPI esté listo para transaccionar. */
  HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SX1262_NRESET_Pin */
  GPIO_InitStruct.Pin = SX1262_NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SX1262_NRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1262_NSS_Pin */
  GPIO_InitStruct.Pin = SX1262_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SX1262_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1262_BUSY_Pin */
  /* Pull-down a propósito (diagnóstico): si el módulo está desconectado o
   * sin alimentación, BUSY flotante debe leerse en bajo de forma estable
   * en vez de ruido ambiguo — así "BUSY libre" no da falsa confianza. */
  GPIO_InitStruct.Pin = SX1262_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SX1262_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SX1262_DIO1_Pin */
  GPIO_InitStruct.Pin = SX1262_DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SX1262_DIO1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * Task_RadioTest — build de prueba mínima del driver SX1262 (Semtech
 * sx126x_driver vendorizado + HAL STM32 propio, ver Drivers/SX126x/).
 *
 * Secuencia: REGTEST -> XOSC -> telemetría de quaternion por LoRa cada 10 s.
 * REGTEST es diagnóstico: por defecto no detiene el flujo.
 */
static int16_t quat_to_q10000(float value)
{
    if (value > 1.0f) {
        value = 1.0f;
    } else if (value < -1.0f) {
        value = -1.0f;
    }

    const float scaled = value * 10000.0f;
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void format_quaternion_payload(const FusionQuaternion q, char *payload, size_t payload_len)
{
    snprintf(payload, payload_len, "Q,%+06d,%+06d,%+06d,%+06d",
             quat_to_q10000(q.element.w),
             quat_to_q10000(q.element.x),
             quat_to_q10000(q.element.y),
             quat_to_q10000(q.element.z));
}

static int16_t mag_to_payload_d10(float value_mg)
{
    float value = value_mg * 0.1f;
    if (value > 999.0f) {
        value = 999.0f;
    }
    if (value < -999.0f) {
        value = -999.0f;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static float wrap_heading_degrees(float heading)
{
    while (heading < 0.0f) {
        heading += 360.0f;
    }
    while (heading >= 360.0f) {
        heading -= 360.0f;
    }
    return heading;
}

static void compass_vector_from_heading(float heading_deg,
                                        HMC5883L_CalData_t *horizontal)
{
    if (horizontal == NULL) return;

    const float heading_rad = heading_deg * (3.14159265358979323846f / 180.0f);
    horizontal->x = 1000.0f * cosf(heading_rad);
    horizontal->y = 1000.0f * sinf(heading_rad);
    horizontal->z = 0.0f;
}

static bool calculate_ecompass_heading_an4248(const FusionVector accelerometer,
                                              const FusionVector magnetometer,
                                              float *heading_deg)
{
    if (heading_deg == NULL) {
        return false;
    }

    const float ax = accelerometer.axis.x;
    const float ay = accelerometer.axis.y;
    const float az = accelerometer.axis.z;
    const float accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
    if (accel_norm < 0.25f) {
        return false;
    }

    /* AN4248: acelerómetro y magnetómetro primero deben estar en el mismo
     * marco. Convertimos la lectura del acelerómetro en vector "down". */
    const float down_x = ECOMPASS_ACCEL_DOWN_SIGN * ax / accel_norm;
    const float down_y = ECOMPASS_ACCEL_DOWN_SIGN * ay / accel_norm;
    const float down_z = ECOMPASS_ACCEL_DOWN_SIGN * az / accel_norm;

    const float mx = magnetometer.axis.x;
    const float my = magnetometer.axis.y;
    const float mz = magnetometer.axis.z;
    const float vertical_mag = (mx * down_x) + (my * down_y) + (mz * down_z);

    const float north_x = mx - (vertical_mag * down_x);
    const float north_y = my - (vertical_mag * down_y);
    const float horizontal_norm = sqrtf((north_x * north_x) + (north_y * north_y));
    if (horizontal_norm < 1.0f) {
        return false;
    }

    *heading_deg = wrap_heading_degrees((ECOMPASS_HEADING_SIGN *
                                        atan2f(north_y, north_x) *
                                        (180.0f / 3.14159265358979323846f)) +
                                        ECOMPASS_HEADING_OFFSET_DEG);
    return true;
}

static void format_telemetry_payload(const FusionQuaternion q,
                                     const HMC5883L_CalData_t *mag,
                                     bool mag_ok,
                                     char *payload,
                                     size_t payload_len)
{
    if (mag_ok && (mag != NULL)) {
        snprintf(payload, payload_len,
                 "Q,%+06d,%+06d,%+06d,%+06d,M,%+04d,%+04d,%+04d,%u",
                 quat_to_q10000(q.element.w),
                 quat_to_q10000(q.element.x),
                 quat_to_q10000(q.element.y),
                 quat_to_q10000(q.element.z),
                 mag_to_payload_d10(mag->x),
                 mag_to_payload_d10(mag->y),
                 mag_to_payload_d10(mag->z),
                 1U);
    } else {
        format_quaternion_payload(q, payload, payload_len);
    }
}

static int16_t clamp_float_to_i16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint8_t build_lorawan_abp_uplink(const uint8_t *app_payload,
                                        uint8_t app_len,
                                        uint8_t fport,
                                        uint8_t *frame,
                                        uint8_t frame_max)
{
    if ((app_payload == NULL) || (frame == NULL) ||
        (app_len > LORAWAN_MAX_APP_LEN) || (frame_max < (uint8_t)(13U + app_len))) {
        return 0;
    }

    uint8_t idx = 0;
    frame[idx++] = 0x40; /* MHDR: Unconfirmed Data Up, LoRaWAN 1.0 */

    frame[idx++] = (uint8_t)(LORAWAN_DEV_ADDR & 0xFF);
    frame[idx++] = (uint8_t)((LORAWAN_DEV_ADDR >> 8) & 0xFF);
    frame[idx++] = (uint8_t)((LORAWAN_DEV_ADDR >> 16) & 0xFF);
    frame[idx++] = (uint8_t)((LORAWAN_DEV_ADDR >> 24) & 0xFF);

    frame[idx++] = 0x00; /* FCtrl: no ADR, no ACK, FOptsLen=0 */
    frame[idx++] = (uint8_t)(lorawan_fcnt_up & 0xFF);
    frame[idx++] = (uint8_t)((lorawan_fcnt_up >> 8) & 0xFF);
    frame[idx++] = fport;

    lorawan_encrypt_payload(app_payload, app_len, LORAWAN_DEV_ADDR,
                            lorawan_fcnt_up, LORAWAN_APP_SKEY, &frame[idx]);
    idx += app_len;

    uint8_t mic[4];
    lorawan_compute_mic(frame, idx, LORAWAN_DEV_ADDR, lorawan_fcnt_up,
                        LORAWAN_NWK_SKEY, mic);
    memcpy(&frame[idx], mic, sizeof(mic));
    idx += sizeof(mic);

    lorawan_fcnt_up++;
    return idx;
}

static void uart2_write(const char *msg)
{
    if (msg == NULL) {
        return;
    }

    if (uartMutexHandle != NULL) {
        xSemaphoreTake(uartMutexHandle, portMAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_TX_TIMEOUT_MS);
    if (uartMutexHandle != NULL) {
        xSemaphoreGive(uartMutexHandle);
    }
}

static void telemetry_publish(const char *payload)
{
    if (payload == NULL) {
        return;
    }

    xSemaphoreTake(telemetryMutexHandle, portMAX_DELAY);
    snprintf(telemetry_snapshot.payload, sizeof(telemetry_snapshot.payload), "%s", payload);
    telemetry_snapshot.updated_ms = HAL_GetTick();
    telemetry_snapshot.valid = true;
    xSemaphoreGive(telemetryMutexHandle);
}

static bool telemetry_copy(char *payload, size_t payload_len)
{
    bool valid;

    if ((payload == NULL) || (payload_len == 0U)) {
        return false;
    }

    xSemaphoreTake(telemetryMutexHandle, portMAX_DELAY);
    snprintf(payload, payload_len, "%s", telemetry_snapshot.payload);
    valid = telemetry_snapshot.valid;
    xSemaphoreGive(telemetryMutexHandle);

    return valid;
}

void Task_RadioTest(void *argument)
{
    char msg[192];

    vTaskDelay(pdMS_TO_TICKS(500));

    snprintf(msg, sizeof(msg), "[BUILD] radio_test %s %s\r\n", __DATE__, __TIME__);
    uart2_write(msg);

    radio_test_init(&hspi1);

    bool reg_ok = radio_register_test();
    snprintf(msg, sizeof(msg), "[REGTEST] resultado=%s\r\n", reg_ok ? "OK" : "FAIL");
    uart2_write(msg);

    if (!reg_ok && (RADIO_STOP_ON_REGTEST_FAIL != 0)) {
        const char *stop_msg = "[RADIOTEST] detenido: REGTEST fallo\r\n";
        uart2_write(stop_msg);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } else if (!reg_ok) {
        const char *cont_msg = "[RADIOTEST] REGTEST fallo, continuo con XOSC/TX\r\n";
        uart2_write(cont_msg);
    }

    bool xosc_ok = radio_xosc_test();
    snprintf(msg, sizeof(msg), "[XOSC] resultado=%s\r\n", xosc_ok ? "OK" : "FAIL");
    uart2_write(msg);

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LORAWAN_TX_PERIOD_MS));

        char payload[LORAWAN_MAX_APP_LEN + 1U];
        if (!telemetry_copy(payload, sizeof(payload))) {
            snprintf(payload, sizeof(payload), "QERR,NO_DATA");
        }

        uint8_t frame[64];
        const uint32_t fcnt_sent = lorawan_fcnt_up;
        uint8_t frame_len = build_lorawan_abp_uplink((const uint8_t*)payload,
                                                     (uint8_t)strlen(payload),
                                                     LORAWAN_FPORT,
                                                     frame,
                                                     sizeof(frame));
        bool tx_ok = false;
        if (frame_len > 0U) {
            tx_ok = radio_send_payload(frame, frame_len);
        }
        snprintf(msg, sizeof(msg), ">> TX LoRaWAN fcnt=%lu len=%u r=%s\r\n",
                 (unsigned long)fcnt_sent, frame_len, tx_ok ? "OK" : "FAIL");
        uart2_write(msg);
    }
}

void Task_SensorFusion(void *argument)
{
    char msg[192];

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSettings ahrs_settings = fusionAhrsDefaultSettings;
    ahrs_settings.gain = FUSION_GAIN;
    ahrs_settings.gyroscopeRange = 250.0f;
    /*
     * La fusión no usa magnetómetro para corregir orientación, por lo que el
     * acelerómetro es la única referencia absoluta de roll/pitch. 20° era
     * demasiado estricto: si el equipo arrancaba o se movía y quedaba con más
     * error, Fusion ignoraba el acelerómetro y el quaternion no podía volver.
     */
    ahrs_settings.accelerationRejection = 90.0f;
    ahrs_settings.magneticRejection = 20.0f;
    ahrs_settings.recoveryTriggerPeriod = 250U;
    FusionAhrsSetSettings(&ahrs, &ahrs_settings);

    FusionBias gyro_bias;
    FusionBiasInitialise(&gyro_bias);
    FusionBiasSettings bias_settings = fusionBiasDefaultSettings;
    bias_settings.sampleRate = FUSION_SAMPLE_RATE_HZ;
    bias_settings.stationaryThreshold = FUSION_BIAS_STATIONARY_THRESHOLD_DPS;
    bias_settings.stationaryPeriod = FUSION_BIAS_STATIONARY_PERIOD_S;
    FusionBiasSetSettings(&gyro_bias, &bias_settings);

    bool imu_ok = (MPU6050_Init(&hi2c1) == HAL_OK);
    snprintf(msg, sizeof(msg), "[IMU] MPU6050=%s\r\n", imu_ok ? "OK" : "FAIL");
    uart2_write(msg);

    uint8_t mag_id[3] = {0};
    HAL_StatusTypeDef mag_id_status = HMC5883L_ReadId(&hi2c1, mag_id);
    snprintf(msg, sizeof(msg), "[MAG] ID ret=%ld bytes=%02X %02X %02X\r\n",
             (long)mag_id_status, mag_id[0], mag_id[1], mag_id[2]);
    uart2_write(msg);

    bool mag_ok = (HMC5883L_Init(&hi2c1) == HAL_OK);
    snprintf(msg, sizeof(msg), "[MAG] HMC5883L=%s\r\n", mag_ok ? "OK" : "FAIL");
    uart2_write(msg);
    const char *ori_msg = "[ORI] chip-down body: X=chipX Y=-chipY Z=chipZ heading_sign=+1 offset=0\r\n";
    uart2_write(ori_msg);
    const char *mag_map_msg = "[MAGMAP] bodyX=+chipX bodyY=-chipY bodyZ=+chipZ\r\n";
    uart2_write(mag_map_msg);

    uint32_t last_update_ms = HAL_GetTick();
    uint32_t last_uart_ms = HAL_GetTick();
    uint32_t last_mag_ms = 0U;
    uint32_t last_mag_diag_ms = 0U;

    MPU6050_CalibratedData_t last_cal = {0};
    HMC5883L_RawData_t last_mag_raw = {0};
    HMC5883L_CalData_t last_mag_chip = {0};
    HMC5883L_CalData_t last_mag_cal = {0};
    HMC5883L_CalData_t last_mag_compass = {0};
    float last_heading_deg = 0.0f;
    bool imu_data_ok = imu_ok;
    bool mag_data_ok = false;
    bool last_ecompass_ok = false;
    uint8_t mag_fail_count = 0U;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        const uint32_t now_ms = HAL_GetTick();
        float delta_time = (float)(now_ms - last_update_ms) * 0.001f;
        last_update_ms = now_ms;
        if (delta_time <= 0.0f || delta_time > 0.1f) {
            delta_time = 0.02f;
        }

        imu_data_ok = false;
        mag_data_ok = false;

        if (imu_ok) {
            MPU6050_RawData_t raw;
            MPU6050_CalibratedData_t cal;
            if (MPU6050_ReadRaw(&hi2c1, &raw) == HAL_OK) {
                imu_data_ok = true;
                MPU6050_ApplyCalibration(&raw, &cal);
                last_cal = cal;

                const FusionVector gyroscope = {
                    .axis = { .x = cal.gx_w, .y = cal.gy_w, .z = cal.gz_w }
                };
                const FusionVector corrected_gyroscope = FusionBiasUpdate(&gyro_bias, gyroscope);
                const FusionVector accelerometer = {
                    .axis = { .x = cal.ax_w, .y = cal.ay_w, .z = cal.az_w }
                };

                if (mag_ok && ((now_ms - last_mag_ms) >= MAG_READ_PERIOD_MS)) {
                    last_mag_ms = now_ms;
                    HMC5883L_RawData_t mag_raw;
                    if (HMC5883L_ReadRaw(&hi2c1, &mag_raw) == HAL_OK) {
                        mag_data_ok = true;
                        mag_fail_count = 0U;
                        HMC5883L_CalData_t mag_chip = {0};
                        HMC5883L_CalData_t mag_world = {0};
                        HMC5883L_ApplyCalibration(&mag_raw, &mag_chip);
                        HMC5883L_RotateToBody(&mag_chip, &mag_world);
                        last_mag_raw = mag_raw;
                        last_mag_chip = mag_chip;
                        last_mag_cal = mag_world;
                        const FusionVector magnetometer = {
                            .axis = { .x = mag_world.x, .y = mag_world.y, .z = mag_world.z }
                        };
                        float heading = 0.0f;
                        if (calculate_ecompass_heading_an4248(accelerometer,
                                                              magnetometer,
                                                              &heading)) {
                            last_heading_deg = heading;
                            compass_vector_from_heading(heading, &last_mag_compass);
                            last_ecompass_ok = true;
                        } else {
                            last_mag_compass = (HMC5883L_CalData_t){0};
                            last_ecompass_ok = false;
                        }
                    } else {
                        if (mag_fail_count < 255U) {
                            mag_fail_count++;
                        }
                        if (mag_fail_count >= 5U) {
                            mag_ok = (HMC5883L_Init(&hi2c1) == HAL_OK);
                            mag_fail_count = 0U;
                            uart2_write("[MAG] reinit\r\n");
                        }
                    }
                }
                mag_data_ok = mag_ok &&
                              last_ecompass_ok &&
                              ((last_mag_raw.x != 0) || (last_mag_raw.y != 0) || (last_mag_raw.z != 0));

                if ((FUSION_USE_MAGNETOMETER != 0U) && mag_data_ok) {
                    const FusionVector magnetometer = {
                        .axis = { .x = last_mag_cal.x, .y = last_mag_cal.y, .z = last_mag_cal.z }
                    };
                    FusionAhrsUpdate(&ahrs, corrected_gyroscope, accelerometer, magnetometer, delta_time);
                } else {
                    FusionAhrsUpdateNoMagnetometer(&ahrs, corrected_gyroscope, accelerometer, delta_time);
                }
            } else {
                imu_ok = false;
                last_cal = (MPU6050_CalibratedData_t){0};
                last_mag_raw = (HMC5883L_RawData_t){0};
                last_mag_chip = (HMC5883L_CalData_t){0};
                last_mag_cal = (HMC5883L_CalData_t){0};
                last_mag_compass = (HMC5883L_CalData_t){0};
                last_ecompass_ok = false;
                FusionAhrsSetQuaternion(&ahrs, FUSION_QUATERNION_IDENTITY);
                uart2_write("[IMU] read FAIL\r\n");
            }
        }

        char payload[LORAWAN_MAX_APP_LEN + 1U];
        if (imu_data_ok) {
            format_telemetry_payload(FusionAhrsGetQuaternion(&ahrs),
                                     &last_mag_compass,
                                     mag_data_ok,
                                     payload,
                                     sizeof(payload));
        } else {
            snprintf(payload, sizeof(payload), "QERR,MPU6050");
        }
        telemetry_publish(payload);

        if ((HAL_GetTick() - last_uart_ms) >= QUAT_UART_PERIOD_MS) {
            last_uart_ms = HAL_GetTick();
            FusionQuaternion q = imu_data_ok ? FusionAhrsGetQuaternion(&ahrs) : FUSION_QUATERNION_IDENTITY;
            snprintf(msg, sizeof(msg),
                     "T=%lu|AX=%d AY=%d AZ=%d|GX=%d GY=%d GZ=%d|MX=%d MY=%d MZ=%d|Q=%d,%d,%d,%d|MAG=%s\r\n",
                     (unsigned long)HAL_GetTick(),
                     clamp_float_to_i16(last_cal.ax_w * 1000.0f),
                     clamp_float_to_i16(last_cal.ay_w * 1000.0f),
                     clamp_float_to_i16(last_cal.az_w * 1000.0f),
                     clamp_float_to_i16(last_cal.gx_w * 100.0f),
                     clamp_float_to_i16(last_cal.gy_w * 100.0f),
                     clamp_float_to_i16(last_cal.gz_w * 100.0f),
                     clamp_float_to_i16(last_mag_compass.x),
                     clamp_float_to_i16(last_mag_compass.y),
                     clamp_float_to_i16(last_mag_compass.z),
                     quat_to_q10000(q.element.w),
                     quat_to_q10000(q.element.x),
                     quat_to_q10000(q.element.y),
                     quat_to_q10000(q.element.z),
                     mag_data_ok ? "OK" : "ERR");
            uart2_write(msg);
        }

        if ((HAL_GetTick() - last_mag_diag_ms) >= MAG_DIAG_PERIOD_MS) {
            last_mag_diag_ms = HAL_GetTick();
            snprintf(msg, sizeof(msg),
                     "[MAGD] R=%d,%d,%d C=%d,%d,%d B=%d,%d,%d H=%d OK=%u\r\n",
                     last_mag_raw.x,
                     last_mag_raw.y,
                     last_mag_raw.z,
                     clamp_float_to_i16(last_mag_chip.x),
                     clamp_float_to_i16(last_mag_chip.y),
                     clamp_float_to_i16(last_mag_chip.z),
                     clamp_float_to_i16(last_mag_cal.x),
                     clamp_float_to_i16(last_mag_cal.y),
                     clamp_float_to_i16(last_mag_cal.z),
                     clamp_float_to_i16(last_heading_deg * 10.0f),
                     mag_data_ok ? 1U : 0U);
            uart2_write(msg);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
