# OBC_MPU6050_FreeRTOS_BT

Firmware para un nodo de telemetría embebido (OBC simplificado) del CubeSat USM, basado en
STM32L476RGT6 (Nucleo-L476RG) y FreeRTOS (CMSIS-RTOSv2). Adquiere orientación desde una IMU
MPU6050 y un magnetómetro GY-271 (HMC5883L), la funde con el algoritmo AHRS de Madgwick
(librería [Fusion](https://github.com/xioTechnologies/Fusion) de xioTechnologies) y transmite
el cuaternión resultante por LoRaWAN (modo ABP) hacia The Things Network (TTN), usando un
transceptor SX1262.

## Arquitectura de firmware

El firmware corre dos tareas FreeRTOS de aplicación (más la tarea `defaultTask` idle que crea
CMSIS-RTOSv2 por defecto). No se usan colas: la comunicación entre tareas es un *mailbox* de
último valor protegido por mutex.

| Tarea | Prioridad | Periodo | Responsabilidad |
|---|---|---|---|
| `defaultTask` | Normal | Idle (`osDelay(1)`) | Tarea por defecto de CMSIS-RTOSv2, sin lógica de aplicación |
| `Task_SensorFusion` | AboveNormal | 20 ms (~50 Hz) | Lee MPU6050 + HMC5883L, calibra, corrige bias del giroscopio (`FusionBias`), funde con `FusionAhrs` (Madgwick), calcula heading (AN4248) y publica el snapshot de telemetría |
| `Task_RadioTest` | Normal | 10000 ms (una vez al inicio: diagnóstico SPI/XOSC) | Lee el snapshot, arma el uplink LoRaWAN (ABP) y transmite por el SX1262 |

Sincronización:
- `telemetryMutexHandle` protege `telemetry_snapshot` (escrito por `Task_SensorFusion`, leído por `Task_RadioTest`).
- `uartMutexHandle` serializa el acceso a UART2, usada por ambas tareas para logging.

El pin DIO1 del SX1262 está configurado como interrupción externa (EXTI1) a nivel de GPIO,
pero **no hay un `HAL_GPIO_EXTI_Callback` implementado**: el fin de transmisión se detecta
actualmente por *polling* del registro de IRQ vía SPI (`radio_send_payload()` en `radio_test.c`),
no por la interrupción DIO1.

## Sensores

| Sensor | Bus / dirección | Notas |
|---|---|---|
| MPU6050 (GY-521) | I2C1, `0x68` | Acelerómetro + giroscopio triaxial. Calibración de bias estático (`mpu6050.h`) + corrección dinámica de bias del giroscopio en runtime (`FusionBias`). |
| HMC5883L (GY-271) | I2C1, `0x1E` | Magnetómetro. `HMC5883L_ReadId()` valida los registros 0x0A/0x0B/0x0C (`'H'`,`'4'`,`'3'`) para confirmar que el chip es un HMC5883L genuino y no un clon QMC5883L antes de configurar el modo continuo. |

Ambos sensores comparten el bus I2C1 (`PB8`=SCL, `PB9`=SDA).

## Radio / LoRaWAN

- Transceptor: SX1262 (Semtech), driver vendorizado en `Drivers/SX126x/` + HAL SPI propio.
- Protocolo: LoRaWAN 1.0, activación **ABP** (DevAddr + NwkSKey/AppSKey embebidos en firmware),
  FPort 44, cifrado AES-128 y MIC (CMAC) implementados desde cero en `lorawan_crypto.c`.
- Parámetros de radio (`radio_test.h`): AU915 FSB2 canal 8 (916.8 MHz), SF7, BW 125 kHz, CR 4/5,
  preámbulo de 8 símbolos, potencia TX 14 dBm, *sync word* público `0x34`.
- Payload de aplicación: texto ASCII con el cuaternión escalado ×10000
  (`"Q,+w,+x,+y,+z"`, opcionalmente extendido con el vector de heading del magnetómetro).
- Servidor de red: [The Things Network](https://www.thethingsnetwork.org/) (TTN).

## Mapa de pines

| Periférico | Pines | Función |
|---|---|---|
| I2C1 | PB8 / PB9 | SCL / SDA — MPU6050 + HMC5883L |
| SPI1 | PA5 / PA6 / PA7 | SCK / MISO / MOSI — SX1262 |
| SX1262 NSS | PA4 | Chip select (software) |
| SX1262 NRESET | PC0 | Reset |
| SX1262 BUSY | PB0 | Entrada, pull-down |
| SX1262 DIO1 | PB1 | EXTI1 (configurada, sin callback registrado) |
| USART1 | PA9 / PA10 | TX / RX, 9600 baud — reservado para HC-06 (no usado en el firmware actual) |
| USART2 | PA2 / PA3 | TX / RX, 115200 baud — log de depuración por USB (ST-LINK VCP) |

## Estructura del proyecto

```
Core/
  Inc/        Headers de la aplicación (main.h, mpu6050.h, hmc5883l.h, Fusion*.h, lorawan_crypto.h, radio_test.h)
  Src/        main.c (tareas de aplicación), mpu6050.c, hmc5883l.c, Fusion*.c, lorawan_crypto.c, radio_test.c
  Startup/    Arranque del Cortex-M4 (startup_stm32l476xx.s)
Drivers/
  CMSIS/                  Soporte de core ARM y device STM32L4xx
  STM32L4xx_HAL_Driver/   HAL de ST
  SX126x/                 Driver vendorizado del transceptor SX1262
  Radiolib/               Referencia adicional de radio
Middlewares/Third_Party/FreeRTOS/   FreeRTOS + CMSIS-RTOSv2
OBC_MPU6050_FreeRTOS_BT.ioc          Configuración de STM32CubeMX
STM32L476RGTX_FLASH.ld / _RAM.ld     Scripts de enlazado (flash / RAM)
```

## Compilar y flashear

Proyecto de STM32CubeIDE (contiene `.project` / `.cproject`).

1. Abrir STM32CubeIDE → `File > Open Projects from File System...` → seleccionar esta carpeta.
2. Compilar con `Project > Build Project` (configuración `Debug`, genera el ejecutable en `Debug/`).
3. Flashear con un ST-LINK sobre la Nucleo-L476RG (`Run > Debug` o `Run > Run`).
4. Abrir un puerto serie sobre el VCP del ST-LINK a **115200 baud, 8N1** para ver el log de
   `Task_SensorFusion` / `Task_RadioTest` por UART2.

El archivo `.ioc` puede reabrirse en STM32CubeMX si se necesita regenerar la configuración de
periféricos (los bloques `USER CODE` de `main.c` se preservan).

## Estado conocido / limitaciones

- El fin de transmisión LoRaWAN se detecta por *polling* SPI, no por la interrupción DIO1
  (configurada pero sin callback).
- La fusión AHRS opera actualmente sin magnetómetro (`FusionAhrsUpdateNoMagnetometer`); el
  magnetómetro solo alimenta el cálculo de heading (AN4248), no corrige el cuaternión.
- El módulo `FusionCompass` de la librería Fusion está presente pero no se usa.
- UART1 (pensado para un módulo HC-06) está inicializada pero sin tráfico de telemetría en esta
  versión del firmware.
- El payload LoRaWAN es texto ASCII; migrar a un formato binario reduciría el tiempo en aire.
