#include "radio_test.h"
#include "sx126x.h"
#include "sx126x_hal_stm32.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

static void log_msg(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

static void radio_pin_probe(const char *tag)
{
    char dbg[160];
    snprintf(dbg, sizeof(dbg),
             "[PIN] %s NSS_ODR=%u NSS_IDR=%u RESET_ODR=%u BUSY=%u DIO1=%u GPIOA_ODR=0x%04lX GPIOA_IDR=0x%04lX\r\n",
             tag,
             (unsigned int)((SX1262_NSS_GPIO_Port->ODR & SX1262_NSS_Pin) != 0U),
             (unsigned int)((SX1262_NSS_GPIO_Port->IDR & SX1262_NSS_Pin) != 0U),
             (unsigned int)((SX1262_NRESET_GPIO_Port->ODR & SX1262_NRESET_Pin) != 0U),
             (unsigned int)HAL_GPIO_ReadPin(SX1262_BUSY_GPIO_Port, SX1262_BUSY_Pin),
             (unsigned int)HAL_GPIO_ReadPin(SX1262_DIO1_GPIO_Port, SX1262_DIO1_Pin),
             (unsigned long)GPIOA->ODR,
             (unsigned long)GPIOA->IDR);
    log_msg(dbg);
}

static bool read_register_with_retries(uint16_t addr, uint8_t expected, uint8_t *got)
{
    char dbg[120];

    for (uint32_t attempt = 1; attempt <= RADIO_REGTEST_READ_RETRIES; attempt++) {
        if (sx126x_read_register(&sx126x_ctx, addr, got, 1) != SX126X_STATUS_OK) {
            snprintf(dbg, sizeof(dbg),
                     "[REGTEST] retry=%lu reg=0x%04X read SPI error\r\n",
                     (unsigned long)attempt, addr);
            log_msg(dbg);
        } else if (*got == expected) {
            if (attempt > 1U) {
                snprintf(dbg, sizeof(dbg),
                         "[REGTEST] recovered reg=0x%04X intento=%lu val=0x%02X\r\n",
                         addr, (unsigned long)attempt, *got);
                log_msg(dbg);
            }
            return true;
        } else {
            snprintf(dbg, sizeof(dbg),
                     "[REGTEST] retry=%lu reg=0x%04X esperado=0x%02X leido=0x%02X\r\n",
                     (unsigned long)attempt, addr, expected, *got);
            log_msg(dbg);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return false;
}

void radio_test_init(SPI_HandleTypeDef *hspi)
{
    sx126x_hal_stm32_attach(hspi);
    HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_SET);
    radio_pin_probe("before_reset");
    sx126x_reset(&sx126x_ctx);
    radio_pin_probe("after_reset");
}

/* ── radio_spi_diag ───────────────────────────────────────────────────────
 * Diagnóstico de una sola pasada para separar "SPI roto en general" de
 * "solo fallan los comandos con dirección de 16 bits": GetStatus (opcode
 * solo, sin dirección) + GetDeviceErrors (opcode + dummy, sin dirección) +
 * trace byte-a-byte de un write/read de registro con eco crudo de MISO.
 * Si GetStatus/GetDeviceErrors dan datos válidos pero el trace muestra
 * MISO en 0x00/0xFF durante el byte de dato, el problema está en la
 * electrónica/cableado de esa línea, no en el protocolo. */
static void radio_spi_diag(void)
{
    char dbg[160];

    sx126x_chip_status_t status = { 0 };
    sx126x_errors_mask_t errors = 0;
    sx126x_get_status(&sx126x_ctx, &status);
    sx126x_get_device_errors(&sx126x_ctx, &errors);
    snprintf(dbg, sizeof(dbg),
             "[DIAG] GetStatus (sin addr): chip_mode=%u cmd_status=%u | GetDeviceErrors OpError=0x%04X\r\n",
             status.chip_mode, status.cmd_status, errors);
    log_msg(dbg);

    SPI_HandleTypeDef *hspi = sx126x_ctx.hspi;
    uint8_t tx[4], rx[4];
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(SX1262_BUSY_GPIO_Port, SX1262_BUSY_Pin) == GPIO_PIN_SET &&
           (HAL_GetTick() - t0) < 1000U) { }

    /* WriteRegister 0x0740=0x34, eco crudo de MISO durante la transacción */
    HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_RESET);
    tx[0] = 0x0D; tx[1] = 0x07; tx[2] = 0x40; tx[3] = 0x34;
    HAL_SPI_TransmitReceive(hspi, tx, rx, 4, 100);
    HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_SET);
    snprintf(dbg, sizeof(dbg),
             "[DIAG] WRITE 0x0740 tx=[%02X %02X %02X %02X] rx=[%02X %02X %02X %02X]\r\n",
             tx[0], tx[1], tx[2], tx[3], rx[0], rx[1], rx[2], rx[3]);
    log_msg(dbg);

    t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(SX1262_BUSY_GPIO_Port, SX1262_BUSY_Pin) == GPIO_PIN_SET &&
           (HAL_GetTick() - t0) < 1000U) { }

    /* ReadRegister 0x0740, eco crudo de MISO */
    HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_RESET);
    uint8_t tx5[5] = { 0x1D, 0x07, 0x40, 0x00, 0x00 };
    uint8_t rx5[5] = { 0 };
    HAL_SPI_TransmitReceive(hspi, tx5, rx5, 5, 100);
    HAL_GPIO_WritePin(SX1262_NSS_GPIO_Port, SX1262_NSS_Pin, GPIO_PIN_SET);
    snprintf(dbg, sizeof(dbg),
             "[DIAG] READ  0x0740 tx=[%02X %02X %02X %02X %02X] rx=[%02X %02X %02X %02X %02X] val=0x%02X\r\n",
             tx5[0], tx5[1], tx5[2], tx5[3], tx5[4],
             rx5[0], rx5[1], rx5[2], rx5[3], rx5[4], rx5[4]);
    log_msg(dbg);
}

/* ── radio_register_test ─────────────────────────────────────────────────
 * Escribe/lee 100 veces los 4 registros que el driver viejo no lograba
 * persistir (sync word LoRaWAN público + errata TX_CLAMP + OCP). Acceso
 * directo write_register/read_register, sin pasar por cfg_tx_clamp()/
 * set_ocp_value() — el objetivo es probar la persistencia cruda de SPI,
 * no la semántica de alto nivel. Tiene reintentos de lectura porque el
 * protoboard/cableado ha mostrado lecturas espurias aisladas. */
bool radio_register_test(void)
{
    static const uint16_t addrs[4]    = { 0x0740, 0x0741, 0x08D8, 0x08E7 };
    static const uint8_t  expected[4] = { 0x34,   0x44,   0xFE,   0x38   };
    char dbg[96];

    radio_spi_diag();

    snprintf(dbg, sizeof(dbg), "[REGTEST] Inicio %lu iteraciones, read_retries=%lu\r\n",
             (unsigned long)RADIO_REGTEST_ITERATIONS,
             (unsigned long)RADIO_REGTEST_READ_RETRIES);
    log_msg(dbg);

    for (uint32_t i = 0; i < RADIO_REGTEST_ITERATIONS; i++) {
        for (uint32_t r = 0; r < 4U; r++) {
            if (sx126x_write_register(&sx126x_ctx, addrs[r], &expected[r], 1) != SX126X_STATUS_OK) {
                snprintf(dbg, sizeof(dbg), "[REGTEST] FAIL iter=%lu reg=0x%04X write SPI error\r\n",
                         (unsigned long)i, addrs[r]);
                log_msg(dbg);
                return false;
            }
        }

        for (uint32_t r = 0; r < 4U; r++) {
            uint8_t got = 0;
            if (!read_register_with_retries(addrs[r], expected[r], &got)) {
                snprintf(dbg, sizeof(dbg), "[REGTEST] FAIL iter=%lu reg=0x%04X esperado=0x%02X ultimo=0x%02X\r\n",
                         (unsigned long)i, addrs[r], expected[r], got);
                log_msg(dbg);
                return false;
            }
        }

        if ((i % 10U) == 9U) {
            snprintf(dbg, sizeof(dbg), "[REGTEST] OK iter=%lu\r\n", (unsigned long)(i + 1U));
            log_msg(dbg);
        }
    }

    snprintf(dbg, sizeof(dbg), "[REGTEST] OK %lu/%lu\r\n",
             (unsigned long)RADIO_REGTEST_ITERATIONS,
             (unsigned long)RADIO_REGTEST_ITERATIONS);
    log_msg(dbg);
    return true;
}

/* ── radio_xosc_test ──────────────────────────────────────────────────────
 * Un solo intento: STBY_RC -> DIO3=TCXO 3.3V (timeout HW 100ms) -> STBY_XOSC
 * -> verifica chip_mode y OpError. Reintenta con reset si XOSC no arranca. */
bool radio_xosc_test(void)
{
    char dbg[128];

    for (uint32_t attempt = 1; attempt <= RADIO_XOSC_ATTEMPTS; attempt++) {
        snprintf(dbg, sizeof(dbg), "[XOSC] intento=%lu/%lu\r\n",
                 (unsigned long)attempt, (unsigned long)RADIO_XOSC_ATTEMPTS);
        log_msg(dbg);

        sx126x_clear_device_errors(&sx126x_ctx);

        if (sx126x_set_standby(&sx126x_ctx, SX126X_STANDBY_CFG_RC) != SX126X_STATUS_OK) {
            log_msg("[XOSC] FAIL set_standby(RC) SPI error\r\n");
        } else if (sx126x_set_dio3_as_tcxo_ctrl(&sx126x_ctx, SX126X_TCXO_CTRL_3_3V, 6400) != SX126X_STATUS_OK) {
            log_msg("[XOSC] FAIL set_dio3_as_tcxo_ctrl SPI error\r\n");
        } else if (sx126x_set_standby(&sx126x_ctx, SX126X_STANDBY_CFG_XOSC) != SX126X_STATUS_OK) {
            log_msg("[XOSC] FAIL set_standby(XOSC) SPI error\r\n");
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));

            sx126x_chip_status_t status = { 0 };
            sx126x_errors_mask_t errors = 0;
            sx126x_get_status(&sx126x_ctx, &status);
            sx126x_get_device_errors(&sx126x_ctx, &errors);

            snprintf(dbg, sizeof(dbg), "[XOSC] chip_mode=%u cmd_status=%u OpError=0x%04X\r\n",
                     status.chip_mode, status.cmd_status, errors);
            log_msg(dbg);

            bool ok = (status.chip_mode == SX126X_CHIP_MODE_STBY_XOSC) &&
                      ((errors & SX126X_ERRORS_XOSC_START) == 0U);
            sx126x_clear_device_errors(&sx126x_ctx);
            if (ok) {
                log_msg("[XOSC] OK\r\n");
                return true;
            }
        }

        radio_pin_probe("xosc_fail");
        if (attempt < RADIO_XOSC_ATTEMPTS) {
            log_msg("[XOSC] reset y reintento\r\n");
            sx126x_reset(&sx126x_ctx);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    log_msg("[XOSC] FAIL\r\n");
    return false;
}

/* ── radio_send_payload ─────────────────────────────────────────────────────
 * LoRa plano: SF7/BW125/CR4_5, CRC ON, header explícito, IQ normal,
 * syncword publico 0x3444. */
bool radio_send_payload(const uint8_t *payload, uint8_t payload_len)
{
    char dbg[96];

    if ((payload == NULL) || (payload_len == 0U)) {
        log_msg("[TX] FAIL payload vacio\r\n");
        return false;
    }

    if (sx126x_set_standby(&sx126x_ctx, SX126X_STANDBY_CFG_XOSC) != SX126X_STATUS_OK) {
        log_msg("[TX] FAIL set_standby(XOSC)\r\n");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    sx126x_set_pkt_type(&sx126x_ctx, SX126X_PKT_TYPE_LORA);
    sx126x_set_rf_freq(&sx126x_ctx, RADIO_FREQ_HZ);

    sx126x_mod_params_lora_t mod = {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5,
        .ldro = 0,
    };
    sx126x_set_lora_mod_params(&sx126x_ctx, &mod);

    sx126x_pkt_params_lora_t pkt = {
        .preamble_len_in_symb = 8,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = payload_len,
        .crc_is_on = true,
        .invert_iq_is_on = false,
    };
    sx126x_set_lora_pkt_params(&sx126x_ctx, &pkt);

    sx126x_set_lora_sync_word(&sx126x_ctx, 0x34); /* codifica el patron publico 0x3444 */
    sx126x_set_buffer_base_address(&sx126x_ctx, 0, 0);

    sx126x_pa_cfg_params_t pa_cfg = { .pa_duty_cycle = 0x02, .hp_max = 0x02, .device_sel = 0x00, .pa_lut = 0x01 };
    sx126x_set_pa_cfg(&sx126x_ctx, &pa_cfg);
    sx126x_cfg_tx_clamp(&sx126x_ctx);          /* errata TX_CLAMP_CFG */
    sx126x_set_ocp_value(&sx126x_ctx, 0x38);   /* 160mA aprox */
    sx126x_set_tx_params(&sx126x_ctx, RADIO_TX_POWER_DBM, SX126X_RAMP_200_US);

    sx126x_set_dio_irq_params(&sx126x_ctx,
                               SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                               SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                               SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    sx126x_clear_irq_status(&sx126x_ctx, SX126X_IRQ_ALL);
    sx126x_write_buffer(&sx126x_ctx, 0, payload, payload_len);

    snprintf(dbg, sizeof(dbg), "[TX] payload_len=%u\r\n", payload_len);
    log_msg(dbg);

    if (sx126x_set_tx(&sx126x_ctx, 5000) != SX126X_STATUS_OK) {
        log_msg("[TX] FAIL set_tx SPI error\r\n");
        return false;
    }

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 5000U) {
        sx126x_irq_mask_t irq = 0;
        sx126x_get_irq_status(&sx126x_ctx, &irq);

        if (irq & SX126X_IRQ_TX_DONE) {
            sx126x_clear_irq_status(&sx126x_ctx, SX126X_IRQ_ALL);
            log_msg("[TX] TX_DONE\r\n");
            return true;
        }
        if (irq & SX126X_IRQ_TIMEOUT) {
            sx126x_clear_irq_status(&sx126x_ctx, SX126X_IRQ_ALL);
            log_msg("[TX] IRQ_TIMEOUT del chip\r\n");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    snprintf(dbg, sizeof(dbg), "[TX] timeout SW tras %lums sin TX_DONE\r\n", 5000UL);
    log_msg(dbg);
    return false;
}

bool radio_send_plain_hello(void)
{
    static const uint8_t payload[] = "banana";
    return radio_send_payload(payload, (uint8_t)(sizeof(payload) - 1U));
}
