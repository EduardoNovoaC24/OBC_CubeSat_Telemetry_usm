#ifndef RADIO_TEST_H
#define RADIO_TEST_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

/* AU915 FSB2 canal 8 (916.8 MHz) / EU868 (868.1 MHz). Cambiar RADIO_FREQ_HZ
 * a RADIO_FREQ_HZ_EU868 para operar en EU868. */
#define RADIO_FREQ_HZ_EU868   868100000UL
#define RADIO_FREQ_HZ_AU915   916800000UL
#ifndef RADIO_FREQ_HZ
#define RADIO_FREQ_HZ         RADIO_FREQ_HZ_AU915
#endif

#define RADIO_TX_POWER_DBM    14

#ifndef RADIO_REGTEST_ITERATIONS
#define RADIO_REGTEST_ITERATIONS       100U
#endif

#ifndef RADIO_REGTEST_READ_RETRIES
#define RADIO_REGTEST_READ_RETRIES     3U
#endif

#ifndef RADIO_XOSC_ATTEMPTS
#define RADIO_XOSC_ATTEMPTS            3U
#endif

#ifndef RADIO_STOP_ON_REGTEST_FAIL
#define RADIO_STOP_ON_REGTEST_FAIL     0
#endif

void radio_test_init(SPI_HandleTypeDef *hspi);

bool radio_register_test(void);
bool radio_xosc_test(void);
bool radio_send_payload(const uint8_t *payload, uint8_t payload_len);
bool radio_send_plain_hello(void);

#endif /* RADIO_TEST_H */
