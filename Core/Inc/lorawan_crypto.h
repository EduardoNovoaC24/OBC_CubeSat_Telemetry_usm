#ifndef __LORAWAN_CRYPTO_H__
#define __LORAWAN_CRYPTO_H__

#include <stdint.h>
#include <stddef.h>

/* AES-128 ECB block encryption (single 16-byte block) */
void aes128_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

/*
 * Calcula el MIC (Message Integrity Code) de 4 bytes para un frame LoRaWAN
 * usando AES-CMAC con la NwkSKey.
 *   msg     : buffer del frame (MHDR..FRMPayload, sin MIC)
 *   len     : longitud de msg
 *   dev_addr: DevAddr (uint32)
 *   fcnt    : frame counter uplink
 *   nwkskey : clave de red (16 bytes)
 *   mic     : salida de 4 bytes
 */
void lorawan_compute_mic(const uint8_t *msg, size_t len,
                         uint32_t dev_addr, uint32_t fcnt,
                         const uint8_t nwkskey[16], uint8_t mic[4]);

/*
 * Cifra/descifra el FRMPayload con AppSKey (el algoritmo LoRaWAN es simétrico).
 *   in      : payload en claro
 *   len     : longitud
 *   dev_addr: DevAddr
 *   fcnt    : frame counter uplink
 *   appskey : clave de aplicación (16 bytes)
 *   out     : salida cifrada (puede ser el mismo buffer que in)
 */
void lorawan_encrypt_payload(const uint8_t *in, size_t len,
                             uint32_t dev_addr, uint32_t fcnt,
                             const uint8_t appskey[16], uint8_t *out);

#endif /* __LORAWAN_CRYPTO_H__ */
