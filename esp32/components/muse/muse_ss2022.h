/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "psa/crypto.h"

/* TCP subset of SIP022, one 16-byte PSK, no identity/UDP extensions. */
typedef struct {
    psa_key_id_t key;
    uint8_t nonce[12];
} ss2022_cipher_t;
typedef struct {
    ss2022_cipher_t cipher;
    uint8_t psk[16], request_salt[16];
    uint8_t wire[65551], plain[65535];
    size_t have, need;
    unsigned phase;
    bool failed;
} ss2022_rx_t;
typedef bool (*ss2022_emit_t)(void *, const uint8_t *, size_t);
bool ss2022_cipher_init(ss2022_cipher_t *, const uint8_t psk[16], const uint8_t salt[16]);
void ss2022_cipher_free(ss2022_cipher_t *);
bool ss2022_request(ss2022_cipher_t *, const uint8_t salt[16], const char *host,
                    uint16_t port, uint64_t now, const uint8_t *padding, size_t padding_len,
                    uint8_t *out, size_t cap, size_t *len);
bool ss2022_payload(ss2022_cipher_t *, const uint8_t *data, size_t len,
                    uint8_t *out, size_t cap, size_t *out_len);
void ss2022_rx_init(ss2022_rx_t *, const uint8_t psk[16], const uint8_t request_salt[16]);
void ss2022_rx_free(ss2022_rx_t *);
bool ss2022_feed(ss2022_rx_t *, const uint8_t *, size_t, uint64_t now, ss2022_emit_t, void *);
