/* SPDX-License-Identifier: Apache-2.0
 * Wire format: https://shadowsocks.org/doc/sip022.html
 * Cryptography uses upstream BLAKE3 and IDF's PSA AES-GCM implementation.
 */
#include "muse_ss2022.h"
#include "third_party/blake3/blake3.h"
#include <string.h>

static void wipe(void *p, size_t n) { volatile uint8_t *b = p; while (n--) *b++ = 0; }
static void put16(uint8_t *p, size_t n) { p[0] = n >> 8; p[1] = n; }
static size_t get16(const uint8_t *p) { return ((size_t)p[0] << 8) | p[1]; }
static void put64(uint8_t *p, uint64_t n) { for (int i = 7; i >= 0; i--, n >>= 8) p[i] = n; }
static uint64_t get64(const uint8_t *p) { uint64_t n = 0; for (int i = 0; i < 8; i++) n = (n << 8) | p[i]; return n; }
static bool next_nonce(ss2022_cipher_t *c) {
    for (unsigned i = 0; i < sizeof(c->nonce); i++) if (++c->nonce[i]) return true;
    return false;
}

bool ss2022_cipher_init(ss2022_cipher_t *c, const uint8_t psk[16], const uint8_t salt[16]) {
    memset(c, 0, sizeof(*c));
    if (psa_crypto_init() != PSA_SUCCESS) return false;
    uint8_t material[32], key[16];
    memcpy(material, psk, 16); memcpy(material + 16, salt, 16);
    blake3_hasher h;
    blake3_hasher_init_derive_key(&h, "shadowsocks 2022 session subkey");
    blake3_hasher_update(&h, material, sizeof(material));
    blake3_hasher_finalize(&h, key, sizeof(key));
    psa_key_attributes_t a = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&a, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&a, 128);
    psa_set_key_usage_flags(&a, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&a, PSA_ALG_GCM);
    bool ok = psa_import_key(&a, key, sizeof(key), &c->key) == PSA_SUCCESS;
    psa_reset_key_attributes(&a);
    wipe(key, sizeof(key)); wipe(material, sizeof(material)); wipe(&h, sizeof(h));
    return ok;
}
void ss2022_cipher_free(ss2022_cipher_t *c) {
    if (c->key) psa_destroy_key(c->key);
    wipe(c, sizeof(*c));
}
static bool seal(ss2022_cipher_t *c, const uint8_t *p, size_t n, uint8_t *out, size_t cap) {
    size_t written = 0;
    return psa_aead_encrypt(c->key, PSA_ALG_GCM, c->nonce, 12, NULL, 0, p, n,
                            out, cap, &written) == PSA_SUCCESS && written == n + 16 && next_nonce(c);
}
static bool open_chunk(ss2022_cipher_t *c, const uint8_t *p, size_t n, uint8_t *out, size_t cap) {
    size_t written = 0;
    if (psa_aead_decrypt(c->key, PSA_ALG_GCM, c->nonce, 12, NULL, 0, p, n,
                         out, cap, &written) != PSA_SUCCESS || written + 16 != n) {
        wipe(out, cap); return false;
    }
    return next_nonce(c);
}
bool ss2022_request(ss2022_cipher_t *c, const uint8_t salt[16], const char *host,
                    uint16_t port, uint64_t now, const uint8_t *padding, size_t padding_len,
                    uint8_t *out, size_t cap, size_t *len) {
    size_t hn = strlen(host), vn = hn + 6 + padding_len;
    if (!hn || hn > 255 || !port || !padding_len || padding_len > 900 || cap < 16 + 27 + vn + 16) return false;
    uint8_t fixed[11], variable[1161];
    fixed[0] = 0; put64(fixed + 1, now); put16(fixed + 9, vn);
    variable[0] = 3; variable[1] = hn; memcpy(variable + 2, host, hn);
    put16(variable + 2 + hn, port); put16(variable + 4 + hn, padding_len);
    memcpy(variable + 6 + hn, padding, padding_len);
    memcpy(out, salt, 16);
    bool ok = seal(c, fixed, 11, out + 16, cap - 16) && seal(c, variable, vn, out + 43, cap - 43);
    *len = 59 + vn;
    return ok;
}
bool ss2022_payload(ss2022_cipher_t *c, const uint8_t *p, size_t n, uint8_t *out, size_t cap, size_t *len) {
    if (!n || n > 65535 || cap < n + 34) return false;
    uint8_t size[2]; put16(size, n);
    *len = n + 34;
    return seal(c, size, 2, out, cap) && seal(c, p, n, out + 18, cap - 18);
}
void ss2022_rx_init(ss2022_rx_t *r, const uint8_t psk[16], const uint8_t salt[16]) {
    memset(r, 0, sizeof(*r)); memcpy(r->psk, psk, 16); memcpy(r->request_salt, salt, 16);
    r->need = 59; /* server salt + encrypted type/time/request-salt/length */
}
void ss2022_rx_free(ss2022_rx_t *r) { ss2022_cipher_free(&r->cipher); wipe(r, sizeof(*r)); }
bool ss2022_feed(ss2022_rx_t *r, const uint8_t *p, size_t n, uint64_t now, ss2022_emit_t emit, void *arg) {
    if (r->failed) return false;
    while (n) {
        size_t take = r->need - r->have; if (take > n) take = n;
        memcpy(r->wire + r->have, p, take); r->have += take; p += take; n -= take;
        if (r->have < r->need) continue;
        if (r->phase == 0) {
            if (!ss2022_cipher_init(&r->cipher, r->psk, r->wire) ||
                !open_chunk(&r->cipher, r->wire + 16, 43, r->plain, sizeof(r->plain))) goto fail;
            uint64_t stamp = get64(r->plain + 1), diff = stamp > now ? stamp - now : now - stamp;
            if (r->plain[0] != 1 || diff > 30 || memcmp(r->plain + 9, r->request_salt, 16)) goto fail;
            r->need = get16(r->plain + 25) + 16; r->phase = 1;
            wipe(r->psk, sizeof(r->psk));
        } else if (r->phase == 2) {
            if (!open_chunk(&r->cipher, r->wire, 18, r->plain, sizeof(r->plain))) goto fail;
            r->need = get16(r->plain) + 16; r->phase = 1;
        } else {
            size_t body = r->need - 16;
            if (!open_chunk(&r->cipher, r->wire, r->need, r->plain, sizeof(r->plain)) ||
                (body && !emit(arg, r->plain, body))) goto fail;
            r->need = 18; r->phase = 2;
        }
        r->have = 0;
    }
    return true;
fail:
    r->failed = true; wipe(r->plain, sizeof(r->plain)); return false;
}
