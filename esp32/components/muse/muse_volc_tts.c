/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Doubao speech synthesis 2.0 over the V3 unidirectional HTTP API:
 *   POST https://openspeech.bytedance.com/api/v3/tts/unidirectional
 *   X-Api-Key, X-Api-Resource-Id: seed-tts-2.0, X-Api-Request-Id
 *   req_params.speaker zh_female_vv_uranus_bigtts (Vivi 2.0) by default
 *   audio_params format pcm, sample_rate 16000
 * The body comes back as NDJSON. code 0 lines carry base64 PCM; code 20000000
 * ends the stream. Playback is 16 kHz mono s16le, which muse_audio_write plays
 * after the voice task reads it from the reply stream.
 *
 * The key comes from main/volc_tts_secret.h (VOLC_TTS_API_KEY) when that
 * gitignored file exists, otherwise from CONFIG_MUSE_VOLC_TTS_API_KEY.
 * This request stays on the open network. openspeech.bytedance.com is a
 * China service and is not sent through the Shadowsocks bridge.
 */

#include "muse_volc_tts.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "muse_audio.h"
#include "muse_chat.h"

static const char *TAG = "volc_tts";

#define TTS_URL "https://openspeech.bytedance.com/api/v3/tts/unidirectional"
#define TTS_STACK (20 * 1024)
#define TTS_LINE_MAX 65536
#define TTS_PCM_MAX 49152
#define API_DONE 20000000

#if __has_include("volc_tts_secret.h")
#include "volc_tts_secret.h"
#endif

#ifndef CONFIG_MUSE_VOLC_TTS_API_KEY
#define CONFIG_MUSE_VOLC_TTS_API_KEY ""
#endif
#ifndef CONFIG_MUSE_VOLC_TTS_VOICE
#define CONFIG_MUSE_VOLC_TTS_VOICE ""
#endif
#ifndef CONFIG_MUSE_VOLC_TTS_RESOURCE_ID
#define CONFIG_MUSE_VOLC_TTS_RESOURCE_ID ""
#endif

static atomic_int s_busy;
static atomic_int s_result; /* 1 ok, -1 fail */
static atomic_int s_http;
static atomic_int s_api;
static atomic_uint s_frames;

static const char *api_key(void)
{
#ifdef VOLC_TTS_API_KEY
    if (VOLC_TTS_API_KEY[0]) {
        return VOLC_TTS_API_KEY;
    }
#endif
    return CONFIG_MUSE_VOLC_TTS_API_KEY;
}

static const char *voice_id(void)
{
    return CONFIG_MUSE_VOLC_TTS_VOICE[0] ? CONFIG_MUSE_VOLC_TTS_VOICE : "zh_female_vv_uranus_bigtts";
}

static const char *resource_id(void)
{
    return CONFIG_MUSE_VOLC_TTS_RESOURCE_ID[0] ? CONFIG_MUSE_VOLC_TTS_RESOURCE_ID : "seed-tts-2.0";
}

bool muse_volc_tts_enabled(void)
{
    const char *key = api_key();
    return key && key[0];
}

int muse_volc_tts_take_result(int *http_status, int *api_code, uint32_t *frames)
{
    int st = atomic_exchange(&s_result, 0);
    if (http_status) {
        *http_status = atomic_load(&s_http);
    }
    if (api_code) {
        *api_code = atomic_load(&s_api);
    }
    if (frames) {
        *frames = atomic_load(&s_frames);
    }
    return st;
}

static int b64_val(int c)
{
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    return -1;
}

/* Decodes into dst. Returns bytes written, or -1. */
static int b64_decode(const char *in, uint8_t *dst, size_t dst_cap)
{
    size_t n = 0;
    int val = 0, bits = 0;
    for (const char *p = in; *p; p++) {
        if (*p == '=' || *p == '\r' || *p == '\n' || *p == ' ') {
            continue;
        }
        int v = b64_val((unsigned char)*p);
        if (v < 0) {
            return -1;
        }
        val = (val << 6) | v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (n >= dst_cap) {
                return -1;
            }
            dst[n++] = (uint8_t)((val >> bits) & 0xff);
        }
    }
    return (int)n;
}

static void request_id(char out[37])
{
    uint32_t r[4];
    for (int i = 0; i < 4; i++) {
        r[i] = esp_random();
    }
    snprintf(out, 37, "%08lx-%04lx-%04lx-%04lx-%04lx%08lx",
             (unsigned long)r[0],
             (unsigned long)(r[1] >> 16), (unsigned long)(r[1] & 0xffff),
             (unsigned long)(r[2] >> 16), (unsigned long)(r[2] & 0xffff),
             (unsigned long)r[3]);
}

typedef struct {
    char *text;
    uint8_t *pcm;
    size_t pcm_len;
    uint32_t frames;
    int http;
    int api;
    bool failed;
} job_t;

static bool queue_pcm(job_t *job)
{
    size_t frames = job->pcm_len / sizeof(int16_t);
    if (!frames) {
        return true;
    }
    if (!muse_hatch_reply_pcm((const int16_t *)job->pcm, frames)) {
        return false;
    }
    size_t used = frames * sizeof(int16_t);
    job->pcm_len -= used;
    if (job->pcm_len) {
        memmove(job->pcm, job->pcm + used, job->pcm_len);
    }
    job->frames += (uint32_t)frames;
    return true;
}

static bool take_line(job_t *job, char *line)
{
    char *json = line;
    while (*json == ' ' || *json == '\t') {
        json++;
    }
    if (strncmp(json, "data:", 5) == 0) {
        json += 5;
        while (*json == ' ') {
            json++;
        }
    }
    cJSON *root = cJSON_Parse(json);
    if (!root) {
        return true; /* skip a non-JSON keepalive */
    }
    cJSON *code = cJSON_GetObjectItem(root, "code");
    int c = cJSON_IsNumber(code) ? code->valueint : -1;
    job->api = c;
    if (c == API_DONE) {
        cJSON_Delete(root);
        if (!queue_pcm(job)) {
            job->failed = true;
            return false;
        }
        return true;
    }
    if (c != 0) {
        const char *msg = cJSON_GetStringValue(cJSON_GetObjectItem(root, "message"));
        char brief[80];
        snprintf(brief, sizeof(brief), "%s", msg ? msg : "");
        ESP_LOGW(TAG, "api code %d %s", c, brief);
        job->failed = true;
        cJSON_Delete(root);
        return false;
    }
    const char *data = cJSON_GetStringValue(cJSON_GetObjectItem(root, "data"));
    if (data && data[0]) {
        int n = b64_decode(data, job->pcm + job->pcm_len, TTS_PCM_MAX - job->pcm_len);
        if (n < 0) {
            ESP_LOGW(TAG, "audio chunk did not fit");
            job->failed = true;
            cJSON_Delete(root);
            return false;
        }
        job->pcm_len += (size_t)n;
        if (job->pcm_len >= MUSE_AUDIO_CHUNK * sizeof(int16_t) && !queue_pcm(job)) {
            job->failed = true;
            cJSON_Delete(root);
            return false;
        }
    }
    cJSON_Delete(root);
    return true;
}

static void speak(job_t *job)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *req = cJSON_AddObjectToObject(root, "req_params");
    cJSON_AddStringToObject(req, "text", job->text);
    cJSON_AddStringToObject(req, "speaker", voice_id());
    cJSON *audio = cJSON_AddObjectToObject(req, "audio_params");
    cJSON_AddStringToObject(audio, "format", "pcm");
    cJSON_AddNumberToObject(audio, "sample_rate", MUSE_AUDIO_RATE);
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body) {
        job->failed = true;
        return;
    }

    char req_id[37];
    request_id(req_id);
    ESP_LOGI(TAG, "request %u chars voice %s resource %s id %s", (unsigned)strlen(job->text), voice_id(),
             resource_id(), req_id);

    esp_http_client_config_t cfg = {
        .url = TTS_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 20000,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGW(TAG, "http client init failed");
        free(body);
        job->failed = true;
        return;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-Api-Key", api_key());
    esp_http_client_set_header(client, "X-Api-Resource-Id", resource_id());
    esp_http_client_set_header(client, "X-Api-Request-Id", req_id);

    esp_err_t err = esp_http_client_open(client, strlen(body));
    int body_len = (int)strlen(body);
    int wrote = err == ESP_OK ? esp_http_client_write(client, body, body_len) : -1;
    free(body);
    if (err != ESP_OK || wrote != body_len) {
        ESP_LOGW(TAG, "http open/write failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        job->failed = true;
        return;
    }
    esp_http_client_fetch_headers(client);
    job->http = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP %d", job->http);
    if (job->http < 200 || job->http >= 300) {
        char snippet[100];
        int n = esp_http_client_read(client, snippet, (int)sizeof(snippet) - 1);
        if (n < 0) {
            n = 0;
        }
        snippet[n] = '\0';
        for (int i = 0; snippet[i]; i++) {
            if (snippet[i] == '\n' || snippet[i] == '\r') {
                snippet[i] = ' ';
            }
        }
        ESP_LOGW(TAG, "HTTP %d body: %s", job->http, snippet);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        job->failed = true;
        return;
    }

    char *line = heap_caps_malloc(TTS_LINE_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!line) {
        line = malloc(TTS_LINE_MAX);
    }
    if (!line) {
        ESP_LOGW(TAG, "no line buffer");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        job->failed = true;
        return;
    }
    size_t have = 0;
    char chunk[512];
    while (!job->failed) {
        int n = esp_http_client_read(client, chunk, sizeof(chunk));
        if (n < 0) {
            ESP_LOGW(TAG, "http read failed");
            job->failed = true;
            break;
        }
        if (n == 0) {
            break;
        }
        for (int i = 0; i < n && !job->failed; i++) {
            char ch = chunk[i];
            if (ch == '\n' || ch == '\r') {
                if (!have) {
                    continue;
                }
                line[have] = '\0';
                have = 0;
                if (!take_line(job, line)) {
                    break;
                }
                continue;
            }
            if (have + 1 >= TTS_LINE_MAX) {
                ESP_LOGW(TAG, "response line too long");
                job->failed = true;
                break;
            }
            line[have++] = ch;
        }
    }
    if (!job->failed && have) {
        line[have] = '\0';
        take_line(job, line);
    }
    if (!job->failed) {
        queue_pcm(job);
    }
    free(line);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    if (!job->failed && job->api != API_DONE && job->frames == 0) {
        ESP_LOGW(TAG, "stream ended without audio (api %d)", job->api);
        job->failed = true;
    }
}

static void tts_task(void *arg)
{
    job_t job = { .text = arg };
    job.pcm = heap_caps_malloc(TTS_PCM_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!job.pcm) {
        job.pcm = malloc(TTS_PCM_MAX);
    }
    if (!job.pcm) {
        ESP_LOGW(TAG, "no pcm buffer");
        job.failed = true;
    } else {
        speak(&job);
        free(job.pcm);
    }
    free(job.text);
    atomic_store(&s_http, job.http);
    atomic_store(&s_api, job.api);
    atomic_store(&s_frames, job.frames);
    if (!job.failed && job.frames) {
        ESP_LOGI(TAG, "played %u frames", (unsigned)job.frames);
        atomic_store(&s_result, 1);
    } else {
        ESP_LOGW(TAG, "failed http %d api %d frames %u", job.http, job.api, (unsigned)job.frames);
        atomic_store(&s_result, -1);
    }
    atomic_store(&s_busy, 0);
    vTaskDeleteWithCaps(NULL);
}

bool muse_volc_tts_submit(const char *text)
{
    if (!muse_volc_tts_enabled() || !text || !text[0]) {
        return false;
    }
    int idle = 0;
    if (!atomic_compare_exchange_strong(&s_busy, &idle, 1)) {
        ESP_LOGW(TAG, "busy");
        return false;
    }
    atomic_store(&s_result, 0);
    atomic_store(&s_http, 0);
    atomic_store(&s_api, 0);
    atomic_store(&s_frames, 0);
    size_t n = strlen(text);
    char *copy = heap_caps_malloc(n + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) {
        copy = malloc(n + 1);
    }
    if (!copy) {
        atomic_store(&s_busy, 0);
        return false;
    }
    memcpy(copy, text, n + 1);
    /* PSRAM stack: mbedTLS plus the HTTP client. The hatch task already does this. */
    if (xTaskCreateWithCaps(tts_task, "volc_tts", TTS_STACK, copy, 4, NULL,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGW(TAG, "task create failed");
        free(copy);
        atomic_store(&s_busy, 0);
        return false;
    }
    return true;
}
