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
 * Dual-mic speech front end (esp-sr AFE_TYPE_SR).
 * Input is "MM": the two ES7210 mics, no playback reference. Noise
 * suppression loads nsnet from the "model" partition; speech enhancement
 * separates the two mics. AGC stays quiet. AEC, VAD and WakeNet stay off.
 * The feed task and the output ring live in PSRAM. muse_audio_read() still
 * takes 20 ms chunks.
 */

#include "muse_afe.h"

#include <string.h>

#include "esp_afe_config.h"
#include "esp_afe_sr_models.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "model_path.h"

#include "muse_audio.h"
#include "muse_mem.h"

static const char *TAG = "muse_afe";

#define AFE_INPUT "MM"
#define RING_FRAMES (MUSE_AUDIO_RATE / 2) /* 0.5 s of mono */
#define FEED_STACK (8 * 1024)
#define READ_WAIT_MS 1000

static const esp_afe_sr_iface_t *s_iface;
static esp_afe_sr_data_t *s_data;
static srmodel_list_t *s_models;
static afe_config_t *s_config;
static TaskHandle_t s_task;
static int16_t *s_ring;
static size_t s_ring_r, s_ring_w, s_ring_n;
static int s_feed_frames;
static int s_feed_channels;
static int16_t *s_feed;
static SemaphoreHandle_t s_mu;
static StaticSemaphore_t s_mu_mem;
static volatile bool s_run;
static volatile bool s_hold;
static volatile bool s_reset;
static bool s_gave_up;

static void ring_clear(void)
{
    s_ring_r = s_ring_w = s_ring_n = 0;
}

static void ring_push(const int16_t *src, size_t frames)
{
    for (size_t i = 0; i < frames; i++) {
        s_ring[s_ring_w] = src[i];
        s_ring_w = (s_ring_w + 1) % RING_FRAMES;
        if (s_ring_n < RING_FRAMES) {
            s_ring_n++;
        } else {
            s_ring_r = (s_ring_r + 1) % RING_FRAMES;
        }
    }
}

static size_t ring_pop(int16_t *dst, size_t frames)
{
    size_t n = frames < s_ring_n ? frames : s_ring_n;
    for (size_t i = 0; i < n; i++) {
        dst[i] = s_ring[s_ring_r];
        s_ring_r = (s_ring_r + 1) % RING_FRAMES;
    }
    s_ring_n -= n;
    return n;
}

static void log_memory(const char *when)
{
    ESP_LOGI(TAG, "%s: internal free %u (min %u), PSRAM free %u (min %u)", when,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
}

static void feed_task(void *arg)
{
    (void)arg;
    while (s_run) {
        if (s_hold) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (s_reset) {
            s_reset = false;
            xSemaphoreTake(s_mu, portMAX_DELAY);
            ring_clear();
            xSemaphoreGive(s_mu);
            if (s_iface->reset_buffer) {
                s_iface->reset_buffer(s_data);
            }
        }
        if (muse_audio_read_interleaved(s_feed, (size_t)s_feed_frames) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        s_iface->feed(s_data, s_feed);
        afe_fetch_result_t *got = s_iface->fetch_with_delay(s_data, pdMS_TO_TICKS(40));
        if (!got || got->ret_value != ESP_OK || !got->data || got->data_size < (int)sizeof(int16_t)) {
            continue;
        }
        xSemaphoreTake(s_mu, portMAX_DELAY);
        ring_push(got->data, (size_t)got->data_size / sizeof(int16_t));
        xSemaphoreGive(s_mu);
    }
    s_task = NULL;
    vTaskDeleteWithCaps(NULL);
}

static void tear_down(void)
{
    s_run = false;
    s_hold = false;
    for (int i = 0; s_task && i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_task) {
        vTaskDeleteWithCaps(s_task);
        s_task = NULL;
    }
    if (s_iface && s_data) {
        s_iface->destroy(s_data);
    }
    s_data = NULL;
    s_iface = NULL;
    if (s_config) {
        afe_config_free(s_config);
        s_config = NULL;
    }
    if (s_models) {
        esp_srmodel_deinit(s_models);
        s_models = NULL;
    }
    free(s_feed);
    s_feed = NULL;
    free(s_ring);
    s_ring = NULL;
    ring_clear();
}

void muse_afe_stop(void)
{
    if (s_mu) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
    }
    s_gave_up = false;
    if (s_mu) {
        xSemaphoreGive(s_mu);
    }
    tear_down();
}

bool muse_afe_running(void)
{
    return s_data != NULL && s_run;
}

void muse_afe_hold(bool hold)
{
    s_hold = hold;
    if (!hold) {
        s_reset = true;
    }
}

static esp_err_t bring_up(void)
{
    log_memory("before AFE init");
    s_models = esp_srmodel_init("model");
    if (!s_models) {
        ESP_LOGW(TAG, "model partition missing or empty");
        return ESP_FAIL;
    }
    s_config = afe_config_init(AFE_INPUT, s_models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!s_config) {
        ESP_LOGW(TAG, "afe_config_init failed");
        return ESP_FAIL;
    }
    s_config->aec_init = false;
    s_config->se_init = true;
    s_config->ns_init = true;
    s_config->vad_init = false;
    s_config->wakenet_init = false;
    s_config->agc_init = true;
    s_config->agc_mode = AFE_AGC_MODE_WEBRTC;
    s_config->agc_compression_gain_db = 3; /* default 9; keep the peaks down */
    s_config->agc_target_level_dbfs = 9;   /* -9 dBFS, quieter than the default -3 */
    s_config->afe_linear_gain = 1.0f;
    s_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    s_config->afe_ringbuf_size = 8;
    s_config->afe_ns_mode = AFE_NS_MODE_NET;
    char *ns = esp_srmodel_filter(s_models, "nsnet", NULL);
    if (ns) {
        s_config->ns_model_name = ns;
    } else {
        ESP_LOGW(TAG, "no nsnet model in the partition");
        return ESP_FAIL;
    }
    afe_config_check(s_config);
    /* SE on two mics can clear NS. Both were requested; put NS back. */
    if (!s_config->ns_init) {
        ESP_LOGW(TAG, "config check cleared NS; restoring it");
        s_config->ns_init = true;
        s_config->ns_model_name = ns;
        s_config->afe_ns_mode = AFE_NS_MODE_NET;
    }
    s_config->se_init = true;
    s_config->aec_init = false;
    s_config->vad_init = false;
    s_config->wakenet_init = false;

    s_iface = esp_afe_handle_from_config(s_config);
    if (!s_iface) {
        ESP_LOGW(TAG, "no AFE interface");
        return ESP_FAIL;
    }
    s_data = s_iface->create_from_config(s_config);
    if (!s_data) {
        ESP_LOGW(TAG, "AFE create failed");
        return ESP_FAIL;
    }
    s_feed_frames = s_iface->get_feed_chunksize(s_data);
    s_feed_channels = s_iface->get_feed_channel_num(s_data);
    if (s_feed_frames <= 0 || s_feed_channels <= 0) {
        ESP_LOGW(TAG, "AFE chunk size invalid");
        return ESP_FAIL;
    }
    s_feed = heap_caps_malloc((size_t)s_feed_frames * s_feed_channels * sizeof(int16_t), MUSE_BIG_CAPS);
    s_ring = heap_caps_malloc(RING_FRAMES * sizeof(int16_t), MUSE_BIG_CAPS);
    if (!s_feed || !s_ring) {
        ESP_LOGW(TAG, "no PSRAM for the AFE ring");
        return ESP_FAIL;
    }
    ring_clear();
    s_run = true;
    s_hold = false;
    s_reset = false;
    if (xTaskCreateWithCaps(feed_task, "muse_afe", FEED_STACK, NULL, 5, &s_task,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGW(TAG, "feed task create failed");
        s_run = false;
        return ESP_FAIL;
    }
    if (s_iface->print_pipeline) {
        s_iface->print_pipeline(s_data);
    }
    log_memory("after AFE init");
    ESP_LOGI(TAG, "AFE up: %s, feed %d x %d ch, fetch chunk from the ring", AFE_INPUT, s_feed_frames,
             s_feed_channels);
    return ESP_OK;
}

esp_err_t muse_afe_start(void)
{
    if (s_gave_up) {
        return ESP_FAIL;
    }
    if (!s_mu) {
        s_mu = xSemaphoreCreateMutexStatic(&s_mu_mem);
    }
    if (!s_mu) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_mu, portMAX_DELAY);
    if (s_gave_up) {
        xSemaphoreGive(s_mu);
        return ESP_FAIL;
    }
    if (s_data) {
        xSemaphoreGive(s_mu);
        return ESP_OK;
    }
    esp_err_t err = bring_up();
    if (err != ESP_OK) {
        s_gave_up = true;
    }
    xSemaphoreGive(s_mu);
    if (err != ESP_OK) {
        tear_down();
        ESP_LOGW(TAG, "AFE init failed, mic mix + high-pass stays");
        log_memory("after AFE fallback");
    }
    return err;
}

esp_err_t muse_afe_read(int16_t *mono, size_t frames)
{
    if (!mono || !s_data) {
        return ESP_FAIL;
    }
    int waited = 0;
    while (frames) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
        size_t n = ring_pop(mono, frames);
        xSemaphoreGive(s_mu);
        mono += n;
        frames -= n;
        if (!frames) {
            break;
        }
        if (waited >= READ_WAIT_MS) {
            ESP_LOGW(TAG, "AFE read timed out");
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
    return ESP_OK;
}
