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
 * Waveshare ESP32-S3-Touch-AMOLED-1.75C: round 466 px CO5300 AMOLED with
 * CST9217 touch, ES8311 speaker + ES7210 dual mic, AXP2101 PMU. The top
 * button (PWR) is wired to the PMU and, through an inverter, to GPIO3; the
 * bottom one is BOOT (GPIO0).
 *
 * The ESP32-S3-Touch-AMOLED-1.75 (CONFIG_MUSE_BOARD_WAVESHARE_S3_175) runs
 * this driver too. Its BSP moves the panel and touch resets to GPIO 39 and 40
 * and MCLK to 42. GPIO 1 to 3 go to its SD slot, so PWR is read from the PMU's
 * key latch, and BOOT talks: held long, PWR makes the PMU cut power.
 */
#include <math.h>
#include <stdio.h>

#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "bsp/touch.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_lcd_bands.h"
#include "muse_mem.h"
#include "muse_pmu.h"

static const char *TAG = "board";

#define DRAW_BUF_LINES 118      /* four bands to the screen (muse_lcd_bands.h) */
#define LCD_TE GPIO_NUM_13      /* the panel's tearing effect, on both boards */
#define LCD_CHUNK_BYTES (BSP_LCD_H_RES * 8 * 2)
#if CONFIG_MUSE_BOARD_WAVESHARE_S3_175
#define PMU_KEY_EVERY 2         /* poll the PMU over I2C every 20 ms */
#else
#define PWR_GPIO GPIO_NUM_3    /* high while PWR is held (a BSS138 inverts it) */
#endif

static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_touch_handle_t s_tp;
static muse_gpio_button_t s_boot;
#if !CONFIG_MUSE_BOARD_WAVESHARE_S3_175
static muse_gpio_button_t s_pwr;
#endif

static esp_err_t init(void)
{
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");
    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_boot, GPIO_NUM_0), TAG, "boot button");
#if CONFIG_MUSE_BOARD_WAVESHARE_S3_175
    /* Only the PMU sees PWR: latch its edges for poll_buttons(). */
    esp_err_t err = muse_pmu_init(bsp_i2c_get_handle(), true);
#else
    ESP_RETURN_ON_ERROR(muse_gpio_button_init_high(&s_pwr, PWR_GPIO), TAG, "pwr button");
    /* PWR turned the board on and may still be held; don't count that as a press. */
    s_pwr.pressed = gpio_get_level(PWR_GPIO) == 1;
    /* GPIO3 gives the key, so the PMU needn't latch it. Its IRQ line isn't
     * wired to the ESP32. */
    esp_err_t err = muse_pmu_init(bsp_i2c_get_handle(), false);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PMU unavailable (%s): battery status disabled", esp_err_to_name(err));
        return ESP_OK;
    }
#if !CONFIG_MUSE_BOARD_WAVESHARE_S3_175
    /* PWR talks: the 10 s hold-off back on, whatever a reset in a held talk
     * left (a 16 s shutdown cleared then stays off, the datasheet default). */
    (void)muse_pmu_set_key_shutdown(true);   /* logs if it can't */
#endif
    /* Only DCDC1 (VCC3V3, including the NS4150B amp) and ALDO1 (A3V3, the
     * codecs' analog supply) feed anything; the schematic leaves the rest
     * unconnected. xiaozhi and Waveshare's AXP2101 example set both to 3.3 V
     * and then turn the others off. keep_rails() only clears enables, so a
     * PMU that left ALDO1 off (or at its default voltage) stays that way and
     * the DAC has no analog supply. */
    err = muse_pmu_set_dcdc1_aldo1_3v3();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "codec rails not at 3.3 V (%s)", esp_err_to_name(err));
    }
    err = muse_pmu_keep_rails(BIT(0), BIT(0));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "unused rails left on (%s)", esp_err_to_name(err));
    }
    return ESP_OK;
}

/* The CO5300 needs even-aligned update windows. */
static void round_area(lv_event_t *e)
{
    lv_area_t *a = lv_event_get_param(e);
    a->x1 &= ~1;
    a->y1 &= ~1;
    a->x2 |= 1;
    a->y2 |= 1;
}

/*
 * bsp_display_start(), less its draw buffers: the BSP's are PSRAM, and every
 * flush from them needs a fresh 46 KB internal DMA bounce buffer, which can't
 * be had once Wi-Fi and BLE are up. The bands go out through two fixed 7 KB
 * internal buffers instead; 9 KB ones left 1 KB free while Wi-Fi joined.
 */
static lv_display_t *display_start(lv_indev_t **touch)
{
    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }

    esp_lcd_panel_handle_t panel;
    const bsp_display_config_t panel_cfg = {
        .max_transfer_sz = LCD_CHUNK_BYTES,
    };
    if (bsp_display_new(&panel_cfg, &panel, &s_io) != ESP_OK) {
        return NULL;
    }
    muse_lcd_bands_set_gap(6, 0);   /* the BSP's panel gap, for video's direct writes */
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = panel,
        .panel_io = s_io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = BSP_LCD_H_RES,
            .ver_res = BSP_LCD_V_RES,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = muse_lcd_bands_register(disp_cfg, DRAW_BUF_LINES, LCD_CHUNK_BYTES);
    if (!disp) {
        return NULL;
    }
    lv_display_add_event_cb(disp, round_area, LV_EVENT_INVALIDATE_AREA, NULL);
    /* Tearing effect on (V-blank only), as the 1.75C's BSP already asks. */
    esp_lcd_panel_io_tx_param(s_io, (0x02 << 24) | (0x35 << 8), (uint8_t[]){ 0x00 }, 1);
    muse_lcd_bands_set_te(LCD_TE);

    const bsp_display_cfg_t touch_cfg = {
        .touch_flags = { .mirror_x = 1, .mirror_y = 1 },
    };
    if (bsp_touch_new(&touch_cfg, &s_tp) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_touch_config_t tp_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, s_tp);
    *touch = esp_lv_adapter_register_touch(&tp_cfg);
    if (!*touch || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void send_brightness(void *level)
{
    /* CO5300 "write display brightness" (0x51), as the BSP sends it. */
    esp_lcd_panel_io_tx_param(s_io, (0x02 << 24) | (0x51 << 8), level, 1);
}

static void set_brightness(int pct)
{
    uint8_t level = (uint8_t)(pct * 255 / 100);
    muse_lcd_bands_run(send_brightness, &level);
}

static void send_sleep(void *sleep)
{
    esp_lcd_panel_io_tx_param(s_io, (0x02 << 24) | ((*(bool *)sleep ? 0x10 : 0x11) << 8), NULL, 0);
}

/* Plain SLPIN/SLPOUT over the QSPI command path. The driver's own sleep also
 * enters deep standby, whose wake pulses the reset line shared with touch. */
static void panel_sleep(bool sleep)
{
    muse_lcd_bands_run(send_sleep, &sleep);
    vTaskDelay(pdMS_TO_TICKS(120));   /* settle before the next command */
}

/*
 * Screen off: LVGL stops, and the CST9217 goes from scanning to deep sleep
 * (command 0xD105), where it only answers its reset line. That line is touch
 * only; the panel's reset is GPIO 1.
 */
static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
        esp_lcd_panel_io_tx_param(s_tp->io, 0xD1, (uint8_t[]){ 0x05 }, 1);
    } else {
        gpio_set_level(BSP_LCD_TOUCH_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(BSP_LCD_TOUCH_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(50));   /* as the driver waits after its reset */
        esp_lv_adapter_resume();
    }
}

/* NS4150B CTRL is GPIO46, active high (Waveshare's ES8311 sketch and
 * xiaozhi's AUDIO_CODEC_PA_PIN, pa_reverted false). INPUT_OUTPUT so the
 * read-back is the pad, not the unread output latch. */
static bool s_pa_reset;

static void pa_drive(bool high, const char *when)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << BSP_POWER_AMP_IO,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_hold_dis(BSP_POWER_AMP_IO);
    if (!s_pa_reset) {
        gpio_reset_pin(BSP_POWER_AMP_IO);
        s_pa_reset = true;
    }
    gpio_config(&io);
    gpio_sleep_sel_dis(BSP_POWER_AMP_IO);
    int before = gpio_get_level(BSP_POWER_AMP_IO);
    gpio_set_level(BSP_POWER_AMP_IO, high ? 1 : 0);
    int after = gpio_get_level(BSP_POWER_AMP_IO);
    ESP_LOGI(TAG, "PA enable GPIO%d %s %s (before %d, read %d)", (int)BSP_POWER_AMP_IO, when,
             high ? "high" : "low", before, after);
}

static void audio_power(bool on)
{
    pa_drive(on, on ? "on" : "off");
}

/* esp_codec_dev_open() rewrites a full-duplex port to 16-bit slots (32 BCLK
 * per frame). The ES8311 coefficient for 16 kHz and MCLK x256 programs a
 * BCLK divider of 4, which is 64 BCLK per frame: 16-bit samples in 32-bit
 * slots, the format xiaozhi's BoxAudioCodec settles on once its ES7210 TDM
 * RX shares the clock. Muse writes both slots with the same sample, so the
 * mask stays both rather than xiaozhi's mono left. */
typedef struct {
    audio_codec_data_if_t base;
    const audio_codec_data_if_t *inner;
    i2s_chan_handle_t tx;
    i2s_chan_handle_t rx;
    bool tx_on;
    bool rx_on;
} wide_slot_t;

static wide_slot_t s_i2s;

static void force_wide_slots(void)
{
    i2s_channel_disable(s_i2s.tx);
    i2s_channel_disable(s_i2s.rx);
    i2s_std_slot_config_t slot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                                     I2S_SLOT_MODE_STEREO);
    slot.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    slot.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    slot.ws_width = 32;
    slot.slot_mask = I2S_STD_SLOT_BOTH;
    esp_err_t tx = i2s_channel_reconfig_std_slot(s_i2s.tx, &slot);
    esp_err_t rx = i2s_channel_reconfig_std_slot(s_i2s.rx, &slot);
    if (tx != ESP_OK || rx != ESP_OK) {
        ESP_LOGW(TAG, "I2S 16-in-32 slots: tx %s rx %s", esp_err_to_name(tx), esp_err_to_name(rx));
    }
}

static int slot_set_fmt(const audio_codec_data_if_t *h, esp_codec_dev_type_t type, esp_codec_dev_sample_info_t *fs)
{
    const wide_slot_t *s = (const wide_slot_t *)h;
    return s->inner->set_fmt(s->inner, type, fs);
}

static int slot_enable(const audio_codec_data_if_t *h, esp_codec_dev_type_t type, bool on)
{
    wide_slot_t *s = (wide_slot_t *)h;
    if (on) {
        /* set_fmt just left 16-bit slots. Widen before the enable, and put
         * back a direction that was already running (the mic opens second). */
        force_wide_slots();
    }
    int ret = s->inner->enable(s->inner, type, on);
    if (type & ESP_CODEC_DEV_TYPE_OUT) {
        s->tx_on = on;
    }
    if (type & ESP_CODEC_DEV_TYPE_IN) {
        s->rx_on = on;
    }
    if (on && s->tx_on && !(type & ESP_CODEC_DEV_TYPE_OUT)) {
        i2s_channel_enable(s->tx);
    }
    if (on && s->rx_on && !(type & ESP_CODEC_DEV_TYPE_IN)) {
        i2s_channel_enable(s->rx);
    }
    if (on) {
        ESP_LOGI(TAG, "I2S slots 16-bit data in 32-bit frames");
    }
    return ret;
}

static int slot_open(const audio_codec_data_if_t *h, void *cfg, int size)
{
    const wide_slot_t *s = (const wide_slot_t *)h;
    return s->inner->open(s->inner, cfg, size);
}

static bool slot_is_open(const audio_codec_data_if_t *h)
{
    const wide_slot_t *s = (const wide_slot_t *)h;
    return s->inner->is_open(s->inner);
}

static int slot_read(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    const wide_slot_t *s = (const wide_slot_t *)h;
    return s->inner->read(s->inner, data, size);
}

static int slot_write(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    const wide_slot_t *s = (const wide_slot_t *)h;
    return s->inner->write(s->inner, data, size);
}

static int slot_close(const audio_codec_data_if_t *h)
{
    const wide_slot_t *s = (const wide_slot_t *)h;
    return s->inner->close(s->inner);
}

static esp_codec_dev_handle_t s_out;

static void dump_es8311(esp_codec_dev_handle_t spk, const char *when)
{
    char line[180];
    int n = snprintf(line, sizeof(line), "ES8311 %s 00-45:", when);
    for (int reg = 0; reg <= 0x45 && n < (int)sizeof(line) - 3; reg++) {
        int val = 0;
        if (esp_codec_dev_read_reg(spk, reg, &val) != ESP_CODEC_DEV_OK) {
            n += snprintf(line + n, sizeof(line) - n, "??");
        } else {
            n += snprintf(line + n, sizeof(line) - n, "%02x", val & 0xff);
        }
    }
    ESP_LOGI(TAG, "%s", line);
    int fd = -1, fe = -1, ff = -1;
    esp_codec_dev_read_reg(spk, 0xFD, &fd);
    esp_codec_dev_read_reg(spk, 0xFE, &fe);
    esp_codec_dev_read_reg(spk, 0xFF, &ff);
    ESP_LOGI(TAG, "ES8311 id FD=%02x FE=%02x FF=%02x", fd & 0xff, fe & 0xff, ff & 0xff);
}

/* The registers xiaozhi's ES8311 enable leaves for a slave, MCLK-fed DAC:
 * clock on, 16-bit I2S, analog and DAC and HP drive up, unmute, volume. */
static void es8311_fix(esp_codec_dev_handle_t spk)
{
    int mute = 0;
    esp_codec_dev_read_reg(spk, 0x31, &mute);
    esp_codec_dev_write_reg(spk, 0x00, 0x80);
    esp_codec_dev_write_reg(spk, 0x01, 0x3F);
    esp_codec_dev_write_reg(spk, 0x09, 0x0C);
    esp_codec_dev_write_reg(spk, 0x0D, 0x01);
    esp_codec_dev_write_reg(spk, 0x12, 0x00);
    esp_codec_dev_write_reg(spk, 0x13, 0x10);
    esp_codec_dev_write_reg(spk, 0x31, mute & ~0x60);
    esp_codec_dev_write_reg(spk, 0x32, 0xC0);
    esp_codec_dev_write_reg(spk, 0x37, 0x08);
    int r00 = 0, r01 = 0, r09 = 0, r0d = 0, r12 = 0, r13 = 0, r31 = 0, r32 = 0, r37 = 0;
    esp_codec_dev_read_reg(spk, 0x00, &r00);
    esp_codec_dev_read_reg(spk, 0x01, &r01);
    esp_codec_dev_read_reg(spk, 0x09, &r09);
    esp_codec_dev_read_reg(spk, 0x0D, &r0d);
    esp_codec_dev_read_reg(spk, 0x12, &r12);
    esp_codec_dev_read_reg(spk, 0x13, &r13);
    esp_codec_dev_read_reg(spk, 0x31, &r31);
    esp_codec_dev_read_reg(spk, 0x32, &r32);
    esp_codec_dev_read_reg(spk, 0x37, &r37);
    ESP_LOGI(TAG, "ES8311 now 00=%02x 01=%02x 09=%02x 0d=%02x 12=%02x 13=%02x 31=%02x 32=%02x 37=%02x",
             r00 & 0xff, r01 & 0xff, r09 & 0xff, r0d & 0xff, r12 & 0xff, r13 & 0xff, r31 & 0xff, r32 & 0xff,
             r37 & 0xff);
}

static void speaker_probe(esp_codec_dev_handle_t spk, const char *when)
{
    pa_drive(true, when);
    if (spk) {
        dump_es8311(spk, when);
        es8311_fix(spk);
    }
}

static void boot_tone(void)
{
    enum { N = 64 };
    int16_t buf[N * 2];
    ESP_LOGI(TAG, "test tone start");
    speaker_probe(s_out, "tone");
    for (int n = 0; n < MUSE_AUDIO_RATE; n += N) {
        for (int i = 0; i < N; i++) {
            float ph = 2.0f * 3.14159265f * 1000.0f * (float)(n + i) / (float)MUSE_AUDIO_RATE;
            int16_t s = (int16_t)(sinf(ph) * 12000.0f);
            buf[2 * i] = buf[2 * i + 1] = s;
        }
        esp_codec_dev_write(s_out, buf, sizeof(buf));
    }
    ESP_LOGI(TAG, "test tone end");
}

static esp_err_t es8311_reset(void)
{
    /* Same 0x1F hold as Es8311AudioCodec::ResetCodec, before the codec
     * driver owns 7-bit address 0x18. */
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x18,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(bsp_i2c_get_handle(), &cfg, &dev);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t buf[2] = { 0x00, 0x1F };
    err = i2c_master_transmit(dev, buf, sizeof(buf), 50);
    i2c_master_bus_rm_device(dev);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(5));
        ESP_LOGI(TAG, "ES8311 reset 00=1f");
    }
    return err;
}

static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    /* Own the bus instead of bsp_audio_codec_speaker_init(): that leaves the
     * port mono at 22050 Hz, and a later open cannot widen the slots while
     * the other direction is running. Pins are the BSP's (1.75C MCLK 16, the
     * 1.75 MCLK 42; BCLK 9, WS 45, DOUT 8, DIN 10). ES8311 is slave, DAC
     * only, MCLK on, pa_pin 46, hw_gain 5.0/3.3, as BoxAudioCodec on
     * xiaozhi's 1.75(C). The ES7210 stays on the same clocks. */
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan, &s_i2s.tx, &s_i2s.rx), TAG, "i2s chan");
    i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_SCLK,
            .ws = BSP_I2S_LCLK,
            .dout = BSP_I2S_DOUT,
            .din = BSP_I2S_DSIN,
        },
    };
    std.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    std.slot_cfg.ws_width = 32;
    std.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s.tx, &std), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s.rx, &std), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s.tx), TAG, "i2s tx on");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s.rx), TAG, "i2s rx on");

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .tx_handle = s_i2s.tx,
        .rx_handle = s_i2s.rx,
    };
    s_i2s.inner = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(s_i2s.inner, ESP_ERR_NO_MEM, TAG, "i2s data");
    s_i2s.base.open = slot_open;
    s_i2s.base.is_open = slot_is_open;
    s_i2s.base.enable = slot_enable;
    s_i2s.base.set_fmt = slot_set_fmt;
    s_i2s.base.read = slot_read;
    s_i2s.base.write = slot_write;
    s_i2s.base.close = slot_close;

    esp_err_t rst = es8311_reset();
    if (rst != ESP_OK) {
        ESP_LOGW(TAG, "ES8311 reset skipped (%s)", esp_err_to_name(rst));
    }

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    audio_codec_i2c_cfg_t i2c = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = bsp_i2c_get_handle(),
    };
    const audio_codec_ctrl_if_t *spk_ctrl = audio_codec_new_i2c_ctrl(&i2c);
    ESP_RETURN_ON_FALSE(gpio_if && spk_ctrl, ESP_ERR_NO_MEM, TAG, "spk ctrl");
    const audio_codec_if_t *spk_codec = es8311_codec_new(&(es8311_codec_cfg_t){
        .ctrl_if = spk_ctrl,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .hw_gain = { .pa_voltage = 5.0, .codec_dac_voltage = 3.3 },
    });
    ESP_RETURN_ON_FALSE(spk_codec, ESP_FAIL, TAG, "ES8311");
    *spk = s_out = esp_codec_dev_new(&(esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = spk_codec,
        .data_if = &s_i2s.base,
    });

    i2c.addr = ES7210_CODEC_DEFAULT_ADDR;
    const audio_codec_ctrl_if_t *mic_ctrl = audio_codec_new_i2c_ctrl(&i2c);
    ESP_RETURN_ON_FALSE(mic_ctrl, ESP_ERR_NO_MEM, TAG, "mic ctrl");
    const audio_codec_if_t *mic_codec = es7210_codec_new(&(es7210_codec_cfg_t){
        .ctrl_if = mic_ctrl,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2,
    });
    ESP_RETURN_ON_FALSE(mic_codec, ESP_FAIL, TAG, "ES7210");
    *mic = esp_codec_dev_new(&(esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = mic_codec,
        .data_if = &s_i2s.base,
    });
    ESP_RETURN_ON_FALSE(*spk && *mic, ESP_FAIL, TAG, "codec dev");
    ESP_LOGI(TAG, "I2S ready: %d Hz 16-in-32, MCLK %d BCLK %d WS %d DOUT %d DIN %d, PA GPIO%d, port %d",
             MUSE_AUDIO_RATE, (int)BSP_I2S_MCLK, (int)BSP_I2S_SCLK, (int)BSP_I2S_LCLK, (int)BSP_I2S_DOUT,
             (int)BSP_I2S_DSIN, (int)BSP_POWER_AMP_IO, CONFIG_BSP_I2S_NUM);
    return ESP_OK;
}

static void set_mic_gain(esp_codec_dev_handle_t mic, int db)
{
    /* ES7210 PGA steps are 3 dB; snap so the UI shows what's applied. */
    db = (db / 3) * 3;
    /* esp_codec_dev rounds 33 dB down to 30; the next real step up is 34.5. */
    esp_codec_dev_set_in_gain(mic, db == 33 ? 34.5f : (float)db);
}

#if CONFIG_MUSE_BOARD_WAVESHARE_S3_175
static unsigned poll_buttons(void)
{
    static unsigned tick;
    unsigned ev = muse_gpio_button_poll(&s_boot);   /* BOOT talks, PWR is aux */
    if (tick++ % PMU_KEY_EVERY == 0) {
        unsigned key = muse_pmu_poll_key();
        ev |= (key & MUSE_PMU_KEY_PRESS ? MUSE_BTN_AUX_PRESS : 0) |
              (key & MUSE_PMU_KEY_RELEASE ? MUSE_BTN_AUX_RELEASE : 0);
    }
    return ev;
}
#else
static unsigned poll_buttons(void)
{
    unsigned pwr = muse_gpio_button_poll(&s_pwr);
    /* A talk can outlast the PMU's hold-off: held, PWR mustn't power off. */
    if (pwr & MUSE_BTN_TALK_PRESS) {
        (void)muse_pmu_set_key_shutdown(false);   /* logs if it can't */
    }
    if (pwr & MUSE_BTN_TALK_RELEASE) {
        (void)muse_pmu_set_key_shutdown(true);    /* logs if it can't */
    }
    return pwr | muse_gpio_button_poll(&s_boot) << 2;   /* BOOT is aux */
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_buttons_wait((muse_gpio_button_t *const[]){ &s_pwr, &s_boot }, 2, timeout_ms);
}
#endif

static const muse_board_t s_board = {
#if CONFIG_MUSE_BOARD_WAVESHARE_S3_175
    .name = "Waveshare ESP32-S3-Touch-AMOLED-1.75",
#else
    .name = "Waveshare ESP32-S3-Touch-AMOLED-1.75C",
#endif
    .width = BSP_LCD_H_RES,
    .height = BSP_LCD_V_RES,
    .round = true,
    .touch = true,
    .diagonal_in = 1.75f,
#if CONFIG_MUSE_BOARD_WAVESHARE_S3_175
    .talk_button = "boot",
    .aux_button = "pwr",
    /* The same side buttons as the 1.75C, but BOOT (below) talks. */
    .talk_hint = { LV_ALIGN_CENTER, 153, 129 },
    .aux_hint = { LV_ALIGN_CENTER, 153, -129 },
#else
    .talk_button = "top",
    .aux_button = "bottom",
    /* Side buttons: PWR (talk) above BOOT (sleep/off), on the right. */
    .talk_hint = { LV_ALIGN_CENTER, 153, -129 },    /* 40 degrees above/below 3 o'clock */
    .aux_hint = { LV_ALIGN_CENTER, 153, 129 },
#endif
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    .draw_frame = muse_lcd_bands_frame,
    .audio_init = audio_init,
    .audio_power = audio_power,
    .speaker_probe = speaker_probe,
    .boot_tone = boot_tone,
    .mic_slot = -1,
    .set_mic_gain = set_mic_gain,
    .poll_buttons = poll_buttons,
#if !CONFIG_MUSE_BOARD_WAVESHARE_S3_175
    .wait_buttons = wait_buttons,   /* the 1.75's PWR is on the PMU, so it's polled */
#endif
    .read_power = muse_pmu_read_power,
    .power_off = muse_pmu_power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
