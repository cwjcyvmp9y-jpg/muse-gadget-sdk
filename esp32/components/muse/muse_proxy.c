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
 * Device-local Shadowsocks 2022 TCP bridge, ported from the atoms3r Muse
 * firmware (SIP022, one 16-byte PSK, 2022-blake3-aes-128-gcm, TCP 443).
 * The node sees only TLS bytes. Certificate hostname, SNI and HTTP Host stay
 * the original server. Volcengine TTS (openspeech.bytedance.com) is direct.
 *
 * The key comes from main/ss_proxy_secret.h when that gitignored file defines
 * a non-empty SS_PROXY_KEY (which also turns the proxy on), otherwise from
 * CONFIG_MUSE_SS_PROXY_ENABLE and CONFIG_MUSE_SS_KEY. The key is never logged.
 */

#include "muse_proxy.h"

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#if __has_include("ss_proxy_secret.h")
#include "ss_proxy_secret.h"
#endif

#ifndef CONFIG_MUSE_SS_PROXY_ENABLE
#define CONFIG_MUSE_SS_PROXY_ENABLE 0
#endif
#ifndef CONFIG_MUSE_SS_SERVER
#define CONFIG_MUSE_SS_SERVER ""
#endif
#ifndef CONFIG_MUSE_SS_PORT
#define CONFIG_MUSE_SS_PORT 0
#endif
#ifndef CONFIG_MUSE_SS_KEY
#define CONFIG_MUSE_SS_KEY ""
#endif

#include "muse_ss2022.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mbedtls/base64.h"
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define ROUTES 6
#define CONNECTIONS 4
#define IO_SIZE 2048
#define CLOCK_OK 1735689600 /* 2025-01-01: SNTP has moved the clock */
#define CLOCK_WAIT_MS 15000

static const char *TAG = "ss_proxy";

typedef struct {
    char host[256];
    uint16_t port, local;
    int fd;
} route_t;

typedef struct {
    int client;
    route_t *route;
} job_t;

static route_t s_routes[ROUTES];
static uint8_t s_psk[16];
static bool s_initialized;
static bool s_clock_logged;
static StaticSemaphore_t s_lock_mem;
static SemaphoreHandle_t s_lock;
static portMUX_TYPE s_guard = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_active;

static const char *proxy_server(void)
{
#ifdef SS_PROXY_SERVER
    if (SS_PROXY_SERVER[0]) {
        return SS_PROXY_SERVER;
    }
#endif
    return CONFIG_MUSE_SS_SERVER;
}

static int proxy_port(void)
{
#ifdef SS_PROXY_PORT
    if (SS_PROXY_PORT > 0) {
        return SS_PROXY_PORT;
    }
#endif
    return CONFIG_MUSE_SS_PORT;
}

static const char *proxy_key(void)
{
#ifdef SS_PROXY_KEY
    if (SS_PROXY_KEY[0]) {
        return SS_PROXY_KEY;
    }
#endif
    return CONFIG_MUSE_SS_KEY;
}

static bool proxy_on(void)
{
    const char *key = proxy_key();
    if (!key || !key[0]) {
        return false;
    }
#ifdef SS_PROXY_KEY
    if (SS_PROXY_KEY[0]) {
        return true;
    }
#endif
    return CONFIG_MUSE_SS_PROXY_ENABLE;
}

static bool host_is(const char *host, const char *name)
{
    for (; *name; host++, name++) {
        char a = *host;
        char b = *name;
        if (a >= 'A' && a <= 'Z') {
            a = (char)(a + 32);
        }
        if (b >= 'A' && b <= 'Z') {
            b = (char)(b + 32);
        }
        if (a != b) {
            return false;
        }
    }
    return *host == '\0';
}

/* China services stay on the local network. */
static bool direct_host(const char *host)
{
    return host && host_is(host, "openspeech.bytedance.com");
}

static void *big_alloc(size_t n)
{
    return heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static bool send_all(int fd, const uint8_t *p, size_t n)
{
    while (n) {
        ssize_t done = send(fd, p, n, 0);
        if (done < 0 && errno == EINTR) {
            continue;
        }
        if (done <= 0) {
            return false;
        }
        p += done;
        n -= (size_t)done;
    }
    return true;
}

static bool emit(void *arg, const uint8_t *p, size_t n)
{
    return send_all(*(int *)arg, p, n);
}

static void io_timeout(int fd)
{
    struct timeval tv = { .tv_sec = 5 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
}

static int dial(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        ESP_LOGW(TAG, "connect to proxy fail");
        return -1;
    }
    struct sockaddr_in a = {
        .sin_family = AF_INET,
        .sin_port = htons((uint16_t)proxy_port()),
    };
    if (inet_pton(AF_INET, proxy_server(), &a.sin_addr) != 1) {
        close(fd);
        ESP_LOGW(TAG, "connect to proxy fail");
        return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int rc = connect(fd, (struct sockaddr *)&a, sizeof(a));
    if (rc < 0 && errno == EINPROGRESS) {
        fd_set w;
        FD_ZERO(&w);
        FD_SET(fd, &w);
        struct timeval tv = { .tv_sec = 12 };
        rc = select(fd + 1, NULL, &w, NULL, &tv);
        int error = 0;
        socklen_t n = sizeof(error);
        if (rc > 0 && (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &n) || error)) {
            rc = -1;
        } else if (rc > 0) {
            rc = 0;
        } else {
            rc = -1;
        }
    }
    if (rc < 0) {
        close(fd);
        ESP_LOGW(TAG, "connect to proxy fail");
        return -1;
    }
    fcntl(fd, F_SETFL, flags);
    io_timeout(fd);
    ESP_LOGI(TAG, "connect to proxy ok");
    return fd;
}

static void bridge(void *arg)
{
    job_t *j = arg;
    int local = j->client;
    int remote = -1;
    route_t *route = j->route;
    free(j);
    ss2022_cipher_t tx = { 0 };
    ss2022_rx_t *rx = big_alloc(sizeof(*rx));
    uint8_t *input = big_alloc(IO_SIZE);
    uint8_t *wire = big_alloc(IO_SIZE + 1200);
    bool ok = false;
    bool rx_ready = false;
    uint8_t salt[16];
    uint8_t padding[64];
    esp_fill_random(salt, sizeof(salt));
    esp_fill_random(padding, sizeof(padding));
    size_t wire_len = 0;
    if (!rx || !input || !wire) {
        ESP_LOGW(TAG, "no buffer for %s:%u", route->host, route->port);
        goto done;
    }
    ss2022_rx_init(rx, s_psk, salt);
    rx_ready = true;
    if (!ss2022_cipher_init(&tx, s_psk, salt)) {
        ESP_LOGW(TAG, "session key failed for %s:%u", route->host, route->port);
        goto done;
    }
    remote = dial();
    if (remote < 0) {
        goto done;
    }
    io_timeout(local);
    if (!ss2022_request(&tx, salt, route->host, route->port, (uint64_t)time(NULL), padding,
                        1 + padding[0] % sizeof(padding), wire, IO_SIZE + 1200, &wire_len) ||
        !send_all(remote, wire, wire_len)) {
        ESP_LOGW(TAG, "tunnel open failed %s:%u", route->host, route->port);
        goto done;
    }
    ESP_LOGI(TAG, "tunnel open %s:%u", route->host, route->port);
    int64_t last_io = esp_timer_get_time();
    for (;;) {
        fd_set read;
        FD_ZERO(&read);
        FD_SET(local, &read);
        FD_SET(remote, &read);
        struct timeval tv = { .tv_sec = 1 };
        int rc = select((local > remote ? local : remote) + 1, &read, NULL, NULL, &tv);
        if (rc < 0 && errno == EINTR) {
            continue;
        }
        if (rc < 0 || esp_timer_get_time() - last_io > 180000000LL) {
            break;
        }
        if (!rc) {
            continue;
        }
        if (FD_ISSET(local, &read)) {
            ssize_t n = recv(local, input, IO_SIZE, 0);
            if (n == 0) {
                ok = true;
                break;
            }
            if (n < 0 || !ss2022_payload(&tx, input, (size_t)n, wire, IO_SIZE + 1200, &wire_len) ||
                !send_all(remote, wire, wire_len)) {
                break;
            }
            last_io = esp_timer_get_time();
        }
        if (FD_ISSET(remote, &read)) {
            ssize_t n = recv(remote, input, IO_SIZE, 0);
            if (n == 0) {
                ok = true;
                break;
            }
            if (n < 0 || !ss2022_feed(rx, input, (size_t)n, (uint64_t)time(NULL), emit, &local)) {
                ESP_LOGW(TAG, "response authentication failed for %s:%u", route->host, route->port);
                break;
            }
            last_io = esp_timer_get_time();
        }
    }
    if (!ok) {
        ESP_LOGW(TAG, "tunnel error %s:%u", route->host, route->port);
    }
done:
    if (remote >= 0) {
        close(remote);
    }
    close(local);
    ss2022_cipher_free(&tx);
    if (rx_ready) {
        ss2022_rx_free(rx);
    }
    free(rx);
    free(input);
    free(wire);
    taskENTER_CRITICAL(&s_guard);
    s_active--;
    taskEXIT_CRITICAL(&s_guard);
    vTaskDeleteWithCaps(NULL);
}

static void listener(void *arg)
{
    route_t *r = arg;
    for (;;) {
        int fd = accept(r->fd, NULL, NULL);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        taskENTER_CRITICAL(&s_guard);
        bool room = s_active < CONNECTIONS;
        if (room) {
            s_active++;
        }
        taskEXIT_CRITICAL(&s_guard);
        if (!room) {
            ESP_LOGW(TAG, "too many tunnels");
            close(fd);
            continue;
        }
        job_t *j = malloc(sizeof(*j));
        if (j) {
            j->client = fd;
            j->route = r;
        }
        if (!j || xTaskCreateWithCaps(bridge, "ss_proxy_io", 12288, j, 5, NULL,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
            ESP_LOGW(TAG, "tunnel task failed for %s:%u", r->host, r->port);
            free(j);
            close(fd);
            taskENTER_CRITICAL(&s_guard);
            s_active--;
            taskEXIT_CRITICAL(&s_guard);
        }
    }
}

static esp_err_t initialize(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    size_t n = 0;
    const char *key = proxy_key();
    if (mbedtls_base64_decode(s_psk, sizeof(s_psk), &n, (const unsigned char *)key, strlen(key)) || n != 16) {
        ESP_LOGW(TAG, "proxy key rejected");
        return ESP_ERR_INVALID_ARG;
    }
    if (!esp_sntp_enabled()) {
        esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "ntp.aliyun.com");
#if CONFIG_LWIP_SNTP_MAX_SERVERS > 1
        esp_sntp_setservername(1, "time.cloudflare.com");
#endif
        esp_sntp_init();
    }
    s_initialized = true;
    return ESP_OK;
}

static esp_err_t wait_clock(void)
{
    esp_err_t err = initialize();
    if (err != ESP_OK) {
        return err;
    }
    int waited = 0;
    while (time(NULL) < CLOCK_OK && waited < CLOCK_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(100));
        waited += 100;
    }
    if (time(NULL) < CLOCK_OK) {
        ESP_LOGW(TAG, "time sync fail");
        return ESP_ERR_TIMEOUT;
    }
    if (!s_clock_logged) {
        ESP_LOGI(TAG, "time sync ok");
        s_clock_logged = true;
    }
    return ESP_OK;
}

static esp_err_t route(const char *host, uint16_t port, uint16_t *local_port)
{
    taskENTER_CRITICAL(&s_guard);
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_mem);
    }
    taskEXIT_CRITICAL(&s_guard);
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t result = wait_clock();
    if (result != ESP_OK) {
        goto done;
    }
    for (int i = 0; i < ROUTES; i++) {
        route_t *r = &s_routes[i];
        if (r->host[0] && !strcmp(r->host, host) && r->port == port) {
            *local_port = r->local;
            goto done;
        }
    }
    result = ESP_ERR_NO_MEM;
    for (int i = 0; i < ROUTES; i++) {
        route_t *r = &s_routes[i];
        if (r->host[0]) {
            continue;
        }
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            ESP_LOGW(TAG, "listener socket failed for %s:%u", host, port);
            break;
        }
        struct sockaddr_in a = {
            .sin_family = AF_INET,
            .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
            .sin_port = 0,
        };
        socklen_t n = sizeof(a);
        if (bind(fd, (struct sockaddr *)&a, sizeof(a)) || listen(fd, 2) ||
            getsockname(fd, (struct sockaddr *)&a, &n)) {
            close(fd);
            ESP_LOGW(TAG, "listener bind failed for %s:%u", host, port);
            break;
        }
        strlcpy(r->host, host, sizeof(r->host));
        r->port = port;
        r->local = ntohs(a.sin_port);
        r->fd = fd;
        if (xTaskCreate(listener, "ss_proxy_accept", 3072, r, 4, NULL) != pdPASS) {
            close(fd);
            memset(r, 0, sizeof(*r));
            ESP_LOGW(TAG, "listener task failed for %s:%u", host, port);
            break;
        }
        *local_port = r->local;
        result = ESP_OK;
        break;
    }
done:
    xSemaphoreGive(s_lock);
    return result;
}

static bool https_host(const char *url, char *host, size_t cap, const char **path)
{
    if (strncmp(url, "https://", 8) != 0) {
        return false;
    }
    const char *start = url + 8;
    const char *slash = strchr(start, '/');
    const char *end = slash ? slash : start + strlen(start);
    const char *at = memchr(start, '@', (size_t)(end - start));
    if (at) {
        start = at + 1;
    }
    const char *colon = memchr(start, ':', (size_t)(end - start));
    size_t n = (size_t)((colon ? colon : end) - start);
    if (!n || n >= cap) {
        return false;
    }
    memcpy(host, start, n);
    host[n] = '\0';
    if (colon) {
        int port = atoi(colon + 1);
        if (port != 443) {
            return false;
        }
    }
    if (path) {
        *path = slash;
    }
    return true;
}

esp_err_t muse_proxy_prepare(const char *host, uint16_t port, bool *via_proxy, uint16_t *connect_port)
{
    if (!host || !via_proxy || !connect_port || !host[0] || strlen(host) > 255) {
        return ESP_ERR_INVALID_ARG;
    }
    *via_proxy = false;
    if (!proxy_on() || direct_host(host) || port != 443) {
        return ESP_OK;
    }
    uint16_t local = 0;
    esp_err_t err = route(host, port, &local);
    if (err != ESP_OK) {
        return err;
    }
    *via_proxy = true;
    *connect_port = local;
    return ESP_OK;
}

esp_err_t muse_proxy_http_url(const char *url, char *out, size_t cap, char *origin, size_t origin_cap)
{
    if (!url || !out || !origin || !cap || !origin_cap) {
        return ESP_ERR_INVALID_ARG;
    }
    char host[256];
    const char *path = NULL;
    if (!https_host(url, host, sizeof(host), &path)) {
        if (!proxy_on()) {
            int n = snprintf(out, cap, "%s", url);
            origin[0] = '\0';
            return n >= 0 && (size_t)n < cap ? ESP_OK : ESP_ERR_INVALID_SIZE;
        }
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(host) >= origin_cap) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(origin, host, strlen(host) + 1);
    if (!proxy_on() || direct_host(host)) {
        int n = snprintf(out, cap, "%s", url);
        return n >= 0 && (size_t)n < cap ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    uint16_t local = 0;
    esp_err_t err = route(host, 443, &local);
    if (err != ESP_OK) {
        return err;
    }
    int n = snprintf(out, cap, "https://127.0.0.1:%u%s", local, path ? path : "/");
    return n >= 0 && (size_t)n < cap ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
