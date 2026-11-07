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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* esp_err_t is provided by esp_err.h on device. Host fakes typedef it first. */
#if defined(__has_include)
#  if __has_include("esp_err.h")
#    include "esp_err.h"
#  endif
#endif
#ifndef ESP_OK
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_TIMEOUT 0x107
#endif

/* Local TCP bridge for one Shadowsocks 2022 node (2022-blake3-aes-128-gcm).
 * TLS stays end to end: callers keep the original hostname as the certificate
 * name and the HTTP Host. A required proxy that cannot be used fails closed.
 * openspeech.bytedance.com is never tunneled. */

/* On ESP_OK, *via_proxy is true when `host` must be reached by connecting to
 * 127.0.0.1:*connect_port with the TLS common name left as `host`. False means
 * connect to host:port directly. Any other return means do not connect. */
esp_err_t muse_proxy_prepare(const char *host, uint16_t port, bool *via_proxy, uint16_t *connect_port);

/* Rewrite an https URL onto the local bridge, or copy it through when the
 * proxy is off or the host is direct. `origin` receives the real hostname.
 * Fail closed when the proxy is required and the route is not available. */
esp_err_t muse_proxy_http_url(const char *url, char *out, size_t cap, char *origin, size_t origin_cap);

#ifdef __cplusplus
}
#endif
