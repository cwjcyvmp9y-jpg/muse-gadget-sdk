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
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Volcengine (火山引擎) unidirectional HTTP TTS for a finished assistant reply.
 * The key is never logged. Disabled when no key is configured. */

bool muse_volc_tts_enabled(void);

/* Copy `text` and speak it on a task of its own. PCM is queued for the voice
 * task (muse_hatch_reply_pcm). False if TTS is off, busy, or the task did not start. */
bool muse_volc_tts_submit(const char *text);

/* 0 while the task is running or idle. 1 if the last request produced audio.
 * -1 if it failed (native reply audio should play). Clears the result. */
int muse_volc_tts_take_result(int *http_status, int *api_code, uint32_t *frames);

#ifdef __cplusplus
}
#endif
