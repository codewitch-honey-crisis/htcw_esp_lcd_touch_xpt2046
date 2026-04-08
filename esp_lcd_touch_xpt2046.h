/*
 * SPDX-FileCopyrightText: 2025 Min9802
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t x_min;
    uint16_t y_min;
    uint16_t x_max;
    uint16_t y_max;
    uint16_t threshhold;
} esp_lcd_touch_xtp2046_config_t;

/**
 * @brief Create a new XPT2046 touch panel
 *
 * @param[in]  io I/O configuration
 * @param[in]  touch_config Touch panel configuration
 * @param[out] out_touch Touch panel handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 */
esp_err_t esp_lcd_touch_new_xpt2046(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch);

#ifdef __cplusplus
}
#endif
