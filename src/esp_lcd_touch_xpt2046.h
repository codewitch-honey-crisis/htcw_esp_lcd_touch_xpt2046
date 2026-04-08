/*
 * SPDX-FileCopyrightText: 2025 Min9802
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_lcd_touch.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief XPT2046 configuration structure
 */
typedef struct {
    spi_host_device_t spi_host;     /*!< SPI host (SPI2_HOST or SPI3_HOST) */
    int cs_gpio_num;                /*!< SPI CS GPIO number */
    int spi_freq_hz;                /*!< SPI clock frequency (default: 2MHz) */
} xpt2046_spi_config_t;

/**
 * @brief Create a new XPT2046 touch panel
 *
 * @param[in]  spi_config SPI configuration
 * @param[in]  touch_config Touch panel configuration
 * @param[out] out_touch Touch panel handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 */
esp_err_t esp_lcd_touch_new_spi_xpt2046(const xpt2046_spi_config_t *spi_config,
                                         const esp_lcd_touch_config_t *touch_config,
                                         esp_lcd_touch_handle_t *out_touch);

/**
 * @brief XPT2046 configuration structure
 */
typedef struct {
    int16_t x_min;          /*!< Minimum X raw value */
    int16_t x_max;          /*!< Maximum X raw value */
    int16_t y_min;          /*!< Minimum Y raw value */
    int16_t y_max;          /*!< Maximum Y raw value */
    bool swap_xy;           /*!< Swap X and Y coordinates */
    bool invert_x;          /*!< Invert X coordinate */
    bool invert_y;          /*!< Invert Y coordinate */
} xpt2046_calibration_t;

/**
 * @brief Set XPT2046 calibration parameters
 *
 * @param[in] touch Touch panel handle
 * @param[in] calibration Calibration parameters
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_lcd_touch_xpt2046_set_calibration(esp_lcd_touch_handle_t touch,
                                                 const xpt2046_calibration_t *calibration);

#ifdef __cplusplus
}
#endif
