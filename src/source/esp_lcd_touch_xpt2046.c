/*
 * SPDX-FileCopyrightText: 2025 Min9802
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"

static const char *TAG = "XPT2046";

/* XPT2046 registers and commands */
#define XPT2046_CMD_Z1      0xB1  /* Z1 (pressure) */
#define XPT2046_CMD_Z2      0xC1  /* Z2 (pressure) */
#define XPT2046_CMD_X       0x91  /* X position */
#define XPT2046_CMD_Y_PD0   0xD1  /* Y position, pen down */
#define XPT2046_CMD_Y_PD1   0xD0  /* Y position, power down */

#define XPT2046_SAMPLES         CONFIG_LCD_TOUCH_XPT2046_SAMPLES
#define XPT2046_Z_THRESHOLD     CONFIG_LCD_TOUCH_XPT2046_Z_THRESHOLD

typedef struct {
    spi_device_handle_t spi_dev;
    esp_lcd_touch_config_t config;
    xpt2046_calibration_t calibration;
    uint16_t x;
    uint16_t y;
    bool pressed;
} xpt2046_touch_t;

static uint16_t xpt2046_transfer16(spi_device_handle_t spi, uint8_t cmd)
{
    uint8_t tx_data[3] = {cmd, 0x00, 0x00};
    uint8_t rx_data[3] = {0};
    
    spi_transaction_t trans = {
        .length = 24,  /* 3 bytes = 24 bits */
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    spi_device_transmit(spi, &trans);
    
    /* XPT2046 returns 12-bit data: MSB in byte[1] bits[7:0], LSB in byte[2] bits[7:4] */
    return ((rx_data[1] << 8) | rx_data[2]) >> 3;
}

static int16_t besttwoavg(int16_t x, int16_t y, int16_t z)
{
    int16_t da = (x > y) ? (x - y) : (y - x);
    int16_t db = (x > z) ? (x - z) : (z - x);
    int16_t dc = (z > y) ? (z - y) : (y - z);
    
    if (da <= db && da <= dc) return (x + y) >> 1;
    else if (db <= da && db <= dc) return (x + z) >> 1;
    else return (y + z) >> 1;
}

static esp_err_t xpt2046_read_data(esp_lcd_touch_handle_t tp)
{
    xpt2046_touch_t *xpt = (xpt2046_touch_t *)tp->config.user_data;
    int16_t data[6];
    int z;
    
    /* Read pressure using XPT2046 sequence: Z1 → Z2 → check threshold */
    int16_t z1 = xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_Z1);
    z = z1 + 4095;
    int16_t z2 = xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_Z2);
    z -= z2;
    
    if (z < 0) z = 0;
    
    if (z < XPT2046_Z_THRESHOLD) {
        xpt->pressed = false;
        return ESP_OK;
    }
    
    /* Perform XPT2046 recommended reading sequence:
     * 1. Dummy X read (first is always noisy)
     * 2. Read Y, X, Y, X, Y, X (3 pairs for averaging)
     * 3. Final Y with power down
     */
    xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_X);  /* Dummy */
    
    data[0] = xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_Y_PD0);  /* Y1 */
    data[1] = xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_X);       /* X1 */
    data[2] = xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_Y_PD0);  /* Y2 */
    data[3] = xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_X);       /* X2 */
    data[4] = xpt2046_transfer16(xpt->spi_dev, XPT2046_CMD_Y_PD1);  /* Y3, power down */
    data[5] = xpt2046_transfer16(xpt->spi_dev, 0);                   /* X3 */
    
    /* Average using best two of three measurements */
    int16_t x_raw = besttwoavg(data[1], data[3], data[5]);
    int16_t y_raw = besttwoavg(data[0], data[2], data[4]);
    
    /* Apply calibration */
    int32_t x_cal = x_raw;
    int32_t y_cal = y_raw;
    
    #ifdef CONFIG_LCD_TOUCH_XPT2046_ENABLE_CALIBRATION
    if (xpt->calibration.x_max > xpt->calibration.x_min) {
        x_cal = ((x_raw - xpt->calibration.x_min) * xpt->config.x_max) / 
                (xpt->calibration.x_max - xpt->calibration.x_min);
    }
    if (xpt->calibration.y_max > xpt->calibration.y_min) {
        y_cal = ((y_raw - xpt->calibration.y_min) * xpt->config.y_max) / 
                (xpt->calibration.y_max - xpt->calibration.y_min);
    }
    #else
    /* Default scaling for typical XPT2046 */
    x_cal = (x_raw * xpt->config.x_max) / 4095;
    y_cal = (y_raw * xpt->config.y_max) / 4095;
    #endif
    
    /* Apply swap and invert */
    if (xpt->config.flags.swap_xy) {
        int32_t tmp = x_cal;
        x_cal = y_cal;
        y_cal = tmp;
    }
    
    if (xpt->config.flags.mirror_x) {
        x_cal = xpt->config.x_max - x_cal;
    }
    
    if (xpt->config.flags.mirror_y) {
        y_cal = xpt->config.y_max - y_cal;
    }
    
    /* Clamp values */
    xpt->x = (x_cal < 0) ? 0 : ((x_cal > xpt->config.x_max) ? xpt->config.x_max : x_cal);
    xpt->y = (y_cal < 0) ? 0 : ((y_cal > xpt->config.y_max) ? xpt->config.y_max : y_cal);
    xpt->pressed = true;
    
    return ESP_OK;
}

static bool xpt2046_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    xpt2046_touch_t *xpt = (xpt2046_touch_t *)tp->config.user_data;
    
    /* XPT2046 supports only single touch */
    if (max_point_num < 1) {
        return false;
    }
    
    if (!xpt->pressed) {
        if (point_num) *point_num = 0;
        return false;
    }
    
    /* Return coordinates */
    if (x) x[0] = xpt->x;
    if (y) y[0] = xpt->y;
    if (strength) strength[0] = 50;  /* XPT2046 doesn't provide precise pressure */
    if (point_num) *point_num = 1;
    
    return true;
}

static esp_err_t xpt2046_del(esp_lcd_touch_handle_t tp)
{
    xpt2046_touch_t *xpt = (xpt2046_touch_t *)tp->config.user_data;
    
    /* Free resources */
    if (xpt) {
        if (xpt->spi_dev) {
            spi_bus_remove_device(xpt->spi_dev);
        }
        free(xpt);
    }
    
    return ESP_OK;
}

esp_err_t esp_lcd_touch_new_spi_xpt2046(const xpt2046_spi_config_t *spi_config,
                                         const esp_lcd_touch_config_t *touch_config,
                                         esp_lcd_touch_handle_t *out_touch)
{
    ESP_RETURN_ON_FALSE(spi_config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid SPI config");
    ESP_RETURN_ON_FALSE(touch_config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid touch config");
    ESP_RETURN_ON_FALSE(out_touch != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid output handle");
    
    /* Allocate touch handle */
    esp_lcd_touch_handle_t touch = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_RETURN_ON_FALSE(touch != NULL, ESP_ERR_NO_MEM, TAG, "No memory for touch handle");
    
    /* Allocate XPT2046 data */
    xpt2046_touch_t *xpt = calloc(1, sizeof(xpt2046_touch_t));
    if (xpt == NULL) {
        free(touch);
        ESP_LOGE(TAG, "No memory for XPT2046 data");
        return ESP_ERR_NO_MEM;
    }
    
    /* Add SPI device to bus */
    spi_device_interface_config_t dev_cfg = {
        .mode = 0,  /* SPI mode 0 */
        .clock_speed_hz = spi_config->spi_freq_hz > 0 ? spi_config->spi_freq_hz : 2000000,  /* 2MHz default */
        .spics_io_num = spi_config->cs_gpio_num,
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    
    esp_err_t ret = spi_bus_add_device(spi_config->spi_host, &dev_cfg, &xpt->spi_dev);
    if (ret != ESP_OK) {
        free(xpt);
        free(touch);
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Initialize XPT2046 data */
    memcpy(&xpt->config, touch_config, sizeof(esp_lcd_touch_config_t));
    xpt->pressed = false;
    
    /* Default calibration from ESP32-2432S028 (CYD) Arduino example */
    xpt->calibration.x_min = 200;
    xpt->calibration.x_max = 3700;
    xpt->calibration.y_min = 240;
    xpt->calibration.y_max = 3800;
    
    /* Configure interrupt pin if provided */
    if (touch_config->int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << touch_config->int_gpio_num),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,  /* Input-only GPIO can't use internal pull-up */
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,  /* Polling mode for now */
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to configure IRQ GPIO %d (input-only?): %s", 
                     touch_config->int_gpio_num, esp_err_to_name(ret));
        }
    }
    
    /* Fill touch handle */
    touch->config = *touch_config;
    touch->config.user_data = xpt;
    touch->read_data = xpt2046_read_data;
    touch->get_xy = xpt2046_get_xy;
    touch->del = xpt2046_del;
    
    *out_touch = touch;
    
    ESP_LOGI(TAG, "XPT2046 touch initialized, version: 1.0.0");
    
    return ESP_OK;
}

esp_err_t esp_lcd_touch_xpt2046_set_calibration(esp_lcd_touch_handle_t touch,
                                                 const xpt2046_calibration_t *calibration)
{
    ESP_RETURN_ON_FALSE(touch != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid touch handle");
    ESP_RETURN_ON_FALSE(calibration != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid calibration");
    
    xpt2046_touch_t *xpt = (xpt2046_touch_t *)touch->config.user_data;
    memcpy(&xpt->calibration, calibration, sizeof(xpt2046_calibration_t));
    
    ESP_LOGI(TAG, "Calibration set: X[%d:%d], Y[%d:%d]",
             calibration->x_min, calibration->x_max,
             calibration->y_min, calibration->y_max);
    
    return ESP_OK;
}
