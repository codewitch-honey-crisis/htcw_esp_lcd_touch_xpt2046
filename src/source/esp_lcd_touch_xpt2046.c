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
#ifndef XPT2046_SAMPLES
#define XPT2046_SAMPLES         3
#endif
#ifndef XPT2046_Z_THRESHOLD
#define XPT2046_Z_THRESHOLD     400
#endif
typedef struct {
    esp_lcd_touch_config_t config;
    uint16_t x_min;
    uint16_t y_min;
    uint16_t x_max;
    uint16_t y_max;
    uint16_t threshhold;
} xpt2046_touch_t;

static esp_err_t xpt2046_transfer16(esp_lcd_panel_io_handle_t io, uint8_t cmd, int16_t* out_result)
{
    uint8_t rx_data[2] = {0};
    esp_err_t ret = esp_lcd_panel_io_rx_param(io,cmd,rx_data,sizeof(rx_data));
    if(ret!=ESP_OK) {
        return ret;
    }
    if(out_result==NULL) { return ESP_OK; }
    /* XPT2046 returns 12-bit data: MSB in byte[1] bits[7:0], LSB in byte[2] bits[7:4] */
    *out_result = ((rx_data[0] << 8) | rx_data[1]) >> 3;
    return ESP_OK;
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
    int16_t z1;
    esp_err_t ret = xpt2046_transfer16(tp->io, XPT2046_CMD_Z1, &z1);
    if(ret!=ESP_OK) {
        return ret;
    }
    z = z1 + 4095;
    int16_t z2;
    ret = xpt2046_transfer16(tp->io, XPT2046_CMD_Z2,&z2);
    z -= z2;
    if (z < 0) z = 0;
    if (z < xpt->threshhold) {
        return ESP_OK;
    }
    /* Perform XPT2046 recommended reading sequence:
     * 1. Dummy X read (first is always noisy)
     * 2. Read Y, X, Y, X, Y, X (3 pairs for averaging)
     * 3. Final Y with power down
     */
    
    xpt2046_transfer16(tp->io, XPT2046_CMD_X, NULL);  /* Dummy */
    
    ret = xpt2046_transfer16(tp->io, XPT2046_CMD_Y_PD0,&data[0]);  /* Y1 */
    if(ret!=ESP_OK) {return ret;}
    ret = xpt2046_transfer16(tp->io, XPT2046_CMD_X,&data[1]);       /* X1 */
    if(ret!=ESP_OK) {return ret;}
    ret = xpt2046_transfer16(tp->io, XPT2046_CMD_Y_PD0,&data[2]);  /* Y2 */
    if(ret!=ESP_OK) {return ret;}
    ret = xpt2046_transfer16(tp->io, XPT2046_CMD_X,&data[3]);       /* X2 */
    if(ret!=ESP_OK) {return ret;}
    ret = xpt2046_transfer16(tp->io, XPT2046_CMD_Y_PD1,&data[4]);  /* Y3, power down */
    if(ret!=ESP_OK) {return ret;}
    ret = xpt2046_transfer16(tp->io, 0,&data[5]);                   /* X3 */
    if(ret!=ESP_OK) {return ret;}
    /* Average using best two of three measurements */
    int16_t x_raw = besttwoavg(data[1], data[3], data[5]);
    int16_t y_raw = besttwoavg(data[0], data[2], data[4]);
     /* Apply calibration */
    int32_t x_cal = x_raw;
    int32_t y_cal = y_raw;
    
    if (xpt->x_max > xpt->x_min) {
        x_cal = ((x_raw - xpt->x_min) * xpt->config.x_max) / 
                (xpt->x_max - xpt->x_min);
    }
    if (xpt->y_max > xpt->y_min) {
        y_cal = ((y_raw - xpt->y_min) * xpt->config.y_max) / 
                (xpt->y_max - xpt->y_min);
    }
    portENTER_CRITICAL(&tp->data.lock);
      
    /* Clamp values */
    tp->data.points = 1;
    tp->data.coords[0].x = (x_cal < 0) ? 0 : ((x_cal > xpt->config.x_max) ? xpt->config.x_max : x_cal);
    tp->data.coords[0].y = (y_cal < 0) ? 0 : ((y_cal > xpt->config.y_max) ? xpt->config.y_max : y_cal);
    
    portEXIT_CRITICAL(&tp->data.lock);
    
    return ESP_OK;
}

static bool xpt2046_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++)
    {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength)
        {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t xpt2046_del(esp_lcd_touch_handle_t tp)
{
    xpt2046_touch_t *xpt = (xpt2046_touch_t *)tp->config.user_data;
    
    /* Free resources */
    if (xpt) {
         free(xpt);
    }
    
    return ESP_OK;
}

esp_err_t esp_lcd_touch_new_xpt2046(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    ESP_RETURN_ON_FALSE(io != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid io handle");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid touch config");
    ESP_RETURN_ON_FALSE(out_touch != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid output handle");
    esp_err_t ret;
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
    /* Initialize XPT2046 data */
    memcpy(&xpt->config, config, sizeof(esp_lcd_touch_config_t));
    if(config->driver_data==NULL) {
        /* Default calibration from ESP32-2432S028 (CYD) Arduino example */
        xpt->x_min = 200;
        xpt->x_max = 3700;
        xpt->y_min = 240;
        xpt->y_max = 3800;
        xpt->threshhold = XPT2046_Z_THRESHOLD;
    } else {
        esp_lcd_touch_xtp2046_config_t* drv = (esp_lcd_touch_xtp2046_config_t*)config->driver_data;
        xpt->x_min = drv->x_min;
        xpt->y_min = drv->y_min;
        xpt->x_max = drv->x_max;
        xpt->y_max = drv->y_max;
        xpt->threshhold = drv->threshhold;
    }
    touch->data.lock.owner = portMUX_FREE_VAL;
    
    /* Configure interrupt pin if provided */
    if (config->int_gpio_num != GPIO_NUM_NC)
    {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = (config->levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
            .pin_bit_mask = BIT64(config->int_gpio_num)};
        ret = gpio_config(&int_gpio_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to configure IRQ GPIO %d (input-only?): %s", 
                     config->int_gpio_num, esp_err_to_name(ret));
        }

        /* Register interrupt callback */
        if (config->interrupt_callback)
        {
            esp_lcd_touch_register_interrupt_callback(touch, config->interrupt_callback);
        }
    }
    
    /* Fill touch handle */
    touch->io = io;
    touch->config = *config;
    touch->config.user_data = xpt;
    touch->read_data = xpt2046_read_data;
    touch->get_xy = xpt2046_get_xy;
    touch->get_mirror_x = NULL;
    touch->get_mirror_y = NULL;
    touch->get_swap_xy = NULL;
    touch->del = xpt2046_del;
    
    *out_touch = touch;
    
    ESP_LOGI(TAG, "XPT2046 touch initialized, version: 0.2.0");
    
    return ESP_OK;
}
