/*
 * SPDX-FileCopyrightText: 2022 atanisoft (github.com/atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_lcd_panel_io.h>
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// This must be included after FreeRTOS includes due to missing include
// for portMUX_TYPE
#include <esp_lcd_touch.h>
#include <memory.h>

#include "sdkconfig.h"
#include "include/esp_lcd_touch_xpt2046.h"
#include "include/util.h"

static const char *TAG = "xpt2046";

enum xpt2046_registers
{
    Z_VALUE_1   = 0xB0,
    Z_VALUE_2   = 0xC0,
    Y_POSITION  = 0x90,
    X_POSITION  = 0xD0,
    BATTERY     = 0xA7,
};

#if CONFIG_XPT2046_ENABLE_LOCKING
#define XPT2046_LOCK(lock) portENTER_CRITICAL(lock)
#define XPT2046_UNLOCK(lock) portEXIT_CRITICAL(lock)
#else
#define XPT2046_LOCK(lock)
#define XPT2046_UNLOCK(lock)
#endif

static const uint16_t XPT2046_ADC_LIMIT = 4096;
gpio_num_t gpio_irq=0;

#define XPT2046_AVG 7
uint32_t avg_buf_x[XPT2046_AVG];
uint32_t avg_buf_y[XPT2046_AVG];
uint8_t avg_last;
uint32_t x_scale = 0, y_scale = 0;

static esp_err_t xpt2046_read_data(esp_lcd_touch_handle_t tp);
static bool xpt2046_get_xy(esp_lcd_touch_handle_t tp,
                           uint16_t *x, uint16_t *y,
                           uint16_t *strength,
                           uint8_t *point_num,
                           uint8_t max_point_num);
static esp_err_t xpt2046_del(esp_lcd_touch_handle_t tp);
static void xpt2046_avg(uint16_t *x, uint16_t *y);
static void first_set_buf_avg(uint16_t *x, uint16_t *y);
static void clear_set_buf_avg(void);

bool isFirst = true;
uint32_t DISP_BUF_SIZE=0;

esp_err_t esp_lcd_touch_new_spi_xpt2046(const esp_lcd_panel_io_handle_t io,
                                        const esp_lcd_touch_config_t *config,
                                        esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t handle = NULL;

    ESP_GOTO_ON_FALSE(io, ESP_ERR_INVALID_ARG, err, TAG,
                      "esp_lcd_panel_io_handle_t must not be NULL");
    ESP_GOTO_ON_FALSE(config, ESP_ERR_INVALID_ARG, err, TAG,
                      "esp_lcd_touch_config_t must not be NULL");

    handle = (esp_lcd_touch_handle_t)calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(handle, ESP_ERR_NO_MEM, err, TAG,
                      "No memory available for XPT2046 state");
    handle->io = io;
    handle->read_data = xpt2046_read_data;
    handle->get_xy = xpt2046_get_xy;
    handle->del = xpt2046_del;
    handle->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&handle->config, config, sizeof(esp_lcd_touch_config_t));

    // this is not yet supported by esp_lcd_touch.
    if (config->int_gpio_num != GPIO_NUM_NC)
    {
        ESP_GOTO_ON_FALSE(GPIO_IS_VALID_GPIO(config->int_gpio_num),
            ESP_ERR_INVALID_ARG, err, TAG, "Invalid GPIO Interrupt Pin");
        gpio_config_t cfg;
        memset(&cfg, 0, sizeof(gpio_config_t));
        esp_rom_gpio_pad_select_gpio(config->int_gpio_num);
        gpio_irq = config->int_gpio_num;
        cfg.pin_bit_mask = BIT64(config->int_gpio_num);
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLDOWN_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;

        // If the user has provided a callback routine for the interrupt enable
        // the interrupt mode on the negative edge.
        if (config->interrupt_callback)
        {
            cfg.intr_type = GPIO_INTR_NEGEDGE;
        }

        ESP_GOTO_ON_ERROR(gpio_config(&cfg), err, TAG,
                          "Configure GPIO for Interrupt failed");

        // Connect the user interrupt callback routine.
        if (config->interrupt_callback)
        {
            esp_lcd_touch_register_interrupt_callback(handle, config->interrupt_callback);
        }
    }

    //adding to define size buffer with overflow protection
    #if CONFIG_IDF_TARGET_ESP32S3
        DISP_BUF_SIZE = (((config->x_max)*40*3*8>(1<<18)) ? ((((1<<18)-1000)/8)/3) : (config->x_max * 40));
    #else
        DISP_BUF_SIZE = (((config->x_max)*40*3*8>(1<<24)) ? ((((1<<24)-1000)/8)/3) : (config->x_max * 40));
    #endif

err:
    if (ret != ESP_OK)
    {
        if (handle)
        {
            xpt2046_del(handle);
            handle = NULL;
        }
    }

    *out_touch = handle;

    return ret;
}

static esp_err_t xpt2046_del(esp_lcd_touch_handle_t tp)
{
    if (tp != NULL)
    {
        if (tp->config.int_gpio_num != GPIO_NUM_NC)
        {
            gpio_reset_pin(tp->config.int_gpio_num);
        }
    }
    free(tp);

    return ESP_OK;
}

static inline esp_err_t xpt2046_read_register(esp_lcd_touch_handle_t tp, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2] = {0, 0};
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(tp->io, reg, buf, 2), TAG, "XPT2046 read error!");
    *value = ((buf[0] << 8) | (buf[1]));
    return ESP_OK;
}

static void xpt2046_avg(uint16_t *x, uint16_t *y)
{
    /*Shift out the oldest data*/
    uint8_t i;
    for(i = XPT2046_AVG - 1; i > 0 ; i--) {
        avg_buf_x[i] = avg_buf_x[i - 1];
        avg_buf_y[i] = avg_buf_y[i - 1];
    }

    /*Insert the new point*/
    avg_buf_x[0] = *x;
    avg_buf_y[0] = *y;
    if(avg_last < XPT2046_AVG) avg_last++;

    /*Sum the x and y coordinates*/
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    for(i = 0; i < avg_last ; i++) {
        x_sum += avg_buf_x[i];
        y_sum += avg_buf_y[i];
    }

    /*Normalize the sums*/
    (*x) = (int32_t)x_sum / avg_last;
    (*y) = (int32_t)y_sum / avg_last;
}

static void first_set_buf_avg(uint16_t *x, uint16_t *y)
{
    for(uint8_t i = 0; i<XPT2046_AVG; i++) {
        avg_buf_x[i] = *x;
        avg_buf_y[i] = *y;
    }
}

static void clear_set_buf_avg()
{
    memset(avg_buf_x, 0, sizeof(avg_buf_x));
    memset(avg_buf_y, 0, sizeof(avg_buf_y));
}

static esp_err_t xpt2046_read_data(esp_lcd_touch_handle_t tp)
{
    uint16_t z1 = 0, z2 = 0, z = 0;
    uint32_t x = 0, y = 0;
    uint8_t point_count = 0;

    ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, Z_VALUE_1, &z1), TAG, "XPT2046 read error!");
    ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, Z_VALUE_2, &z2), TAG, "XPT2046 read error!");

    // Convert the received values into a Z value.
    z = (z1 >> 3) + (XPT2046_ADC_LIMIT - (z2 >> 3));

    uint8_t pin_irq = gpio_get_level(gpio_irq);
    // ESP_LOGI(TAG, "z level : %d, irq level : %d", z, pin_irq);

    // If the Z (pressure) exceeds the threshold it is likely the user has
    // pressed the screen, read in and average the positions.
    if (z >= CONFIG_XPT2046_Z_THRESHOLD && pin_irq!=1)
    {
        uint16_t discard_buf = 0;

        // read and discard a value as it is usually not reliable.
        ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, X_POSITION, &discard_buf),
                            TAG, "XPT2046 read error!");

        for (uint8_t idx = 0; idx < CONFIG_ESP_LCD_TOUCH_MAX_POINTS; idx++)
        {
            uint16_t x_temp = 0;
            uint16_t y_temp = 0;
            // Read X position and convert returned data to 12bit value
            ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, X_POSITION, &x_temp),
                                TAG, "XPT2046 read error!");
            // drop lowest three bits to convert to 12-bit position
            x_temp >>= 3;

#if CONFIG_XPT2046_CONVERT_ADC_TO_COORDS
            // Convert the raw ADC value into a screen coordinate and store it
            // for averaging.
            // x += ((x_temp / (double)XPT2046_ADC_LIMIT) * tp->config.x_max);
            x_temp = ((x_temp / (double)XPT2046_ADC_LIMIT) * tp->config.x_max);
#else
            // store the raw ADC values and let the user convert them to screen
            // coordinates.
            x += x_temp;
#endif // CONFIG_XPT2046_CONVERT_ADC_TO_COORDS

            // Read Y position and convert returned data to 12bit value
            ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, Y_POSITION, &y_temp),
                                TAG, "XPT2046 read error!");
            // drop lowest three bits to convert to 12-bit position
            y_temp >>= 3;

#if CONFIG_XPT2046_CONVERT_ADC_TO_COORDS
            // Convert the raw ADC value into a screen coordinate and store it
            // for averaging.
            // y += ((y_temp / (double)XPT2046_ADC_LIMIT) * tp->config.y_max);
            y_temp = ((y_temp / (double)XPT2046_ADC_LIMIT) * tp->config.y_max);
#else
            // store the raw ADC values and let the user convert them to screen
            // coordinates.
            y += y_temp;
#endif // CONFIG_XPT2046_CONVERT_ADC_TO_COORDS
            if(isFirst)
            {
                ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, X_POSITION, &x_temp),
                                TAG, "XPT2046 read error!");
                ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, Y_POSITION, &y_temp),
                                TAG, "XPT2046 read error!");
                x_temp >>= 3;
                y_temp >>= 3;
                x_temp = ((x_temp / (double)XPT2046_ADC_LIMIT) * tp->config.x_max);
                y_temp = ((y_temp / (double)XPT2046_ADC_LIMIT) * tp->config.y_max);
                isFirst = false;
                first_set_buf_avg(&x_temp, &y_temp);
            }
            xpt2046_avg(&x_temp, &y_temp);
            // this value is get by trial and error on LCD
            x = (uint16_t)scale(x_temp,18,218,10,230);
            y = (uint16_t)scale(y_temp,20,290,5,290);
        }

        // Average the accumulated coordinate data points.
        x /= CONFIG_ESP_LCD_TOUCH_MAX_POINTS;
        y /= CONFIG_ESP_LCD_TOUCH_MAX_POINTS;
        point_count = 1;
    }
    if (pin_irq == 1)
    {
        if(isFirst==false)
        {
            clear_set_buf_avg();
            isFirst = true;
        }
    }

    XPT2046_LOCK(&tp->data.lock);
    tp->data.coords[0].x = x;
    tp->data.coords[0].y = y;
    tp->data.coords[0].strength = z;
    tp->data.points = point_count;
    XPT2046_UNLOCK(&tp->data.lock);
    // printf("avg buf: %ld %ld %ld %ld\r\n", avg_buf_x[0], avg_buf_x[1], avg_buf_x[2], avg_buf_x[3]);
    // ESP_LOGI(TAG,"isFirst : %d", isFirst);

    return ESP_OK;
}

static bool xpt2046_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y,
                           uint16_t *strength, uint8_t *point_num,
                           uint8_t max_point_num)
{
    XPT2046_LOCK(&tp->data.lock);

    // Determine how many touch points that are available.
    if (tp->data.points > max_point_num)
    {
        *point_num = max_point_num;
    }
    else
    {
        *point_num = tp->data.points;
    }

    for (size_t i = 0; i < *point_num; i++)
    {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength)
        {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    // Invalidate stored touch data.
    tp->data.points = 0;

    XPT2046_UNLOCK(&tp->data.lock);

    if (*point_num)
    {
        ESP_LOGD(TAG, "Touch point: %dx%d", x[0], y[0]);
    }
    else
    {
        ESP_LOGV(TAG, "No touch points");
    }

    return (*point_num > 0);
}
