/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_color_dimmable_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef ESP_ZB_LIGHT_H_
#define ESP_ZB_LIGHT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*——— Includes ———*/
#include "esp_zigbee_core.h"
#include "zcl_utility.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/*——— Hardware I/O ———*/
#define RELAY_GPIO GPIO_NUM_19
#define WARM_PWM_GPIO GPIO_NUM_4
#define COLD_PWM_GPIO GPIO_NUM_5

/*——— LEDC (PWM) Configuration ———*/
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT /* 0–8191 */
#define LEDC_FREQUENCY_HZ 4000          /* 4 kHz */

#define LEDC_CHANNEL_WARM LEDC_CHANNEL_0
#define LEDC_CHANNEL_COLD LEDC_CHANNEL_1

/* CCT range in mireds (153 ≃ 6500 K, 500 ≃ 2000 K) */
#define COLOR_TEMP_PHYSICAL_MIN 153
#define COLOR_TEMP_PHYSICAL_MAX 500

/*——— Zigbee Device Configuration ———*/
#define HA_COLOR_DIMMABLE_LIGHT_ENDPOINT 10
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

#define MAX_CHILDREN 10
#define INSTALLCODE_POLICY_ENABLE false

#define ESP_ZB_ZR_CONFIG()                                  \
    {                                                       \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,           \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,   \
        .nwk_cfg.zczr_cfg = {.max_children = MAX_CHILDREN } \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
    {                                 \
        .radio_mode = ZB_RADIO_MODE_NATIVE}

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
    {                                \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE}

/*——— ZCL Basic Cluster Identity ———*/
/* CharString: length byte + string bytes */
#define ESP_MANUFACTURER_NAME "\x09" \
                              "ESPRESSIF"
#define ESP_MODEL_IDENTIFIER "\x11" \
                             "esp32c6_cct_light"

    // /*——— Driver Prototypes ———*/

    // /**
    //  * @brief   Initialize relay and LEDC peripherals.
    //  */
    // esp_err_t custom_driver_init(void);

    // /**
    //  * @brief   Turn light on/off (with optional fade support).
    //  * @param   on  true=on, false=off
    //  */
    // esp_err_t custom_driver_set_power(bool on);

    // /**
    //  * @brief   Set color (CIE X coordinate, 0–65535).
    //  */
    // esp_err_t custom_driver_set_color(uint16_t color_x);

    // /**
    //  * @brief   Set brightness level (0–254).
    //  */
    // esp_err_t custom_driver_set_level(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif // ESP_ZB_LIGHT_H_
