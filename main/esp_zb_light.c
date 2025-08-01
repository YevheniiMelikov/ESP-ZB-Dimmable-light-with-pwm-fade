#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "driver/ledc.h"
#include "string.h"
#include <driver/temperature_sensor.h>

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

static const char *TAG = "ESP_ZB_COLOR_DIMM_LIGHT";

/*——————————————————————— State ———————————————————————*/
typedef struct
{
    bool power;
    uint16_t color_temp;
    uint8_t brightness;
    uint16_t temp;
} light_state_t;

static const uint32_t MAX_DUTY = (1 << LEDC_DUTY_RES) - 1;
static const uint32_t FADE_TIME_MS = 300;
static const uint16_t DEFAULT_COLOR_TEMP = (COLOR_TEMP_PHYSICAL_MAX + COLOR_TEMP_PHYSICAL_MIN) / 2;
static const uint8_t DEFAULT_TARGET_BRIGHTNESS = 254;

static light_state_t s_state = {
    .power = true,
    .color_temp = DEFAULT_COLOR_TEMP,
    .brightness = DEFAULT_TARGET_BRIGHTNESS,
    .temp = 0,
};

/*——————— Zigbee reporting & binding ———————*/

static void config_reporting(uint16_t cluster_id, uint16_t attr_id,
                             esp_zb_zcl_attr_type_t type, void *report_change)
{
    esp_zb_zcl_config_report_cmd_t rc = {0};
    esp_zb_zcl_config_report_record_t rec = {
        .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
        .attributeID = attr_id,
        .attrType = type,
        .min_interval = 0,
        .max_interval = 3600,
        .reportable_change = report_change,
    };
    rc.zcl_basic_cmd.dst_addr_u.addr_short = esp_zb_get_short_address();
    rc.zcl_basic_cmd.src_endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT;
    rc.zcl_basic_cmd.dst_endpoint = 1;
    rc.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    rc.clusterID = cluster_id;
    rc.record_number = 1;
    rc.record_field = &rec;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&rc);
    esp_zb_lock_release();
}

void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0001,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
    };
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    memcpy(value_r->data_p, value, value_length);
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&cmd);
    esp_zb_lock_release();
}

void tempetature_task(void *pvParameters)
{

    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    while (true)
    {
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

        float temp = -100;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &temp));
        ESP_ERROR_CHECK(temperature_sensor_disable(temp_sensor));

        uint16_t temperature = (uint16_t)(temp * 100);
        s_state.temp = temperature;
        esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, (uint16_t *)&s_state.temp, true);
        reportAttribute(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature, 2);

        ESP_LOGI("TMP", "t(%i)", (int)s_state.temp);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

static void reporting_setup(void)
{
    config_reporting(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                     ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                     ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                     &s_state.power);

    config_reporting(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                     ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                     ESP_ZB_ZCL_ATTR_TYPE_U8,
                     &s_state.brightness);

    config_reporting(
        ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
        ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        &s_state.color_temp);

    config_reporting(
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        &s_state.temp);
}

/*———————————————— Driver implementation —————————————————*/

static IRAM_ATTR bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg)
{
    BaseType_t taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT)
    {
        SemaphoreHandle_t counting_sem = (SemaphoreHandle_t)user_arg;
        xSemaphoreGiveFromISR(counting_sem, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
}

typedef struct
{
    uint32_t warm;
    uint32_t cold;
} light_temp_t;

light_temp_t get_light_temp(uint16_t color_temp, uint8_t brightness)
{
    float ct = color_temp;
    if (ct < COLOR_TEMP_PHYSICAL_MIN)
        ct = COLOR_TEMP_PHYSICAL_MIN;
    if (ct > COLOR_TEMP_PHYSICAL_MAX)
        ct = COLOR_TEMP_PHYSICAL_MAX;
    float ratio = (ct - COLOR_TEMP_PHYSICAL_MIN) / (COLOR_TEMP_PHYSICAL_MAX - COLOR_TEMP_PHYSICAL_MIN);

    light_temp_t temp = {
        .cold = (1.0f - ratio) * brightness * MAX_DUTY / 0xFF,
        .warm = ratio * brightness * MAX_DUTY / 0xFF};

    return temp;
}

SemaphoreHandle_t custom_driver_init(void)
{
    gpio_reset_pin(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 1);

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    light_temp_t duty = get_light_temp(DEFAULT_COLOR_TEMP, DEFAULT_TARGET_BRIGHTNESS);

    const struct
    {
        int channel;
        gpio_num_t pin;
        uint32_t duty;
    } chs[] = {
        {.channel = LEDC_CHANNEL_WARM, .pin = WARM_PWM_GPIO, .duty = duty.warm},
        {.channel = LEDC_CHANNEL_COLD, .pin = COLD_PWM_GPIO, .duty = duty.cold},
    };
    for (size_t i = 0; i < sizeof(chs) / sizeof(chs[0]); i++)
    {
        ledc_channel_config_t ch = {
            .speed_mode = LEDC_SPEED_MODE,
            .channel = chs[i].channel,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = chs[i].pin,
            .duty = chs[i].duty,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }

    ledc_fade_func_install(0);

    SemaphoreHandle_t counting_sem = xSemaphoreCreateCounting(2, 0);
    ledc_cbs_t callbacks = {
        .fade_cb = cb_ledc_fade_end_event};
    ledc_cb_register(LEDC_SPEED_MODE, LEDC_CHANNEL_WARM, &callbacks, (void *)counting_sem);
    ledc_cb_register(LEDC_SPEED_MODE, LEDC_CHANNEL_COLD, &callbacks, (void *)counting_sem);

    return counting_sem;
}

void pwm_task(void *pvParameters)
{
    SemaphoreHandle_t counting_sem = custom_driver_init();
    uint16_t color_temp = DEFAULT_COLOR_TEMP;
    uint8_t brightness = DEFAULT_TARGET_BRIGHTNESS;
    bool pwr = s_state.power;

    while (1)
    {
        if (s_state.power)
        {
            gpio_set_level(RELAY_GPIO, 1);
            if (color_temp != s_state.color_temp || brightness != s_state.brightness)
            {
                ESP_LOGI(TAG, "adjusting pwm");
                color_temp = s_state.color_temp;
                brightness = s_state.brightness;

                light_temp_t duty = get_light_temp(color_temp, brightness);
                ledc_set_fade_with_time(LEDC_SPEED_MODE, LEDC_CHANNEL_WARM, duty.warm, FADE_TIME_MS);
                ledc_set_fade_with_time(LEDC_SPEED_MODE, LEDC_CHANNEL_COLD, duty.cold, FADE_TIME_MS);
                ledc_fade_start(LEDC_SPEED_MODE, LEDC_CHANNEL_WARM, LEDC_FADE_NO_WAIT);
                ledc_fade_start(LEDC_SPEED_MODE, LEDC_CHANNEL_COLD, LEDC_FADE_NO_WAIT);

                xSemaphoreTake(counting_sem, portMAX_DELAY);
                xSemaphoreTake(counting_sem, portMAX_DELAY);

                ESP_LOGI(TAG, "adjusting pwm is done");
            }
        }
        else if (pwr != s_state.power)
        {
            ESP_LOGI(TAG, "powering off");
            ledc_set_fade_with_time(LEDC_SPEED_MODE, LEDC_CHANNEL_WARM, 0, FADE_TIME_MS);
            ledc_set_fade_with_time(LEDC_SPEED_MODE, LEDC_CHANNEL_COLD, 0, FADE_TIME_MS);
            ledc_fade_start(LEDC_SPEED_MODE, LEDC_CHANNEL_WARM, LEDC_FADE_NO_WAIT);
            ledc_fade_start(LEDC_SPEED_MODE, LEDC_CHANNEL_COLD, LEDC_FADE_NO_WAIT);

            xSemaphoreTake(counting_sem, portMAX_DELAY);
            xSemaphoreTake(counting_sem, portMAX_DELAY);

            gpio_set_level(RELAY_GPIO, 0);

            ESP_LOGI(TAG, "powering off is done");
        }
        if (pwr != s_state.power)
        {
            pwr = s_state.power;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

esp_err_t custom_driver_set_level(uint8_t level)
{
    s_state.brightness = level;
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, (uint8_t *)&s_state.brightness, true);
    reportAttribute(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level, 1);
    ESP_LOGI(TAG, "Set target brightness %d", level);
    return ESP_OK;
}

esp_err_t custom_driver_set_color(uint16_t color_temp)
{
    s_state.color_temp = color_temp;
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, (uint16_t *)&s_state.color_temp, true);
    reportAttribute(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_temp, 2);
    ESP_LOGI(TAG, "Set color %d", color_temp);
    return ESP_OK;
}

esp_err_t custom_driver_set_power(bool on)
{
    s_state.power = on;
    esp_zb_zcl_set_attribute_val(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, (bool *)&s_state.power, true);
    reportAttribute(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on, 1);
    ESP_LOGI(TAG, "Set power %d", on);
    return ESP_OK;
}

/*——————— Zigbee app signal handler ———————*/

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
                        , TAG, "Failed to start commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    uint32_t *sig = signal->p_app_signal;
    esp_zb_app_signal_type_t type = *sig;

    switch (type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (signal->esp_err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver init");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (signal->esp_err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Joined network, scheduling reporting setup");
            esp_zb_scheduler_alarm((esp_zb_callback_t)reporting_setup, 0, 2000);
            xTaskCreate(tempetature_task, "Temperature_task", 4096, NULL, 5, NULL);
        }
        else
        {
            ESP_LOGI(TAG, "Steering failed, retrying");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (signal->esp_err_status == ESP_OK)
        {
            if (*(uint8_t *)esp_zb_app_signal_get_params(sig))
            {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(sig));
            }
            else
            {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;

    default:
        break;
    }
}

/*——————— ZCL attribute handler ———————*/

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    ESP_RETURN_ON_FALSE(msg, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        msg->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(0x%x), cluster(0x%x), attribute(0x%x), data size(%d)", msg->info.dst_endpoint, msg->info.cluster,
             msg->attribute.id, msg->attribute.data.size);

    if (!msg)
    {
        return ESP_FAIL;
    }

    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
    {
        custom_driver_set_power(*(bool *)msg->attribute.data.value);
    }
    else if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
    {
        custom_driver_set_level(*(uint8_t *)msg->attribute.data.value);
    }
    else if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL)
    {
        if (msg->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
        {
            custom_driver_set_color(*(uint16_t *)msg->attribute.data.value);
        }
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id, const void *msg)
{
    if (cb_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID)
    {
        return zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)msg);
    }
    return ESP_OK;
}

/*——————— Main task & app_main ———————*/

static void esp_zb_task(void *pv)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // ------------------------------ Cluster BASIC ------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_AF_NODE_POWER_SOURCE_CONSTANT_POWER,
    };
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;

    uint8_t ManufacturerName[] = {
        9,
        'E',
        'S',
        'P',
        'R',
        'E',
        'S',
        'S',
        'I',
        'F',
    }; // warning: this is in format {length, 'string'} :
    uint8_t ModelIdentifier[] = {
        17,
        'e',
        's',
        'p',
        '3',
        '2',
        'c',
        '6',
        '_',
        'c',
        'c',
        't',
        '_',
        'l',
        'i',
        'g',
        'h',
        't',
    };
    uint8_t DateCode[] = {8, '2', '0', '2', '5', '0', '7', '3', '0'};
    uint8_t SWVersion[] = {2, 'v', '1'};
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, &SWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode);

    // ------------------------------ Cluster IDENTIFY ------------------------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    // ------------------------------ Cluster LIGHT ------------------------------
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    // ===============================Color Cluster for test==============================
    // It works with HA!!!
    // Use esp_zb_color_control_cluster_add_attr for this
    // esp_zb_color_control_cluster_create
    esp_zb_color_cluster_cfg_t esp_zb_color_cluster_cfg = {
        .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,                         /*!<  The current value of the normalized chromaticity value x */
        .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,                         /*!<  The current value of the normalized chromaticity value y */
        .color_mode = 0x0002,                                                              /*!<  The mode which attribute determines the color of the device */
        .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,                         /*!<  The bitmap determines behavior of some cluster commands */
        .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE, /*!<  The enhanced-mode which attribute determines the color of the device */
        .color_capabilities = 0x0010,                                                      /*!<  Specifying the color capabilities of the device support the color control cluster */
    };
    esp_zb_attribute_list_t *esp_zb_color_cluster = esp_zb_color_control_cluster_create(&esp_zb_color_cluster_cfg);

    uint16_t color_attr = s_state.color_temp;
    uint16_t min_temp = COLOR_TEMP_PHYSICAL_MIN; // ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_DEFAULT_VALUE;
    uint16_t max_temp = COLOR_TEMP_PHYSICAL_MAX; // ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_DEFAULT_VALUE;
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_attr);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &min_temp);
    esp_zb_color_control_cluster_add_attr(esp_zb_color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &max_temp);

    // ============================= Level Cluster for test =============================
    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
    uint8_t level = 50;
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level);

    // ------------------------------ Cluster Temperature ------------------------------
    esp_zb_temperature_meas_cluster_cfg_t temperature_meas_cfg = {
        .measured_value = 0xFFFF,
        .min_value = -50,
        .max_value = 100,
    };
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster = esp_zb_temperature_meas_cluster_create(&temperature_meas_cfg);

    // ------------------------------ Create cluster list ------------------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // esp_zb_cluster_list_update_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_update_color_control_cluster(esp_zb_cluster_list, esp_zb_color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // ------------------------------ Create endpoint list ------------------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    // Create struct for esp_zb_ep_list_add_ep function from newest library
    // Warning! BitFields?!
    esp_zb_endpoint_config_t zb_endpoint_config = {
        .endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,      /*!< Endpoint */
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,         /*!< Application profile identifier */
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, /*!< Application device identifier */
        .app_device_version = 4,                           /*!< Application device version */
    };
    // For old library
    // esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);

    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, zb_endpoint_config);

    // ------------------------------ Register Device ------------------------------
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&(esp_zb_platform_config_t){
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    }));

    xTaskCreate(pwm_task, "pwm_task", 4096, NULL, 5, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
