#include "main.h"

#include "esp_check.h"
#include "string.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "ESP_ZB_TEST_BENCH";

static void start_heart_beat_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &heart_beat_timer_callback,
        .name = "heart_beat_timer"};

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ESP_BINARY_HB_UPDATE_INTERVAL));
}

static void start_analog_in_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &analog_in_timer_callback,
        .name = "analog_input_timer"};

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ESP_AI_UPDATE_INTERVAL));
}

void report_heart_beat_attr(uint8_t ep)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID;
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = ep;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send HB 'report attributes' command");
}

void report_analog_in_attr(uint8_t ep)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID;
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = ep;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send AI 'report attributes' command");
}

void esp_app_heart_beat_handler()
{

    esp_zb_zcl_attr_t *heart_beat = esp_zb_zcl_get_attribute(
        HA_ESP_HB_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID);
    bool cur_val = *(bool *)heart_beat->data_p;

    ESP_LOGI(TAG, "Heart Beat: %s", cur_val ? "On" : "Off");

    cur_val = cur_val ^ 1;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_HB_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &cur_val, false);
    esp_zb_lock_release();
}

void esp_app_analog_in_handler()
{

    esp_zb_zcl_attr_t *analog_in = esp_zb_zcl_get_attribute(
        HA_ESP_ANALOG_IN_EP,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID);
    float cur_val = *(float *)analog_in->data_p;

    

    cur_val += 1;
    if (cur_val > 100)
    {
        cur_val = 0;
    }


    ESP_LOGI(TAG, "AI : %f", cur_val);

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_ANALOG_IN_EP,
                                 ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &cur_val, false);
    esp_zb_lock_release();
}

static void heart_beat_timer_callback(void *arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Heart Beat timer called, time since boot: %lld us", time_since_boot);
    esp_app_heart_beat_handler();
    report_heart_beat_attr(HA_ESP_HB_ENDPOINT);
}

static void analog_in_timer_callback(void *arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Analog Input timer called, time since boot: %lld us", time_since_boot);
    esp_app_analog_in_handler();
    report_analog_in_attr(HA_ESP_ANALOG_IN_EP);
}
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    ESP_LOGI(TAG, "Starting Timers");
    start_heart_beat_timer();
    start_analog_in_timer();
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool switch_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_SW_IN_EP) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                switch_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : switch_state;
                ESP_LOGI(TAG, "Switch set to %s", switch_state ? "On" : "Off");
                //GPIO logic here
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Received Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *add_basic_and_identify_clusters_create(
    esp_zb_identify_cluster_cfg_t *identify_cluster_cfg,
    esp_zb_basic_cluster_cfg_t *basic_cluster_cfg)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(basic_cluster_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(identify_cluster_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    return cluster_list;
}

static void binary_sensor_ep_create(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, esp_zb_cluster_list_t *binary_sensor_cluster_list)
{
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, binary_sensor_cluster_list, endpoint_config);
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    };

    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE};

    esp_zb_binary_input_cluster_cfg_t heart_beat_sensor_cfg = {
        .out_of_service = ESP_ZB_ZCL_BINARY_INPUT_OUT_OF_SERVICE_DEFAULT_VALUE,
        .status_flags = ESP_ZB_ZCL_BINARY_INPUT_STATUS_FLAG_DEFAULT_VALUE};

    esp_zb_cluster_list_t *heart_beat_cluster_list = add_basic_and_identify_clusters_create(
        &identify_cluster_cfg,
        &basic_cluster_cfg);

    esp_zb_attribute_list_t *hb_attr_list = esp_zb_binary_input_cluster_create(&heart_beat_sensor_cfg);
    bool bin_val = 0x00;
    esp_zb_binary_input_cluster_add_attr(hb_attr_list, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, HB_IDENTIFIER);
    esp_zb_binary_input_cluster_add_attr(hb_attr_list, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &bin_val);

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(heart_beat_cluster_list, hb_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    binary_sensor_ep_create(ep_list, HA_ESP_HB_ENDPOINT, heart_beat_cluster_list);

    float cur_val = 0x00;
    esp_zb_analog_input_cluster_cfg_t analog_in_cfg = {
        .out_of_service = ESP_ZB_ZCL_ANALOG_INPUT_OUT_OF_SERVICE_DEFAULT_VALUE,
        .status_flags = ESP_ZB_ZCL_ANALOG_INPUT_STATUS_FLAG_DEFAULT_VALUE,
        .present_value = cur_val};

    esp_zb_cluster_list_t *analog_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *analog_in_cluster = esp_zb_analog_input_cluster_create(&analog_in_cfg);
    float min_ai_val = 0x00; 
    float max_ai_val = 100;
    uint16_t no_units = 0x005f;
    ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(analog_in_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, AI_DESCRIPTION));
    ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(analog_in_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID, &max_ai_val));
    ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(analog_in_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID, &min_ai_val));
    ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(analog_in_cluster, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &no_units));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(analog_cluster_list, analog_in_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_endpoint_config_t ai_endpoint_config = {
        .endpoint = HA_ESP_ANALOG_IN_EP,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, analog_cluster_list, ai_endpoint_config);


    //On Off 

    uint16_t sw_cur_val = 0x00;
    esp_zb_on_off_cluster_cfg_t sw_in_cfg = {
        .on_off = sw_cur_val
        };

    esp_zb_cluster_list_t *sw_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *sw_in_cluster = esp_zb_on_off_cluster_create(&sw_in_cfg);
   
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(sw_cluster_list, sw_in_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_endpoint_config_t sw_endpoint_config = {
        .endpoint = HA_ESP_SW_IN_EP,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID, 
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, sw_cluster_list, sw_endpoint_config);

    esp_zb_device_register(ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);
    
    esp_zb_zcl_reporting_info_t ai_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_ANALOG_IN_EP,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .dst.endpoint = 1,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u8 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&ai_reporting_info));

    esp_zb_zcl_reporting_info_t hb_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_HB_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .dst.endpoint = 1,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u8 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&hb_reporting_info));

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}