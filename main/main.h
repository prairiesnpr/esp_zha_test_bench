#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define HA_ESP_HB_ENDPOINT              1
#define HA_ESP_BIN_OUT_EP               2
#define HA_ESP_ANALOG_IN_EP             3
#define HA_ESP_SW_IN_EP                 4

#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

#define ESP_BINARY_HB_UPDATE_INTERVAL (30000000)     /* Local sensor update interval (microsecond) */
#define ESP_AI_UPDATE_INTERVAL (5000000)     /* Local sensor update interval (microsecond) */




/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME               "\x0A""iSilentLLC"
#define MODEL_IDENTIFIER                "\x09""Test Mule"
#define HB_IDENTIFIER                   "\x0A""Heart Beat"
#define AI_DESCRIPTION                  "\x0E""Analog In Test"
#define MS_DESCRIPTION                  "\x0E""MultValue Test"
#define T_BIN_DESC                      "\x07""Bin Out"
#define T_BIN_ACTIVE                    "\x06""Active"
#define T_BIN_INACTIVE                  "\x08""Inactive" 
#define MS_STATES                       "\0x42""\x06""State1""\x06""State2""\x06""State3" 


#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                   \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

static void heart_beat_timer_callback(void* arg);
static void analog_in_timer_callback(void* arg);
