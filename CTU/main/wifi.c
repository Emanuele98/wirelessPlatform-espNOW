#include "wifi.h"

static const char *TAG = "WIFI";

bool MQTT_ACTIVE = false;
static bool update = false;
time_t now;
struct tm info;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
esp_mqtt_client_handle_t client;

/* LIST OF TOPICS FOR MQTT*/
 //* TX
const char tx_voltage[4][80] =          {"warwicktrial/ctu/pad1/sensors/voltage", "warwicktrial/ctu/pad2/sensors/voltage",
                                         "warwicktrial/ctu/pad3/sensors/voltage", "warwicktrial/ctu/pad4/sensors/voltage"};
const char tx_current[4][80] =          {"warwicktrial/ctu/pad1/sensors/current", "warwicktrial/ctu/pad2/sensors/current",
                                         "warwicktrial/ctu/pad3/sensors/current", "warwicktrial/ctu/pad4/sensors/current"};
const char tx_temp1[4][80] =            {"warwicktrial/ctu/pad1/sensors/temperature1", "warwicktrial/ctu/pad2/sensors/temperature1",
                                         "warwicktrial/ctu/pad3/sensors/temperature1", "warwicktrial/ctu/pad4/sensors/temperature1"};
const char tx_temp2[4][80] =            {"warwicktrial/ctu/pad1/sensors/temperature2", "warwicktrial/ctu/pad2/sensors/temperature2",
                                         "warwicktrial/ctu/pad3/sensors/temperature2", "warwicktrial/ctu/pad4/sensors/temperature2"};
const char tx_power[4][80] =            {"warwicktrial/ctu/pad1/sensors/power", "warwicktrial/ctu/pad2/sensors/power",
                                         "warwicktrial/ctu/pad3/sensors/power", "warwicktrial/ctu/pad4/sensors/power"};
const char tx_efficiency[4][80] =       {"warwicktrial/ctu/pad1/sensors/efficiency", "warwicktrial/ctu/pad2/sensors/efficiency",
                                         "warwicktrial/ctu/pad3/sensors/efficiency", "warwicktrial/ctu/pad4/sensors/efficiency"};
const char tx_fod[4][80] =              {"warwicktrial/ctu/pad1/alerts/fod", "warwicktrial/ctu/pad2/alerts/fod",
                                         "warwicktrial/ctu/pad3/alerts/fod", "warwicktrial/ctu/pad4/alerts/fod"};
const char tx_overvoltage[4][80] =      {"warwicktrial/ctu/pad1/alerts/overvoltage", "warwicktrial/ctu/pad2/alerts/overvoltage",
                                         "warwicktrial/ctu/pad3/alerts/overvoltage", "warwicktrial/ctu/pad4/alerts/overvoltage"};
const char tx_overcurrent[4][80] =      {"warwicktrial/ctu/pad1/alerts/overcurrent", "warwicktrial/ctu/pad2/alerts/overcurrent",
                                         "warwicktrial/ctu/pad3/alerts/overcurrent", "warwicktrial/ctu/pad4/alerts/overcurrent"};
const char tx_overtemperature[4][80] =  {"warwicktrial/ctu/pad1/alerts/overtemperature", "warwicktrial/ctu/pad2/alerts/overtemperature",
                                         "warwicktrial/ctu/pad3/alerts/overtemperature", "warwicktrial/ctu/pad4/alerts/overtemperature"};
const char tx_status[4][80] =           {"warwicktrial/ctu/pad1/status", "warwicktrial/ctu/pad2/status",
                                         "warwicktrial/ctu/pad3/status", "warwicktrial/ctu/pad4/status"};
const char tx_scooter[4][80] =          {"warwicktrial/ctu/pad1/scooter", "warwicktrial/ctu/pad2/scooter",
                                         "warwicktrial/ctu/pad3/scooter", "warwicktrial/ctu/pad4/scooter"};
 //* RX
const char rx_voltage[4][80] =          {"warwicktrial/cru/scooter3PAU/sensors/voltage", "warwicktrial/cru/scooterISIN/sensors/voltage",
                                         "warwicktrial/cru/scooterEUUP/sensors/voltage", "warwicktrial/cru/scooter6666/sensors/voltage"};
const char rx_current[4][80] =          {"warwicktrial/cru/scooter3PAU/sensors/current", "warwicktrial/cru/scooterISIN/sensors/current",
                                         "warwicktrial/cru/scooterEUUP/sensors/current", "warwicktrial/cru/scooter6666/sensors/current"};
const char rx_temp[4][80] =             {"warwicktrial/cru/scooter3PAU/sensors/temperature", "warwicktrial/cru/scooterISIN/sensors/temperature",
                                         "warwicktrial/cru/scooterEUUP/sensors/temperature", "warwicktrial/cru/scooter6666/sensors/temperature"};
const char rx_power[4][80] =            {"warwicktrial/cru/scooter3PAU/sensors/power", "warwicktrial/cru/scooterISIN/sensors/power",
                                         "warwicktrial/cru/scooterEUUP/sensors/power", "warwicktrial/cru/scooter6666/sensors/power"};
const char rx_charge_complete[4][80] =  {"warwicktrial/cru/scooter3PAU/alerts/chargecomplete", "warwicktrial/cru/scooterISIN/alerts/chargecomplete",
                                         "warwicktrial/cru/scooterEUUP/alerts/chargecomplete", "warwicktrial/cru/scooter6666/alerts/chargecomplete"};
const char rx_overvoltage[4][80] =      {"warwicktrial/cru/scooter3PAU/alerts/overvoltage", "warwicktrial/cru/scooterISIN/alerts/overvoltage",
                                         "warwicktrial/cru/scooterEUUP/alerts/overvoltage", "warwicktrial/cru/scooter6666/alerts/overvoltage"};
const char rx_overcurrent[4][80] =      {"warwicktrial/cru/scooter3PAU/alerts/overcurrent", "warwicktrial/cru/scooterISIN/alerts/overcurrent",
                                         "warwicktrial/cru/scooterEUUP/alerts/overcurrent", "warwicktrial/cru/scooter6666/alerts/overcurrent"};
const char rx_overtemperature[4][80] =  {"warwicktrial/cru/scooter3PAU/alerts/overtemperature", "warwicktrial/cru/scooterISIN/alerts/overtemperature",
                                         "warwicktrial/cru/scooterEUUP/alerts/overtemperature", "warwicktrial/cru/scooter6666/alerts/overtemperature"};

  //* DEBUG
const char debug[80] = {"warwicktrial/debug"};


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Callback function which is called once the Network Time Protocol is synced
 * 
 */
static void sntp_callback(struct timeval *tv)
{
    update = true;
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, (int) event_id);
    esp_mqtt_event_handle_t event = event_data;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        break;
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        MQTT_ACTIVE = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_ACTIVE = false;
        // force reconnection
        esp_mqtt_client_reconnect(client);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);

        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        //ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        //.cert_pem = (const char *)mqtt_eclipse_org_pem_start,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

/**
 * @brief Handler for WiFi and IP events
 * 
 */

static void wifi_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        // CONNECTION DONE - START MQTT
        mqtt_app_start();
    } else
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    }
}


/* WiFi should start before using ESPNOW */
//? with wifi connection - esp now and wifi

void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta(); // create default wifi station
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    ESP_ERROR_CHECK( esp_wifi_start());


    // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    //   number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually happened.
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID: %s", CONFIG_WIFI_SSID);
        
        //! print the primary and secondary channels (THE CHANNEL IS DECIDED BY THE ROUTER)
        //uint8_t primary, secondary;
        //esp_wifi_get_channel(&primary, &secondary);
        //ESP_LOGI(TAG, "primary channel: %d, secondary channel: %d", primary, secondary);    

        //*wait for mqtt to connect
        while (!MQTT_ACTIVE) {}

        //*Simple Network Time Protocol
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_set_time_sync_notification_cb(sntp_callback);
        sntp_init();

        // wait for time to be set
        while(!update){}

        // Set timezone to Eastern Standard Time and print local time
        // with daylight saving
        //setenv("TZ", "GMTGMT-1,M3.4.0/01,M10.4.0/02", 1);
        //without
        setenv("TZ", "GMTGMT-1", 1);
        tzset();
        time(&now);
        localtime_r(&now, &info);
        ESP_LOGE(TAG, "Time is %s", asctime(&info));
        
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s", CONFIG_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler);
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler);
    vEventGroupDelete(s_wifi_event_group);

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

void mqtt_unit_reset(uint8_t id)
{
    if (MQTT_ACTIVE)
    {
        if (id < NUMBER_TX)
        {
            esp_mqtt_client_publish(client, tx_voltage[id-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, tx_current[id-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, tx_temp1[id-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, tx_temp2[id-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, tx_power[id-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, tx_efficiency[id-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, tx_status[id-1], "0", 0, MQTT_QoS, 0);
        }
        else
        {
            esp_mqtt_client_publish(client, rx_voltage[id-NUMBER_TX-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, rx_current[id-NUMBER_TX-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, rx_temp[id-NUMBER_TX-1], "0", 0, MQTT_QoS, 0);
            esp_mqtt_client_publish(client, rx_power[id-NUMBER_TX-1], "0", 0, MQTT_QoS, 0);
        }
    }

}
