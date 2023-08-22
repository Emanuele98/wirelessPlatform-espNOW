#include "espnow.h"
#include "led_strip.h"
#include "aux_ctu_hw.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "MAIN";

static QueueHandle_t espnow_queue;
static uint8_t master_mac[ESP_NOW_ETH_ALEN] = { 0 };

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
esp_now_peer_num_t peer_num = {0, 0};

wpt_alert_payload_t alert_payload;
pad_status_t pad_status = PAD_DISCONNECTED;
TimerHandle_t dynamic_timer, alert_timer, connected_leds_timer, misaligned_leds_timer, charging_leds_timer, hw_readings_timer;

/* virtual switch for default led mode */
bool strip_enable;
/* virtual switch for the led misalignment mode */
bool strip_misalignment;
/* virtual switch foe the charging mode */
bool strip_charging;

/* Semaphore used to protect against I2C reading simultaneously */
SemaphoreHandle_t i2c_sem;
wpt_dynamic_payload_t dynamic_payload;


//ALERTS LIMITS
float OVERCURRENT;
float OVERVOLTAGE;
float OVERTEMPERATURE;
bool FOD_ACTIVE;


void espnow_data_prepare(espnow_data_t *buf, message_type type);


/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

static int count = 0;
void dynamic_timer_callback(void)
{
    count ++;
    //send them to master //
    espnow_data_t *buf = malloc(sizeof(espnow_data_t));
    if (buf == NULL) {
        ESP_LOGE(TAG, "Malloc  buffer fail");
        free(buf);
        return;
    }
    espnow_data_prepare(buf, ESPNOW_DATA_DYNAMIC);
    if (esp_now_send(master_mac, (uint8_t *) buf, sizeof(espnow_data_t)) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
    }

    free(buf);
}

void alert_timer_callback(void)
{
    espnow_data_t *buf = malloc(sizeof(espnow_data_t));
    if (buf == NULL) {
        ESP_LOGE(TAG, "Malloc  buffer fail");
        free(buf);
        return;
    }
    
    //check if values are in range
    if (dynamic_payload.voltage > OVERVOLTAGE)
        alert_payload.overvoltage = true;
    if (dynamic_payload.current > OVERCURRENT)
        alert_payload.overcurrent = true;
    if ((dynamic_payload.temp1 > OVERTEMPERATURE) || (dynamic_payload.temp2 > OVERTEMPERATURE))
        alert_payload.overtemperature = true;
    //todo: check FOD

    //* IF ANY ALERT IS ACTIVE, SEND ALERTS TO MASTER
    if (alert_payload.internal)
    {
        safely_switch_off();
        espnow_data_prepare(buf, ESPNOW_DATA_ALERT);
        if (esp_now_send(master_mac, (uint8_t *) buf, sizeof(espnow_data_t)) != ESP_OK) {
            ESP_LOGE(TAG, "Send error");
        }
    }

    free(buf);
}

static void esp_now_register_master(uint8_t *mac_addr, bool encrypt)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    if (encrypt == true)
    {
        peer->encrypt = true;
        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    } else {
        peer->encrypt = false;
    }
    memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);
}


/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void ESPNOW_SEND_CB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ID_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void ESPNOW_RECV_CB(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ID_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;

    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
uint8_t espnow_data_crc_control(uint8_t *data, uint16_t data_len)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return 1;
    }

    return 0;
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_data_t *buf, message_type type)
{
    //initiliaze buf to 0
    memset(buf, 0, sizeof(espnow_data_t));
    // esp NOW data
    buf->id = UNIT_ROLE;
    buf->type = type;
    
    switch (type)
    {
        case ESPNOW_DATA_BROADCAST:
            //nothing else to do
            break;
    
        case ESPNOW_DATA_LOCALIZATION:
            // on/off
            break;

        case ESPNOW_DATA_ALERT:
            // fill in alerts data
            buf->field_1 = alert_payload.overtemperature;
            buf->field_2 = alert_payload.overcurrent;
            buf->field_3 = alert_payload.overvoltage;
            buf->field_4 = alert_payload.FOD;

            break;
        
        case ESPNOW_DATA_DYNAMIC:
            // fill in dynamic data
            buf->field_1 = dynamic_payload.voltage;
            buf->field_2 = dynamic_payload.current;
            buf->field_3 = dynamic_payload.temp1;
            buf->field_4 = dynamic_payload.temp2;
            break;
        
        case ESPNOW_DATA_CONTROL:
            // only received - never sent
            break;

        default:
            ESP_LOGE(TAG, "Message type error: %d", type);
            break;
    }

    // CRC
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, sizeof(espnow_data_t));
}

static void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t addr_type;
    espnow_data_t *espnow_data = (espnow_data_t *)pvParameter;

    ESP_LOGI(TAG, "Start sending broadcast data");
    if (esp_now_send(broadcast_mac, (uint8_t *) espnow_data, sizeof(espnow_data_t)) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
    }

    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ID_ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                addr_type = IS_BROADCAST_ADDR(send_cb->mac_addr) ? true : false;
                esp_now_get_peer_num(&peer_num);

                ESP_LOGI(TAG, "Send data to "MACSTR". status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (addr_type)
                {
                    // Broadcast message will never return status 0 as it does not wait for ack
                    // if the master (encrypted) is registered, then stop sending broadcast data
                    if (peer_num.encrypt_num)
                    {
                        ESP_LOGI(TAG, "Stop sending broadcast data");
                        break;
                    }

                    /* Otherwise, Delay a while before sending the next data. */
                    vTaskDelay(BROADCAST_TIMEGAP);
                    
                    espnow_data_prepare(espnow_data, ESPNOW_DATA_BROADCAST);

                    /* Send the next data after the previous data is sent. */
                    if (esp_now_send(broadcast_mac, (uint8_t *) espnow_data, sizeof(espnow_data_t)) != ESP_OK) {
                        ESP_LOGE(TAG, "Send error");
                    }
                } 
                else
                {
                    if (send_cb->status != ESP_NOW_SEND_SUCCESS) 
                        ESP_LOGE(TAG, "ERROR SENDING DATA TO MASTER");

                    //check it was sent correctly
                    //todo: if status != 0, then resend data
                    //todo: check retransmission count
                }
                break;
            }
            case ID_ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                // Check CRC of received ESPNOW data.
                if(!espnow_data_crc_control(recv_cb->data, recv_cb->data_len))
                {
                    ESP_LOGE(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                    break;
                }

                espnow_data_t *recv_data = (espnow_data_t *)recv_cb->data;
                addr_type = recv_data->type;

                if (addr_type == ESPNOW_DATA_BROADCAST) 
                {
                    ESP_LOGI(TAG, "Receive broadcast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    //! ignore it as it comes from other peers
                }
                else if(addr_type == ESPNOW_DATA_CONTROL) 
                {
                    ESP_LOGI(TAG, "Receive control data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    //* Add MASTER 
                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) 
                    {
                        if (recv_data->field_1 > 1) //meaning the payload contains the alerts thresholds
                        {
                            ESP_LOGW(TAG, "Setting alerts limits");
                            //parse data to set local alerts limits
                            OVERCURRENT = recv_data->field_1;
                            OVERVOLTAGE = recv_data->field_2;
                            OVERTEMPERATURE = recv_data->field_3;
                            FOD_ACTIVE = recv_data->field_4;
                        }
                        else
                        {
                            memcpy(master_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            ESP_LOGW(TAG, "Add master "MACSTR" to peer list", MAC2STR(master_mac));
                            esp_now_register_master(master_mac, true);
                            if (dynamic_timer == NULL)
                            {
                                // freeRTOS timers
                                const int dyn_timegap = pdMS_TO_TICKS( (int) recv_data->field_3 );

                                dynamic_timer = xTimerCreate("dynamic_timer", dyn_timegap, pdTRUE, NULL, dynamic_timer_callback);
                                ESP_ERROR_CHECK( dynamic_timer == NULL ? ESP_FAIL : ESP_OK);
                                alert_timer = xTimerCreate("alert_timer", ALERT_TIMEGAP, pdTRUE, NULL, alert_timer_callback);
                                ESP_ERROR_CHECK( alert_timer == NULL ? ESP_FAIL : ESP_OK);
                                //!start sending measurements data to the master
                                if (xTimerStart(dynamic_timer, 0) != pdPASS) {
                                    ESP_LOGE(TAG, "Cannot start dynamic timer");
                                }
                                if (xTimerStart(alert_timer, 0) != pdPASS) {
                                    ESP_LOGE(TAG, "Cannot start alert timer");
                                }
                                pad_status = PAD_CONNECTED;
                                strip_misalignment = false;
                                strip_charging = false;
                                strip_enable = true;
                            }
                        }
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Master "MACSTR" already in peer list", MAC2STR(master_mac));
                        //TODO: handle the dynamic change of dynamic timer from field_3

                        //handle strip and switches
                        if (recv_data->field_1)
                        {
                            safely_enable_full_power();
                            strip_misalignment = false;
                            strip_enable = false;
                            strip_charging = true;
                        }
                        else if (recv_data->field_2)
                            {
                                //safely_enable_low_power(); //todo: reactivate low power!
                                safely_enable_full_power();
                            }
                            else
                            {
                                safely_switch_off();
                                strip_misalignment = false;
                                strip_enable = false;
                                strip_charging = false;
                            }

                        //todo: misalignment
                        //todo: fully charged
                    }
                }
                else 
                    ESP_LOGI(TAG, "Receive unexpected message type %d data from: "MACSTR"", addr_type, MAC2STR(recv_cb->mac_addr));
                
                free(recv_cb->data);
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }

    vQueueDelete(espnow_queue);
    vTaskDelete(NULL);
    free(espnow_data);
}

static esp_err_t espnow_init(void)
{
    //Create queue to process callbacks from espnow
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(ESPNOW_SEND_CB) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(ESPNOW_RECV_CB) );
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_register_master(broadcast_mac, false);

    espnow_data_t *buffer = malloc(sizeof(espnow_data_t));
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(buffer);
        return ESP_FAIL;
    }
    espnow_data_prepare(buffer, ESPNOW_DATA_BROADCAST);

    // create a task to handle espnow events
    xTaskCreate(espnow_task, "espnow_task", 2048, buffer, 4, NULL);

    return ESP_OK;
}

static void hw_init()
{
    esp_err_t err_code;

    install_strip(STRIP_PIN);
    ESP_LOGI(TAG, "LED strip initialized successfully");

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    //* init GPIOs
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //todo: how do we get FOD?

    safely_switch_off();

    /* Initialize I2C semaphore */
    i2c_sem = xSemaphoreCreateMutex();

    //* HW reading timer*/
    hw_readings_timer = xTimerCreate("hw_readings", HW_READINGS_TIMER_PERIOD, pdTRUE, NULL, hw_readings_timeout);
    if (xTimerStart(hw_readings_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Cannot start hw readings timer");
    }

    //* LED strip timers
    connected_leds_timer = xTimerCreate("connected_leds", CONNECTED_LEDS_TIMER_PERIOD, pdTRUE, NULL, connected_leds);
    misaligned_leds_timer = xTimerCreate("misaligned_leds", MISALIGNED_LEDS_TIMER_PERIOD, pdTRUE, NULL, misaligned_leds);
    charging_leds_timer = xTimerCreate("charging leds", CHARGING_LEDS_TIMER_PERIOD, pdTRUE, NULL, charging_state);

    if ( (connected_leds_timer == NULL) || (misaligned_leds_timer == NULL) || (charging_leds_timer == NULL))
    {
        ESP_LOGW(TAG, "Timers were not created successfully");
        return;
    }

    xTimerStart(connected_leds_timer, 10);
    xTimerStart(misaligned_leds_timer, 10);
    xTimerStart(charging_leds_timer, 10);

}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGW(TAG, "\n[APP] Free memory: %d bytes\n", (int) esp_get_free_heap_size());

    wifi_init();
    espnow_init();

    hw_init();

    //todo: switch safely off when master disappear
    //ESP_NOW_DEINIT()
    //WIFI_DEINIT()
}
