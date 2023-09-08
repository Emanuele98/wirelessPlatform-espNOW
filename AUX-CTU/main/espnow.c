#include "espnow.h"

static const char *TAG = "ESP-NOW";

static SemaphoreHandle_t send_semaphore = NULL;
static QueueHandle_t espnow_queue;
static uint8_t master_mac[ESP_NOW_ETH_ALEN] = {0};

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_num_t peer_num = {0, 0};

message_type last_msg_type;
pad_status_t pad_status = PAD_DISCONNECTED;
TimerHandle_t dynamic_timer, alert_timer;

/* virtual switch for default led mode */
bool strip_enable;
/* virtual switch for the led misalignment mode */
bool strip_misalignment;
/* virtual switch foe the charging mode */
bool strip_charging;

/* Semaphore used to protect against I2C reading simultaneously */
wpt_dynamic_payload_t dynamic_payload;
wpt_alert_payload_t alert_payload;
static bool alert_sent = false;

static uint8_t comms_fail = 0;

// ALERTS LIMITS
float OVERCURRENT;
float OVERVOLTAGE;
float OVERTEMPERATURE;
bool FOD_ACTIVE;

static uint8_t alert_count = 0;
float voltage_window[AVG_ALERT_WINDOW], current_window[AVG_ALERT_WINDOW], temp1_window[AVG_ALERT_WINDOW], temp2_window[AVG_ALERT_WINDOW];

static void espnow_data_prepare(espnow_data_t *buf, message_type type);

/* WiFi should start before using ESPNOW */
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

static void dynamic_timer_callback(void)
{
    // send them to master //
    espnow_data_t *buf = malloc(sizeof(espnow_data_t));
    if (buf == NULL)
    {
        ESP_LOGE(TAG, "Malloc  buffer fail");
        free(buf);
        return;
    }
    if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
    {
        espnow_data_prepare(buf, ESPNOW_DATA_DYNAMIC);
        esp_now_send(master_mac, (uint8_t *)buf, sizeof(espnow_data_t));
    }
    else
        ESP_LOGE(TAG, "Could not take send semaphore");

    free(buf);
}

static void alert_timer_callback(void)
{
    espnow_data_t *buf = malloc(sizeof(espnow_data_t));
    if (buf == NULL)
    {
        ESP_LOGE(TAG, "Malloc  buffer fail");
        free(buf);
        return;
    }

    voltage_window[alert_count] = dynamic_payload.voltage;
    current_window[alert_count] = dynamic_payload.current;
    temp1_window[alert_count] = dynamic_payload.temp1;
    temp2_window[alert_count] = dynamic_payload.temp2;

    // alert count keeps going between 0 and AVG_ALERT_WINDOW - 1
    alert_count = (alert_count + 1) % AVG_ALERT_WINDOW;

    // never send an alert for the first (AVG_ALERT_WINDOW - 1) checks
    if (voltage_window[AVG_ALERT_WINDOW - 1] == 0)
        return;

    // calculate average values
    float avg_voltage = 0, avg_current = 0, avg_temp1 = 0, avg_temp2 = 0;
    for (int i = 0; i < AVG_ALERT_WINDOW; i++)
    {
        avg_voltage += voltage_window[i];
        avg_current += current_window[i];
        avg_temp1 += temp1_window[i];
        avg_temp2 += temp2_window[i];
    }
    avg_voltage /= AVG_ALERT_WINDOW;
    avg_current /= AVG_ALERT_WINDOW;
    avg_temp1 /= AVG_ALERT_WINDOW;
    avg_temp2 /= AVG_ALERT_WINDOW;

    // check if values are in range
    if (avg_voltage > OVERVOLTAGE)
        alert_payload.overvoltage = 1;
    if (avg_current > OVERCURRENT)
        alert_payload.overcurrent = 1;
    if ((avg_temp1 > OVERTEMPERATURE || avg_temp2 > OVERTEMPERATURE))
        alert_payload.overtemperature = 1;
    // todo: check FOD

    //* IF ANY ALERT IS ACTIVE, SEND ALERTS TO MASTER
    //alert_sent needed because this fast timer might have 1-2 cycles left before being actually stopped
    if ((alert_payload.internal) && (!alert_sent))
    {
        ESP_LOGE(TAG, "ALERTS ACTIVE");
        alert_sent = true;
        pad_status = PAD_ALERT;
        // locally switch off
        safely_switch_off();
        // STOP TIMERS
        xTimerStop(dynamic_timer, 0);
        xTimerStop(alert_timer, 0);
        // DELETE TIMERS
        xTimerDelete(dynamic_timer, 0);
        xTimerDelete(alert_timer, 0);
        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
        {
            espnow_data_prepare(buf, ESPNOW_DATA_ALERT);
            esp_now_send(master_mac, (uint8_t *)buf, sizeof(espnow_data_t));
        }
        else
            ESP_LOGE(TAG, "Could not take send semaphore");
    }

    free(buf);
}

static void esp_now_register_master(uint8_t *mac_addr, bool encrypt)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    if (encrypt == true)
    {
        peer->encrypt = true;
        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    }
    else
    {
        peer->encrypt = false;
    }
    memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void ESPNOW_SEND_CB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ID_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }

    // give back the semaphore if status is successfull
    if (status == ESP_NOW_SEND_SUCCESS)
        xSemaphoreGive(send_semaphore);
}

static void ESPNOW_RECV_CB(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ID_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;

    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
uint8_t espnow_data_crc_control(uint8_t *data, uint16_t data_len)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t))
    {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc)
    {
        return 1;
    }

    return 0;
}

/* Prepare ESPNOW data to be sent. */
static void espnow_data_prepare(espnow_data_t *buf, message_type type)
{
    // initiliaze buf to 0
    memset(buf, 0, sizeof(espnow_data_t));
    // esp NOW data
    buf->id = UNIT_ROLE;
    buf->type = type;

    // save last message type to allow retranmission
    last_msg_type = type;

    switch (type)
    {
    case ESPNOW_DATA_BROADCAST:
        // nothing else to do
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

    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        case ID_ESPNOW_SEND_CB:
        {
            espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
            addr_type = IS_BROADCAST_ADDR(send_cb->mac_addr) ? true : false;
            esp_now_get_peer_num(&peer_num);

            ESP_LOGI(TAG, "Send data to " MACSTR ". status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

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

                // send broadcast data again
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    espnow_data_prepare(espnow_data, ESPNOW_DATA_BROADCAST);
                    esp_now_send(broadcast_mac, (uint8_t *)espnow_data, sizeof(espnow_data_t));
                }
                else
                    ESP_LOGE(TAG, "Could not take send semaphore");
            }
            else
            {
                if (send_cb->status != ESP_NOW_SEND_SUCCESS)
                {
                    ESP_LOGE(TAG, "ERROR SENDING DATA TO MASTER");
                    comms_fail++;

                    //*MAX_COMMS_CONSECUTIVE_ERRORS --> RESTART
                    if (comms_fail > MAX_COMMS_ERROR)
                    {
                        ESP_LOGE(TAG, "TOO MANY COMMS ERRORS, DISCONNECTING");
                        pad_status = PAD_DISCONNECTED;
                        safely_switch_off();
                        esp_now_del_peer(master_mac);
                        xTimerStop(dynamic_timer, 0);
                        xTimerStop(alert_timer, 0);
                        xTimerDelete(dynamic_timer, 0);
                        xTimerDelete(alert_timer, 0);

                        // reboot
                        esp_restart();
                    }
                    else //*RETRANSMISSIONS
                    {
                        ESP_LOGW(TAG, "RETRANSMISSION n. %d", comms_fail);
                        espnow_data_prepare(espnow_data, last_msg_type);
                        esp_now_send(send_cb->mac_addr, (uint8_t *)espnow_data, sizeof(espnow_data_t));
                    }
                }
                else
                {
                    comms_fail = 0;
                }
            }
            break;
        }
        case ID_ESPNOW_RECV_CB:
        {
            espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

            // Check CRC of received ESPNOW data.
            if (!espnow_data_crc_control(recv_cb->data, recv_cb->data_len))
            {
                ESP_LOGE(TAG, "Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
                break;
            }

            espnow_data_t *recv_data = (espnow_data_t *)recv_cb->data;
            addr_type = recv_data->type;

            if (addr_type == ESPNOW_DATA_BROADCAST)
            {
                // ESP_LOGI(TAG, "Receive broadcast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                //! ignore it as it comes from other peers
            }
            else if (addr_type == ESPNOW_DATA_CONTROL)
            {
                ESP_LOGI(TAG, "Receive control data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));

                //* Add MASTER
                /* If MAC address does not exist in peer list, add it to peer list. */
                if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                {
                    if (recv_data->field_1 > 1) // meaning the payload contains the alerts thresholds
                    {
                        ESP_LOGW(TAG, "Setting alerts limits");
                        // parse data to set local alerts limits
                        OVERCURRENT = recv_data->field_1;
                        OVERVOLTAGE = recv_data->field_2;
                        OVERTEMPERATURE = recv_data->field_3;
                        FOD_ACTIVE = recv_data->field_4;
                    }
                    else
                    {
                        memcpy(master_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_LOGW(TAG, "Add master " MACSTR " to peer list", MAC2STR(master_mac));
                        esp_now_register_master(master_mac, true);
                        if (dynamic_timer == NULL)
                        {
                            // freeRTOS timers
                            const int dyn_timegap = pdMS_TO_TICKS((int)recv_data->field_3);

                            dynamic_timer = xTimerCreate("dynamic_timer", dyn_timegap, pdTRUE, NULL, dynamic_timer_callback);
                            ESP_ERROR_CHECK(dynamic_timer == NULL ? ESP_FAIL : ESP_OK);
                            alert_timer = xTimerCreate("alert_timer", ALERT_TIMEGAP, pdTRUE, NULL, alert_timer_callback);
                            ESP_ERROR_CHECK(alert_timer == NULL ? ESP_FAIL : ESP_OK);
                            //! start sending measurements data to the master
                            if (xTimerStart(dynamic_timer, 0) != pdPASS)
                            {
                                ESP_LOGE(TAG, "Cannot start dynamic timer");
                            }
                            if (xTimerStart(alert_timer, 0) != pdPASS)
                            {
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
                    ESP_LOGI(TAG, "Master " MACSTR " already in peer list", MAC2STR(master_mac));
                    // TODO: handle the dynamic change of dynamic timer from field_3 - check if it is different from dyn_timegap

                    // handle strip and switches
                    if (recv_data->field_1)
                    {
                        safely_enable_full_power();
                        if (recv_data->field_4 == LED_CHARGING)
                        {
                            strip_misalignment = false;
                            strip_enable = false;
                            strip_charging = true;
                        }
                        else if (recv_data->field_4 == LED_MISALIGNED)
                        {
                            strip_enable = false;
                            strip_charging = false;
                            strip_misalignment = true;
                        }
                        pad_status = PAD_FULL_POWER;
                    }
                    else if (recv_data->field_2)
                    {
                        // safely_enable_low_power(); //!todo: reactivate low power!
                        safely_enable_full_power();
                        pad_status = PAD_LOW_POWER;
                    }
                    else
                    {
                        safely_switch_off();
                        pad_status = PAD_CONNECTED;
                        if (recv_data->field_4 == LED_OFF) // scooter left
                        {
                            strip_misalignment = false;
                            strip_charging = false;
                            strip_enable = true;
                        }
                        else if (recv_data->field_4 == LED_FULLY_CHARGED) // scooter is fully charged and still placed on it
                        {
                            strip_misalignment = false;
                            strip_charging = false;
                            strip_enable = false;
                            set_strip(0, 200, 0);
                        }
                        else if (recv_data->field_4 == LED_ALERT) // alert on the pad or the relative scooter
                        {
                            strip_misalignment = false;
                            strip_charging = false;
                            strip_enable = false;
                            set_strip(200, 0, 0);
                        }
                    }
                }
            }
            else if (addr_type == ESPNOW_DATA_ALERT)
            {
                // ESP_LOGI(TAG, "Receive alert data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                // REBOOT
                if ((recv_data->field_2 == ALERT_MESSAGE) && (recv_data->field_3 == ALERT_MESSAGE) && (recv_data->field_4 == ALERT_MESSAGE))
                {
                    ESP_LOGE(TAG, "REBOOTING");
                    safely_switch_off();
                    xTimerStop(dynamic_timer, 0);
                    xTimerStop(alert_timer, 0);
                    xTimerDelete(dynamic_timer, 0);
                    xTimerDelete(alert_timer, 0);

                    if (pad_status != PAD_ALERT)
                        vTaskDelay(pdMS_TO_TICKS(recv_data->field_1 * 1000));

                    esp_now_del_peer(master_mac);

                    // reboot
                    esp_restart();
                }
            }
            else
                ESP_LOGI(TAG, "Receive unexpected message type %d data from: " MACSTR "", addr_type, MAC2STR(recv_cb->mac_addr));

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

esp_err_t espnow_init(void)
{
    uint8_t rc;

    // Create queue to process callbacks from espnow
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(ESPNOW_SEND_CB));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(ESPNOW_RECV_CB));
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_register_master(broadcast_mac, false);

    espnow_data_t *buffer = malloc(sizeof(espnow_data_t));
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(buffer);
        return ESP_FAIL;
    }

    // create a task to handle espnow events
    rc = xTaskCreate(espnow_task, "espnow_task", 2048, buffer, 4, NULL);
    if (rc != pdPASS)
    {
        ESP_LOGE(TAG, "Create espnow task fail");
        free(buffer);
        return ESP_FAIL;
    }

    send_semaphore = xSemaphoreCreateBinary();
    if (send_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Create send semaphore fail");
        return ESP_FAIL;
    }

    // crucial! otherwise the first message is not sent
    xSemaphoreGive(send_semaphore);

    ESP_LOGI(TAG, "Start sending broadcast data");
    if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
    {
        espnow_data_prepare(buffer, ESPNOW_DATA_BROADCAST);
        esp_now_send(broadcast_mac, (uint8_t *)buffer, sizeof(espnow_data_t));
    }
    else
        ESP_LOGE(TAG, "Could not take send semaphore");

    return ESP_OK;
}
