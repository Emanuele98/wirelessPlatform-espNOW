#include "espnow.h"

static const char *TAG = "ESP-NOW";

extern bool MQTT_ACTIVE;
static uint8_t mqtt_refresh[NUMBER_TX + NUMBER_RX] = {0};

static SemaphoreHandle_t send_semaphore = NULL;
static QueueHandle_t espnow_queue;
static QueueHandle_t localization_queue;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//static esp_now_peer_num_t peer_num = { 0, 0 }; //use esp_now_peer_num_t(&peer_num) to get the number of peers

static pad_status pads_status[NUMBER_TX] = {PAD_DISCONNECTED};
bool connected_pads = false;
static scooter_status scooters_status[NUMBER_RX] = {SCOOTER_DISCONNECTED};
static bool scooter_tobechecked[NUMBER_RX] = {false};
static bool scooters_left[NUMBER_RX] = {false};
static uint8_t n_peer_broadcasts[NUMBER_TX + NUMBER_RX] = {0};

static uint8_t Baton = 0;
static uint16_t comms_fail = 0;

static bool peer_msg_received[NUMBER_TX + NUMBER_RX] = {true};

void espnow_data_prepare(espnow_data_t *buf, message_type type, peer_id id);

extern esp_mqtt_client_handle_t client;

/* LIST OF TOPICS FOR MQTT*/
 //* TX
extern const char tx_voltage[4][80];
extern const char tx_current[4][80];
extern const char tx_temp1[4][80];
extern const char tx_temp2[4][80];
extern const char tx_power[4][80];
extern const char tx_efficiency[4][80];
extern const char tx_fod[4][80];
extern const char tx_overvoltage[4][80];
extern const char tx_overcurrent[4][80];
extern const char tx_overtemperature[4][80];
extern const char tx_status[4][80];
extern const char tx_scooter[4][80];
 //* RX
extern const char rx_voltage[4][80];
extern const char rx_current[4][80];
extern const char rx_temp[4][80];
extern const char rx_power[4][80];
extern const char rx_charge_complete[4][80];
extern const char rx_overvoltage[4][80];
extern const char rx_overcurrent[4][80];
extern const char rx_overtemperature[4][80];
  //* DEBUG
extern const char debug[80];

/* WiFi should start before using ESPNOW */
//? without wifi connection - only esp now

void wifi_init_connectionless(void)
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

static void esp_now_register_peer(uint8_t *mac_addr)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        return;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);
}

static void esp_now_encrypt_peer(uint8_t *mac_addr)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        return;
    }
    esp_now_get_peer(mac_addr, peer);
    //add local master key (LMK)
    peer->encrypt = true;
    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    ESP_ERROR_CHECK( esp_now_mod_peer(peer) );
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
    if (xQueueSend(espnow_queue, &evt, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }

    //give back the semaphore if status is successfull
    if (status == ESP_NOW_SEND_SUCCESS)
        xSemaphoreGive(send_semaphore);
}

static void ESPNOW_RECV_CB(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    int rssi = recv_info->rx_ctrl->rssi;

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

    espnow_data_t *recv_data = (espnow_data_t *)recv_cb->data;

    //check unit id --> scooter --> check rssi
    //check messge type --> localization --> special queue
    if (recv_data->id > NUMBER_TX )
    {
        // if it is not close enough
        if (rssi < RSSI_LIMIT) //TODO: if scooter is connected - remove it from the list - CHECK ID
        {
            ESP_LOGW(TAG, "Scooter not close enough");
            return;
        }
        
        // if it is a localization message
        if (recv_data->type == ESPNOW_DATA_LOCALIZATION)
        {
            if (xQueueSend(localization_queue, &evt, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) != pdTRUE)
            {
                ESP_LOGW(TAG, "Send localization queue fail");
                free(recv_cb->data);
            }
            return;
        }
    }

    if (xQueueSend(espnow_queue, &evt, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

static void handle_peer_alert(espnow_data_t *data, espnow_data_t *alert_data)
{
    //send ALERT to the dashboard
    peer_id unitID = data->id;

    if (unitID > NUMBER_TX) //scooter
    {
        if (MQTT_ACTIVE)
        {            
            if (data->field_1) // overvoltage
                esp_mqtt_client_publish(client, rx_overvoltage[unitID - NUMBER_TX - 1], "1", 0, MQTT_QoS, 0);
            else if (data->field_2) //overcurrent
                    esp_mqtt_client_publish(client, rx_overcurrent[unitID - NUMBER_TX - 1], "1", 0, MQTT_QoS, 0);
                else if (data->field_3) //overtemperature
                        esp_mqtt_client_publish(client, rx_overtemperature[unitID - NUMBER_TX - 1], "1", 0, MQTT_QoS, 0);
                    else if (data->field_4) //charge complete
                            esp_mqtt_client_publish(client, rx_charge_complete[unitID - NUMBER_TX - 1], "1", 0, MQTT_QoS, 0);
        }

        struct peer *scooter = peer_find_by_id(unitID);
        if (scooter == NULL)
        {
            ESP_LOGE(TAG, "Scooter %d not found", unitID);
            return;
        }

        //if its position is found - switch its relative pad off   
        if (scooter->position != 0)
        {
            struct peer *pad = peer_find_by_id(scooter->position);
            if (pad == NULL)
            {
                ESP_LOGE(TAG, "Pad %d not found", scooter->position);
                return;
            }

            //status off
            if (MQTT_ACTIVE)
                esp_mqtt_client_publish(client, tx_status[pad->position-1], "0", 0, MQTT_QoS, 0);

            if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
            {
                pad->full_power = false;
                pad->low_power = false;
                if (data->field_4)
                    pad->led_command = LED_FULLY_CHARGED;
                else
                    pad->led_command = LED_ALERT;
                espnow_data_prepare(alert_data, ESPNOW_DATA_CONTROL, pad->id);
                esp_now_send(pad->mac, (uint8_t *)alert_data, sizeof(espnow_data_t));
            } else
                ESP_LOGE(TAG, "Could not take the send semaphore!");
        }

        // take care of the scooter
        if (data->field_4)
        {
            scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_FULLY_CHARGED;
            pads_status[scooter->position - 1] = PAD_FULLY_CHARGED;
            // we wait to receive its accelerometer movement
        }
        else   
        {
            //stay connected - send RESET command after a reasonable time
            scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_ALERT;
            pads_status[scooter->position - 1] = PAD_FULLY_CHARGED;  //small hack to unlock the pad when it receives the scooter moves (as it happens in fully charged)

            // the pad stays red until the scooter either moves or reconnects
            if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
            {
                espnow_data_prepare(alert_data, ESPNOW_DATA_LOCALIZATION, scooter->id);
                esp_now_send(scooter->mac, (uint8_t *)alert_data, sizeof(espnow_data_t));
            } else
                ESP_LOGE(TAG, "Could not take the send semaphore!");

            //pad will be available again when scooter moves
            vTaskDelay(pdMS_TO_TICKS(SCOOTER_ALERT_TIMEOUT*1000));
            ESP_LOGE(TAG, "Scooter %d REBOOT", unitID);
            if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
            {
                espnow_data_prepare(alert_data, ESPNOW_DATA_ALERT, unitID);
                esp_now_send(scooter->mac, (uint8_t *)alert_data, sizeof(espnow_data_t));
            } else
                ESP_LOGE(TAG, "Could not take the send semaphore!");

            //todo: scooter reconnection - whatup with the pad
            // if this time passes and the scooter is still on alert
            // then pad goes back to normal - check pad->led_command - PAD_CONNECTED
            // scooter reconnection - check if there was a fully charged pad. 1 to 1
        }
    }
    else //pad
    {
        //switch off the pad (if it comes from a pad it should be already off but better to be sure)    
        struct peer *pad = peer_find_by_id(unitID);
        if (pad == NULL)
        {
            ESP_LOGE(TAG, "Pad %d not found", unitID);
            return;
        }

        if (MQTT_ACTIVE)
        {
            if(data->field_1) //overtemperature
                esp_mqtt_client_publish(client, tx_overtemperature[unitID - 1], "1", 0, MQTT_QoS, 0);
                else if(data->field_2) //overvoltage
                        esp_mqtt_client_publish(client, tx_overvoltage[unitID - 1], "1", 0, MQTT_QoS, 0);
                    else if(data->field_3) //overcurrent
                            esp_mqtt_client_publish(client, tx_overcurrent[unitID - 1], "1", 0, MQTT_QoS, 0);
                        else if(data->field_4) //fod
                                esp_mqtt_client_publish(client, tx_fod[unitID - 1], "1", 0, MQTT_QoS, 0);

            //status off
            esp_mqtt_client_publish(client, tx_status[pad->position-1], "0", 0, MQTT_QoS, 0);
        }
                
        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
        {
            pad->low_power = false;
            pad->full_power = false;
            pad->led_command = LED_ALERT;
            espnow_data_prepare(alert_data, ESPNOW_DATA_CONTROL, pad->id);
            esp_now_send(pad->mac, (uint8_t *)alert_data, sizeof(espnow_data_t));
        } else
            ESP_LOGE(TAG, "Could not take the send semaphore!");

        //handle relative scooter (if there is one)        
        struct peer *scooter = peer_find_by_position(pad->position);
        if (scooter != NULL)
        {
            scooters_status[scooter->id - NUMBER_TX - 1] = SCOOTER_ALERT;
            //send localization request -- scooter will send as soon as it moves
            if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
            {
                espnow_data_prepare(alert_data, ESPNOW_DATA_LOCALIZATION, scooter->id);
                esp_now_send(scooter->mac, (uint8_t *)alert_data, sizeof(espnow_data_t));
            } else
                ESP_LOGE(TAG, "Could not take the send semaphore!");
        }        

        //stay connected - send RESET command after a reasonable time
        pads_status[unitID - 1] = PAD_ALERT;
        //todo: remove this scempio - save values into nvs - block reconnection for a while - need the ntp server for reliable time values
        vTaskDelay(pdMS_TO_TICKS(PAD_ALERT_TIMEOUT*1000));
        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
        {
            espnow_data_prepare(alert_data, ESPNOW_DATA_ALERT, unitID);
            esp_now_send(pad->mac, (uint8_t *)alert_data, sizeof(espnow_data_t));
        } else
            ESP_LOGE(TAG, "Could not take the send semaphore!");
    }
}



static void handle_peer_dynamic(espnow_data_t* data, espnow_data_t *dynamic_data)
{
    peer_id unitID = data->id;
    struct peer *p = peer_find_by_id(unitID);
    if (p == NULL)
    {
        ESP_LOGE(TAG, "Peer %d not found", unitID);
        return;
    }

    // message received - check
    peer_msg_received[unitID] = true;

    // check it was different from the previous one
    //? compare recv_data with the one in the peer structure
    if (MQTT_ACTIVE)
    {
        static char value[50];
        if (unitID < NUMBER_TX)
        {
            if ((fabs(data->field_1 - p->dyn_payload.voltage) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_1);
                esp_mqtt_client_publish(client, tx_voltage[unitID-1], value, 0, MQTT_QoS, 0);
            }
            if ((fabs(data->field_2 - p->dyn_payload.current) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_2);
                esp_mqtt_client_publish(client, tx_current[unitID-1], value, 0, MQTT_QoS, 0);
            }
            if ((fabs(data->field_3 - p->dyn_payload.temp1) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_3);
                esp_mqtt_client_publish(client, tx_temp1[unitID-1], value, 0, MQTT_QoS, 0);
            }
            if ((fabs(data->field_4 - p->dyn_payload.temp2) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_4);
                esp_mqtt_client_publish(client, tx_temp2[unitID-1], value, 0, MQTT_QoS, 0);
            }
            if ((fabs(p->dyn_payload.tx_power - (data->field_1 * data->field_2)) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", (data->field_1 * data->field_2));
                esp_mqtt_client_publish(client, tx_power[unitID-1], value, 0, MQTT_QoS, 0);
            }

            //todo: efficiency - must use time to correlate correctly the values
        }
        else
        {
            if ((fabs(data->field_1 - p->dyn_payload.voltage) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_1);
                esp_mqtt_client_publish(client, rx_voltage[unitID-NUMBER_TX-1], value, 0, MQTT_QoS, 0);
            }
            if ((fabs(data->field_2 - p->dyn_payload.current) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_2);
                esp_mqtt_client_publish(client, rx_current[unitID-NUMBER_TX-1], value, 0, MQTT_QoS, 0);
            }
            if ((fabs(data->field_3 - p->dyn_payload.temp1) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_3);
                esp_mqtt_client_publish(client, rx_temp[unitID-NUMBER_TX-1], value, 0, MQTT_QoS, 0);
            }
            /*
            if ((fabs(data->field_4 - p->dyn_payload.temp2) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", data->field_4);
                esp_mqtt_client_publish(client, rx_temp2[unitID-NUMBER_TX-1], value, 0, MQTT_QoS, 0); //todo: do we want to send it?
            }
            */
           //print fabs
            if ((fabs(p->dyn_payload.rx_power - (data->field_1 * data->field_2)) > MQTT_MIN_DELTA) || (mqtt_refresh[unitID] == 0))
            {
                sprintf(value, "%.2f", (data->field_1 * data->field_2));
                esp_mqtt_client_publish(client, rx_power[unitID-NUMBER_TX-1], value, 0, MQTT_QoS, 0); //todo: check power values whatsupp
            }
        }
        mqtt_refresh[unitID] = (mqtt_refresh[unitID] + 1) % MQTT_REFRESH_LIMIT;
    }
    //*save the values into peer structure
    p->dyn_payload.voltage = data->field_1;
    p->dyn_payload.current = data->field_2;
    p->dyn_payload.temp1 = data->field_3;
    p->dyn_payload.temp2 = data->field_4;
    
    if (unitID < NUMBER_TX)
        p->dyn_payload.tx_power = p->dyn_payload.voltage * p->dyn_payload.current;         
    else
        p->dyn_payload.rx_power = p->dyn_payload.voltage * p->dyn_payload.current;
    //todo: make an average for the power values?
    //todo: efficiency
    //todo: save time of this values


    //CHECK WHETER THE SCOOTER IS MISALIGNED OR IT LEFT THE PLATFORM
    if ((p->type == SCOOTER) && (p->position != 0))
    {
        struct peer *pad = peer_find_by_id(p->position);
        if (pad == NULL)
        {
            ESP_LOGE(TAG, "Pad %d not found!!", p->position);
            return;
        }

        //check if the pad is in alert or fully charged
        if ((pads_status[pad->id - 1] == PAD_ALERT) || (pads_status[pad->id - 1] == PAD_FULLY_CHARGED))
            return;

        // check the scooter did not leave
        if (p->dyn_payload.voltage < SCOOTER_LEFT_LIMIT) //!maybe need to wait a minimum time?
        {
            if (scooters_left[unitID - NUMBER_TX - 1])
            {
                ESP_LOGW(TAG, "Scooter %d left", unitID);
                scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_DISCONNECTED;
                pads_status[pad->id - 1] = PAD_CONNECTED;
                //switch off dashboard led
                if (MQTT_ACTIVE)
                    esp_mqtt_client_publish(client, tx_status[p->position-1], "0", 0, MQTT_QoS, 0);

                //SEND REBOOT COMMAND TO THE SCOOTER
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    espnow_data_prepare(dynamic_data, ESPNOW_DATA_ALERT, unitID);
                    esp_now_send(p->mac, (uint8_t *)dynamic_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");

                //SWITCH OFF THE RELATIVE PAD
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    pad->led_command = LED_OFF;
                    pad->full_power = false;
                    espnow_data_prepare(dynamic_data, ESPNOW_DATA_CONTROL, pad->id);
                    esp_now_send(pad->mac, (uint8_t *)dynamic_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
            }
            scooters_left[unitID - NUMBER_TX - 1] = true;
        }
        else if (p->dyn_payload.voltage < MISALIGNED_LIMIT)
        {
            if (scooters_status[unitID - NUMBER_TX - 1] == SCOOTER_CHARGING)
            {
                ESP_LOGW(TAG, "Scooter %d misaligned", unitID);
                scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_MISALIGNED;
                //SEND LED COMMAND TO RELATIVE PAD
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    pad->led_command = LED_MISALIGNED;
                    espnow_data_prepare(dynamic_data, ESPNOW_DATA_CONTROL, pad->id);
                    esp_now_send(pad->mac, (uint8_t *)dynamic_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
            }
        } 
        else
        {
            if (scooters_status[unitID - NUMBER_TX - 1] == SCOOTER_MISALIGNED)
            {
                ESP_LOGW(TAG, "Scooter %d aligned", unitID);
                //SEND LED COMMAND TO RELATIVE PAD
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    pad->led_command = LED_CHARGING;
                    espnow_data_prepare(dynamic_data, ESPNOW_DATA_CONTROL, pad->id);
                    esp_now_send(pad->mac, (uint8_t *)dynamic_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");                
            }
            scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_CHARGING;
            scooters_left[unitID - NUMBER_TX - 1] = false;
        }
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
void espnow_data_prepare(espnow_data_t *buf, message_type type, peer_id id)
{  
    //INITILIAZE THE BUFFER
    memset(buf, 0, sizeof(espnow_data_t));
    // esp NOW data
    buf->id = UNIT_ROLE;
    buf->type = type;

    // find which peers to control/localize
    struct peer *p = peer_find_by_id(id);
    if (p == NULL)
    {
        ESP_LOGE(TAG, "Peer not found");
        return;
    }

    //save message type in peer structure
    p->last_msg_type = type;

    switch (type)
    {
        case ESPNOW_DATA_BROADCAST:
            // only received - never sent
            break;
    
        case ESPNOW_DATA_LOCALIZATION:
            // ask for received voltage after a minimum time
            //todo: exponential time based on the number of times the scooter has been checked
            //todo: exploit accelerometer from scooter - do not check if checked already twice and it did not move since - but be careful with scooter_tobechecked reset
            buf->field_1 = REACTION_TIME;
            buf->field_2 = buf->field_3 = buf->field_4 = 0;
            break;

        case ESPNOW_DATA_ALERT:
            // sent to tell the peer to RESET
            if (!p->fully_charged)
            {
                buf->field_1 = SCOOTER_LEFT_TIMEOUT;
                buf->field_2 = buf->field_3 = buf->field_4 = ALERT_MESSAGE;
            }
            else
            {
                //todo: whatch out in send success check that deletes the peer
                //send fully charged command
                //wait for its movement
            }
            break;
        
        case ESPNOW_DATA_DYNAMIC:
            // only received - never sent
            break;
        
        case ESPNOW_DATA_CONTROL:
            //first message --> set alerts limits!
            if ((p->type == PAD) && (pads_status[p->id - 1] == PAD_DISCONNECTED))
            {
                //ESP_LOGW(TAG, "SET PAD ALERTS LIMITS");
                buf->field_1 = OVERCURRENT_TX;
                buf->field_2 = OVERVOLTAGE_TX;
                buf->field_3 = OVERTEMPERATURE_TX;
                buf->field_4 = FOD_ACTIVE;
            }
            else if ((p->type == SCOOTER) && (scooters_status[p->id - NUMBER_TX - 1] == SCOOTER_DISCONNECTED))
            {
                //ESP_LOGW(TAG, "SET SCOOTER ALERTS LIMITS");
                buf->field_1 = OVERCURRENT_RX;
                buf->field_2 = OVERVOLTAGE_RX;
                buf->field_3 = OVERTEMPERATURE_RX;
                buf->field_4 = MIN_VOLTAGE;
            }
            else 
            {
                //ESP_LOGW(TAG, "SWITCH ON/OFF");
                buf->field_1 = p->full_power;
                buf->field_2 = p->low_power;
                buf->field_3 = PEER_DYNAMIC_TIMER; 
                buf->field_4 = p->led_command;
            }
            break;

        default:
            ESP_LOGE(TAG, "Message type error: %d", type);
            break;
    }

    // CRC
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, sizeof(espnow_data_t));
}

static uint8_t parse_localization_message(espnow_data_t *data)
{
    if ((data->field_1 == ACCELEROMETER_MESSAGE) && (data->field_2 == ACCELEROMETER_MESSAGE) && (data->field_3 == ACCELEROMETER_MESSAGE) && (data->field_4 == ACCELEROMETER_MESSAGE))
        return ACCELEROMETER_WAKEUP;
    else if ((data->field_1 == LOC_START_MESSAGE) && (data->field_2 == LOC_START_MESSAGE) && (data->field_3 == LOC_START_MESSAGE) && (data->field_4 == LOC_START_MESSAGE))
        return LOCALIZATION_START;
    else if ((data->field_1 == LOC_STOP_MESSAGE) && (data->field_2 == LOC_STOP_MESSAGE) && (data->field_3 == LOC_STOP_MESSAGE) && (data->field_4 == LOC_STOP_MESSAGE))
            return LOCALIZATION_STOP;
    else
            return LOCALIZATION_CHECK;
}

static bool loc_pads_off()
{
    for (uint8_t i = 0; i < NUMBER_TX; i++)
    {
        if (pads_status[i] == PAD_LOW_POWER)
            return false;
    }
    return true;
}

static uint8_t find_pad_position()
{
    for (uint8_t i = 0; i < NUMBER_TX; i++)
    {
        if (pads_status[i] == PAD_LOW_POWER)
            return i;
    }
    return 0;
}

static bool pass_the_baton(espnow_data_t *loc_data)
{
    //checking there is no scooter to be checked with the current pad yet
    bool all_scooter_checked = true;
    for (uint8_t i = 0; i < NUMBER_RX; i++)
    {
        if (scooter_tobechecked[i])
            all_scooter_checked = false;
        //todo: if too much time passed since the last baton_passed (and there at least one other pad connected)
        //todo: reset the scooter_tobechecked array
        //the low power pad will be switched off and the baton will be passed to the next one
    }
    //checking there is at least one pad available
    bool pad_available = false;
    for (uint8_t i = 0; i < NUMBER_TX; i++)
    {
        if (pads_status[i] == PAD_CONNECTED)
            pad_available = true;
    }

    if ((all_scooter_checked) && (pad_available))
    {
        ESP_LOGW(TAG, "PASSING THE BATON");
        bool baton_passed = false;
        
        while(!baton_passed)
        {
            if (pads_status[Baton] == PAD_LOW_POWER)
            {
                //switch off the previous pad
                struct peer *p = peer_find_by_id(Baton + 1);
                if (p == NULL)
                {
                    ESP_LOGE(TAG, "Peer not found");
                    return 0;
                }
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    p->low_power = 0;
                    pads_status[Baton] = PAD_CONNECTED;
                    espnow_data_prepare(loc_data, ESPNOW_DATA_CONTROL, p->id);
                    esp_now_send(p->mac, (uint8_t *)loc_data, sizeof(espnow_data_t));
                    ESP_LOGW(TAG, "pad switched OFF: %d", Baton + 1);
                } else 
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
            }
            else if ((pads_status[Baton] == PAD_CONNECTED) && (loc_pads_off()))
            {                
                //switch on the new pad
                struct peer *p = peer_find_by_id(Baton + 1);
                if (p == NULL)
                {
                    ESP_LOGE(TAG, "Peer not found");
                    return 0;
                }
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    p->low_power = 1;
                    pads_status[Baton] = PAD_LOW_POWER;
                    espnow_data_prepare(loc_data, ESPNOW_DATA_CONTROL, p->id);
                    esp_now_send(p->mac, (uint8_t *)loc_data, sizeof(espnow_data_t));
                    baton_passed = true;
                    ESP_LOGW(TAG, "pad switched ON: %d", Baton + 1);
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
            }

            Baton = (Baton + 1) % NUMBER_TX;
            //ESP_LOGW(TAG, "BATON: %d", Baton);
        }

        return 1;
    }

    return 0;
}


static void localization_task(void *pvParameter)
{
    localization_queue = xQueueCreate(LOC_QUEUE_SIZE, sizeof(espnow_event_t));
    if (localization_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }
    
    espnow_event_t evt;
    espnow_data_t *localization_data = (espnow_data_t *)pvParameter;

    while (xQueueReceive(localization_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        //ESP_LOGW(TAG, "LOCALIZATION REQUEST RECEIVED");

        espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
        espnow_data_t *recv_data = (espnow_data_t *)recv_cb->data;

        struct peer *scooter = peer_find_by_id(recv_data->id);
        if (scooter == NULL)
        {
            ESP_LOGE(TAG, "Peer not found!");
            break;
        }

        // check which message type we received
        uint8_t type = parse_localization_message(recv_data);

        switch(type)
        {
            case LOCALIZATION_START:
                ESP_LOGW(TAG, "LOCALIZATION START");

                if (pass_the_baton(localization_data))
                    scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = true;

                // send command to get voltage after a minimum time
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    espnow_data_prepare(localization_data, ESPNOW_DATA_LOCALIZATION, recv_data->id);
                    esp_now_send(recv_cb->mac_addr, (uint8_t *)localization_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
                
                break;
            
            case LOCALIZATION_CHECK:
                ESP_LOGW(TAG, "LOCALIZATION CHECK");

                scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = false;
                
                if (pass_the_baton(localization_data))
                    scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = true;

                // send command to get voltage after a minimum time
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    espnow_data_prepare(localization_data, ESPNOW_DATA_LOCALIZATION, recv_data->id);
                    esp_now_send(recv_cb->mac_addr, (uint8_t *)localization_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
        
                break;

            case LOCALIZATION_STOP: //voltage thresh is passed - position found
                ESP_LOGW(TAG, "LOCALIZATION STOPPED");
                scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = false;
                //find which pad is on
                uint8_t pad_position = find_pad_position();
                struct peer *pad = peer_find_by_id(pad_position + 1);
                if (pad == NULL)
                {
                    ESP_LOGE(TAG, "Peer %d not found!", pad_position + 1);
                    break;
                }
                pad->low_power = 0;
                pad->full_power = 1;
                pad->led_command = LED_CHARGING;
                pads_status[pad_position] = PAD_FULL_POWER;
                scooters_status[recv_data->id - NUMBER_TX - 1] = SCOOTER_CHARGING;
                ESP_LOGW(TAG, "SCOOTER %d CHARGING ON PAD %d", recv_data->id - NUMBER_TX, pad_position + 1);
                //update scooter position
                scooter->position = pad_position + 1;
                if (MQTT_ACTIVE)
                {
                    esp_mqtt_client_publish(client, tx_status[pad_position], "1", 0, MQTT_QoS, 0);
                    if (recv_data->id == SCOOTER1)
                        esp_mqtt_client_publish(client, tx_scooter[pad_position], "3PAU", 0, MQTT_QoS, 0);
                    else if (recv_data->id == SCOOTER2)
                        esp_mqtt_client_publish(client, tx_scooter[pad_position], "ISIN", 0, MQTT_QoS, 0);
                }
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    espnow_data_prepare(localization_data, ESPNOW_DATA_CONTROL, pad->id);
                    esp_now_send(pad->mac, (uint8_t *)localization_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
                break;

            case ACCELEROMETER_WAKEUP:
                // this is received after the scooter went to either FULLY_CHARGED or ALERT
                ESP_LOGW(TAG, "ACCELEROMETER WAKEUP");
                struct peer *p = peer_find_by_id(scooter->position);
                if (scooter == NULL)
                {
                    ESP_LOGE(TAG, "Peer not found!");
                    break;
                }
                if (p != NULL)
                {
                    if (pads_status[p->id -1] == PAD_FULLY_CHARGED)
                    {
                        //make the relative pad free again
                        p->full_power = p->low_power = false;
                        p->led_command = LED_OFF;
                        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                        {
                            espnow_data_prepare(localization_data, ESPNOW_DATA_CONTROL, p->id);
                            esp_now_send(p->mac, (uint8_t *)localization_data, sizeof(espnow_data_t));
                        } else
                            ESP_LOGE(TAG, "Could not take the send semaphore!");
                        pads_status[p->id - 1] = PAD_CONNECTED;      
                    } 
                }
                // reset the scooter (for both cases: fully charged or alert)
                if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                {
                    espnow_data_prepare(localization_data, ESPNOW_DATA_ALERT, scooter->id);
                    esp_now_send(scooter->mac, (uint8_t *)localization_data, sizeof(espnow_data_t));
                } else
                    ESP_LOGE(TAG, "Could not take the send semaphore!");
                break;

            default:
                ESP_LOGE(TAG, "Localization message type error: %d", type);
                break;
        }

        free(recv_data);
    }

    vQueueDelete(localization_queue);
    vTaskDelete(NULL);
    free(localization_data);
}

static void espnow_task(void *pvParameter)
{
    //Create queue to process esp now events
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }
    
    espnow_event_t evt;
    uint8_t addr_type;
    peer_id unitID;
    espnow_data_t *espnow_data = (espnow_data_t *)pvParameter;


    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ID_ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                ESP_LOGI(TAG, "send data to "MACSTR", status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                addr_type = IS_BROADCAST_ADDR(send_cb->mac_addr) ? true : false;
                if (addr_type)
                {
                    //broadcast always successfull anyway (no ack)
                }
                else
                {
                    struct peer *p = peer_find_by_mac(send_cb->mac_addr);
                    if (p == NULL)
                    {
                        ESP_LOGE(TAG, "Peer not found");
                        break;
                    }
                    //unicast message
                    if (send_cb->status != ESP_NOW_SEND_SUCCESS) 
                    {
                        ESP_LOGE(TAG, "ERROR SENDING DATA TO "MACSTR"", MAC2STR(send_cb->mac_addr));
                        comms_fail++;

                        //*MAX_COMMS_CONSECUTIVE_ERRORS --> RESTART
                        if (comms_fail > MAX_COMMS_ERROR)
                        {
                            ESP_LOGE(TAG, "TOO MANY COMMS ERRORS, RESTARTING");

                             //SEND IT TO DEBUG MQTT TOPIC TO KEEP TRACK OF IT
                            if (MQTT_ACTIVE)
                            {
                                char value[50];
                                sprintf(value, "Send failed - n. %d", comms_fail);
                                esp_mqtt_client_publish(client, debug, value, 0, MQTT_QoS, 0);
                            }

                            //delete all peers
                            delete_all_peers();

                            //reboot
                            esp_restart();
                        }
                        else //*RETRANSMISSIONS
                        {
                            //retransmit
                            ESP_LOGW(TAG, "RETRANSMISSION n. %d", comms_fail);
                            espnow_data_prepare(espnow_data, p->last_msg_type, p->id);
                            esp_now_send(send_cb->mac_addr, (uint8_t *)espnow_data, sizeof(espnow_data_t));
                        }
                    }
                    else 
                    {
                        //reset comms - we are good
                        comms_fail = 0;
                        //if the successfull sent message was an alert - remove the peer so it can reconnect
                        if (p->last_msg_type == ESPNOW_DATA_ALERT)
                        {
                            if (p->id > NUMBER_TX)
                                scooters_status[p->id - NUMBER_TX - 1] = SCOOTER_DISCONNECTED;
                            else
                                pads_status[p->id - 1] = PAD_DISCONNECTED;
                            //remove peer from the esp now registered list
                            esp_now_del_peer(send_cb->mac_addr);
                            //delete peer
                            peer_delete(p->id);
                        }
                    }
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
                
                unitID = recv_data->id;
                addr_type = recv_data->type;

                if (addr_type == ESPNOW_DATA_BROADCAST) 
                {
                    ESP_LOGI(TAG, "Receive broadcast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    // todo: use time values to avoid reconnection after a while (alert - fully charged ) - save them into NVS like last time
                    // todo: when pad connects - check if there is saved scooter on its position already (might be on alert - if so, reset it)
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) 
                    {
                        //if its a scooter, there must be at least one pad connected!
                        if ((unitID > NUMBER_TX) && (!connected_pads))
                            break;
                        
                        //avoid encryption for the first message (needs to be registered on both sides)
                        esp_now_register_peer(recv_cb->mac_addr);

                        // save its details into peer structure 
                        peer_add(unitID, recv_cb->mac_addr);
                        struct peer *p = peer_find_by_id(unitID);
                        ESP_LOGW(TAG, "NEW PEER FOUND! ID: %d", unitID);

                        //Send unicast data to make him stop sending broadcast msg
                        // fill in with remote alerts tresholds
                        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                        {
                            espnow_data_prepare(espnow_data, ESPNOW_DATA_CONTROL, unitID);
                            esp_now_send(recv_cb->mac_addr, (uint8_t *)espnow_data, sizeof(espnow_data_t));
                        }
                        else
                            ESP_LOGE(TAG, "Could not take the send semaphore!");

                        if (p->id > NUMBER_TX)   
                        {
                            scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_CONNECTED;
                            scooters_left[unitID - NUMBER_TX - 1] = false;
                        }
                        else
                            pads_status[unitID - 1] = PAD_CONNECTED;

                        // send another control message to set the dynamic timer
                        // pads will start sending dynamic data after this time
                        // scooters will wait until their position is found
                        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                        {
                            espnow_data_prepare(espnow_data, ESPNOW_DATA_CONTROL, unitID);
                            esp_now_send(recv_cb->mac_addr, (uint8_t *)espnow_data, sizeof(espnow_data_t));
                        } else
                            ESP_LOGE(TAG, "Could not take the send semaphore!");

                        // then encrypt for future messages
                        esp_now_encrypt_peer(recv_cb->mac_addr);

                        //reset dashboard
                        mqtt_unit_reset(unitID);
                    }
                    else   //you receive broadcasts data from a registered peer = PEER RESET --> delete peer so you can restart the connection
                    {
                        //need to check multiple broadcasts as they might be sent slighly before register the master
                        if (n_peer_broadcasts[unitID-1] > MAX_BROADCASTS_BEFORE_RECONNECTION)
                        { 
                            //reset its status
                            if (unitID > NUMBER_TX)
                            {
                                scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_DISCONNECTED;
                                // todo: switch off relative pad
                            }
                            else
                            {
                                pads_status[unitID - 1] = PAD_DISCONNECTED;
                                //todo: reset relative scooter
                            }

                            //remove peer from the esp now registered list
                            esp_now_del_peer(recv_cb->mac_addr);

                            //delete peer
                            peer_delete(unitID);
                            n_peer_broadcasts[unitID-1] = 0;
                        }
                        else
                            n_peer_broadcasts[unitID-1]++;

                    }

                }
                else if(addr_type == ESPNOW_DATA_DYNAMIC)
                {
                    ESP_LOGI(TAG, "Receive DYNAMIC data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    handle_peer_dynamic(recv_data, espnow_data);

                }
                else if (addr_type == ESPNOW_DATA_ALERT)
                {
                    ESP_LOGI(TAG, "Receive ALERT data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    handle_peer_alert(recv_data, espnow_data);
                }
                else 
                    ESP_LOGI(TAG, "Receive unexpected message type %d data from: "MACSTR"", addr_type, MAC2STR(recv_cb->mac_addr));
                
                free(recv_data);
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

static void disconnection_callback(void)
{
    for (uint8_t i = 0; i < NUMBER_TX + NUMBER_RX; i++)
    {
        struct peer *p = peer_find_by_id(i + 1);
        if (p == NULL)
            return;

        if (!peer_msg_received[i+1])
        {
            if (p->id > NUMBER_TX)
            {
                scooter_status status = scooters_status[p->id - NUMBER_TX - 1];
                if (( status == SCOOTER_CHARGING ) || ( status == SCOOTER_MISALIGNED )) 
                {
                    scooters_status[p->id - NUMBER_TX - 1] = SCOOTER_DISCONNECTED;
                    ESP_LOGE(TAG, "SCOOTER %d DISCONNECTED", p->id - NUMBER_TX);
                    //todo: relative pad off
                }
                else
                    return;
            }
            else
            {
                pad_status status = pads_status[p->id - 1];
                if (( status != PAD_DISCONNECTED ) || ( status != PAD_ALERT ))
                {
                    pads_status[p->id - 1] = PAD_DISCONNECTED;
                    ESP_LOGE(TAG, "PAD %d DISCONNECTED", p->id);
                    //todo: relative scooter off - send it reset command
                }
                else
                    return;
            }

            //remove peer from the esp now registered list
            esp_now_del_peer(p->mac);
            //delete peer
            peer_delete(p->id);
        } 
        else
        {
            // reset the check
            peer_msg_received[i] = false;
        }
    }
}

esp_err_t espnow_init(void)
{
    uint8_t rc;

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(ESPNOW_SEND_CB) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(ESPNOW_RECV_CB) );
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    espnow_data_t *buffer = malloc(sizeof(espnow_data_t));
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        return ESP_FAIL;
    }

    espnow_data_t *loc_buffer = malloc(sizeof(espnow_data_t));
    if (loc_buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send localization buffer fail");
        return ESP_FAIL;
    }
    
    rc = xTaskCreate(espnow_task, "espnow_task", ESPNOW_TASK_SIZE, buffer, ESPNOW_TASK_PRIORITY, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Create espnow_task fail");
        free(buffer);
        return ESP_FAIL;
    }

    rc = xTaskCreate(localization_task, "localization_task", LOC_TASK_SIZE, loc_buffer, LOC_TASK_PRIORITY, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Create localization_task fail");
        free(loc_buffer);
        return ESP_FAIL;
    }

    send_semaphore = xSemaphoreCreateBinary();
    if (send_semaphore == NULL) {
        ESP_LOGE(TAG, "Create send semaphore fail");
        return ESP_FAIL;
    }

    //crucial! otherwise the first message is not sent
    xSemaphoreGive(send_semaphore);

    TimerHandle_t timer = xTimerCreate("disconnection", pdMS_TO_TICKS(ESPNOW_MAXDELAY), pdTRUE, NULL, disconnection_callback);
    xTimerStart(timer, pdMS_TO_TICKS(ESPNOW_MAXDELAY));


    return ESP_OK;
}

