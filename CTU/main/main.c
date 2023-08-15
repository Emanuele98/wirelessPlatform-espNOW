#include "espnow.h"
#include "peer.h"

/* ALERTS LIMITS TX */
#define OVERCURRENT_TX      3
#define OVERVOLTAGE_TX      50
#define OVERTEMPERATURE_TX  60
#define FOD_ACTIVE          1
/* ALERTS LIMITS RX */
#define OVERCURRENT_RX      3
#define OVERVOLTAGE_RX      50
#define OVERTEMPERATURE_RX  60
#define MIN_VOLTAGE         3.0

/* LOC TIMING */
#define REACTION_TIME       2    //seconds

static const char *TAG = "MAIN";
//todo: create an esp_now.c file and leave this main clean
//todo: create a wifi.c file and leave this main clean

static QueueHandle_t espnow_queue;
static QueueHandle_t localization_queue;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//static esp_now_peer_num_t peer_num = { 0, 0 }; //use esp_now_peer_num_t(&peer_num) to get the number of peers

static pad_status pads_status[NUMBER_TX] = {PAD_DISCONNECTED};
bool connected_pads = false;
static scooter_status scooters_status[NUMBER_RX] = {SCOOTER_DISCONNECTED};
static bool scooter_tobechecked[NUMBER_RX] = {false};

static uint8_t Baton = 0;


/* FreeRTOS event group to signal when we are connected*/

//static EventGroupHandle_t s_wifi_event_group;


/**
 * @brief Handler for WiFi and IP events
 * 
 */
/*
static void wifi_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    }
}
*/

/* WiFi should start before using ESPNOW */
/*
static void wifi_init(void)
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

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
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
        ESP_LOGI(TAG, "connected to ap SSID: manu");
        
        // print the primary and secondary channels
        //uint8_t primary, secondary;
        //esp_wifi_get_channel(&primary, &secondary);
        //ESP_LOGI(TAG, "primary channel: %d, secondary channel: %d", primary, secondary);       
        
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: manu");
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
*/

/* WiFi should start before using ESPNOW */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
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
    peer->ifidx = ESPNOW_WIFI_IF;
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
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void ESPNOW_RECV_CB(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    int rssi = recv_info->rx_ctrl->rssi;

    // TO DO: put in the queue only if the scooter is close enough
    //ESP_LOGE(TAG, "RSSI: %d", rssi);

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
        if (rssi < RSSI_LIMIT)
            return;
        
        // if it is a localization message
        if (recv_data->type == ESPNOW_DATA_LOCALIZATION)
        {
            if (xQueueSend(localization_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
            {
                ESP_LOGW(TAG, "Send localization queue fail");
                free(recv_cb->data);
            }
            return;
        }
    }

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
void espnow_data_prepare(espnow_data_t *buf, message_type type, peer_id id)
{  
    //INITILIAZE THE BUFFER
    memset(buf, 0, sizeof(espnow_data_t));
    // esp NOW data
    buf->id = UNIT_ROLE;
    buf->type = type;

    //! find which peers to control/localize
    struct peer *p = peer_find(id);
    if (p == NULL)
    {
        ESP_LOGE(TAG, "Peer not found");
        return;
    }

    switch (type)
    {
        case ESPNOW_DATA_BROADCAST:
            // only received - never sent
            break;
    
        case ESPNOW_DATA_LOCALIZATION:
            // ask for received voltage after a minimum time
            buf->field_1 = REACTION_TIME;
            buf->field_2 = buf->field_3 = buf->field_4 = 0;
            break;

        case ESPNOW_DATA_ALERT:
            // only received - never sent
            break;
        
        case ESPNOW_DATA_DYNAMIC:
            // only received - never sent
            break;
        
        case ESPNOW_DATA_CONTROL:
            //!first message --> set alerts limits!
            if ((p->type == PAD) && (pads_status[p->id - 1] == PAD_DISCONNECTED))
            {
                buf->field_1 = OVERCURRENT_TX;
                buf->field_2 = OVERVOLTAGE_TX;
                buf->field_3 = OVERTEMPERATURE_TX;
                buf->field_4 = FOD_ACTIVE;
                ESP_LOGW(TAG, "SETTING LOCAL LIMITS");
            }
            else if ((p->type == SCOOTER) && (scooters_status[p->id - NUMBER_TX - 1] == SCOOTER_DISCONNECTED))
            {
                ESP_LOGW(TAG, "SETTING REMOTE LIMITS");
                buf->field_1 = OVERCURRENT_RX;
                buf->field_2 = OVERVOLTAGE_RX;
                buf->field_3 = OVERTEMPERATURE_RX;
                buf->field_4 = MIN_VOLTAGE;
            }
            else //!ANY OTHER MSG - SWITCH ON/OFF
            {
                //for the alerts 
                buf->field_1 = p->full_power;
                buf->field_2 = p->low_power;
                buf->field_3 = buf->field_4 = 0;
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
    static float start = 0;
    static float stop = 1;

    //ALL 0 --> START
    if ((data->field_1 == start) && (data->field_2 == start) && (data->field_3 == start) && (data->field_4 == start))
        return LOCALIZATION_START;
    //ALL 1 --> STOP
    else if ((data->field_1 == stop) && (data->field_2 == stop) && (data->field_3 == stop) && (data->field_4 == stop))
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
    bool all_scooter_checked = true;
    //checking there is no scooter to be checked with the current pad yet
    for (uint8_t i = 0; i < NUMBER_RX; i++)
    {
        if (scooter_tobechecked[i])
            all_scooter_checked = false;
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
        static bool baton_passed = false;
        
        while(!baton_passed)
        {
            if (pads_status[Baton] == PAD_LOW_POWER)
            {
                //switch off the previous pad
                struct peer *p = peer_find(Baton + 1);
                if (p == NULL)
                {
                    ESP_LOGE(TAG, "Peer not found");
                    return 0;
                }
                p->low_power = 0;
                pads_status[Baton] = PAD_CONNECTED;
                espnow_data_prepare(loc_data, ESPNOW_DATA_CONTROL, p->id);
                if(esp_now_send(p->mac, (uint8_t *)loc_data, sizeof(espnow_data_t)) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error");
                    return 0;
                }

            }
            else if ((pads_status[Baton] == PAD_CONNECTED) && (loc_pads_off()))
            {                
                //switch on the new pad
                struct peer *p = peer_find(Baton + 1);
                if (p == NULL)
                {
                    ESP_LOGE(TAG, "Peer not found");
                    return 0;
                }
                p->low_power = 1;
                pads_status[Baton] = PAD_LOW_POWER;
                espnow_data_prepare(loc_data, ESPNOW_DATA_CONTROL, p->id);
                if(esp_now_send(p->mac, (uint8_t *)loc_data, sizeof(espnow_data_t)) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error");
                    return 0;
                }
                baton_passed = true;
                ESP_LOGW(TAG, "BATON PASSED TO %d", Baton +1);
            }

            Baton = (Baton + 1) % NUMBER_TX;
            
        }

        return 1;
    }

    return 0;
}


static void localization_task(void *pvParameter)
{
    localization_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
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

        // check which message type we received
        uint8_t type = parse_localization_message(recv_data);

        switch(type)
        {
            case LOCALIZATION_START:
                ESP_LOGW(TAG, "LOCALIZATION START");

                if (pass_the_baton(localization_data))
                    scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = true;

                // send command to get voltage after a minimum time
                espnow_data_prepare(localization_data, ESPNOW_DATA_LOCALIZATION, recv_data->id);
                if(esp_now_send(recv_cb->mac_addr, (uint8_t *)localization_data, sizeof(espnow_data_t)) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error");
                }
                
                break;
            
            case LOCALIZATION_CHECK:
                ESP_LOGW(TAG, "LOCALIZATION CHECK");

                scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = false;
                
                if (pass_the_baton(localization_data))
                    scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = true;

                // send command to get voltage after a minimum time
                espnow_data_prepare(localization_data, ESPNOW_DATA_LOCALIZATION, recv_data->id);
                if(esp_now_send(recv_cb->mac_addr, (uint8_t *)localization_data, sizeof(espnow_data_t)) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error");
                }
                
                break;

            case LOCALIZATION_STOP: //voltage thresh is passed - position found
                ESP_LOGW(TAG, "LOCALIZATION STOPPED");
                scooter_tobechecked[recv_data->id - NUMBER_TX - 1] = false;
                //find which pad is on
                uint8_t pad_positition = find_pad_position();
                struct peer *p = peer_find(pad_positition + 1);
                if (p == NULL)
                {
                    ESP_LOGE(TAG, "Peer not found!");
                    break;
                }
                p->low_power = 0;
                p->full_power = 1;
                pads_status[pad_positition] = PAD_FULL_POWER;
                scooters_status[recv_data->id - NUMBER_TX - 1] = SCOOTER_CHARGING;
                ESP_LOGW(TAG, "SCOOTER %d CHARGING ON PAD %d", recv_data->id - NUMBER_TX, pad_positition + 1);
                espnow_data_prepare(localization_data, ESPNOW_DATA_CONTROL, p->id);
                if(esp_now_send(p->mac, (uint8_t *)localization_data, sizeof(espnow_data_t)) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error");
                    break;
                }
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
                    //todo: if status != 0, then resend data
                    //todo: check retransmission count
                }
                else
                {
                    // never broadcast --> check only for unicast
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
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) 
                    {
                        //if its a scooter, there must be at least one pad connected!
                        if ((unitID > NUMBER_TX) && (!connected_pads))
                            break;
                        
                        //avoid encryption for the first message (needs to be registered on both sides)
                        esp_now_register_peer(recv_cb->mac_addr);

                        // save its details into peer structure 
                        peer_add(unitID, recv_cb->mac_addr);
                        struct peer *p = peer_find(unitID);
                        ESP_LOGW(TAG, "NEW PEER FOUND! ID: %d", unitID);

                        //Send unicast data to make him stop sending broadcast msg
                        espnow_data_prepare(espnow_data, ESPNOW_DATA_CONTROL, unitID);
                        if (esp_now_send(recv_cb->mac_addr, (uint8_t *)espnow_data, sizeof(espnow_data_t)) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                        }
                        // then encrypt for future messages
                        esp_now_encrypt_peer(recv_cb->mac_addr);

                        if (p->id > NUMBER_TX)   
                            scooters_status[unitID - NUMBER_TX - 1] = SCOOTER_CONNECTED;
                        else
                            pads_status[unitID - 1] = PAD_CONNECTED;
                    }

                }
                else if(addr_type == ESPNOW_DATA_DYNAMIC)
                {
                    ESP_LOGI(TAG, "Receive DYNAMIC data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                                  
                    // save its details into peer structure
                    // check it was different from the previous one
                    // if yes, send it to the queue for the dashboard publication
                }
                else if (addr_type == ESPNOW_DATA_ALERT)
                {
                    ESP_LOGI(TAG, "Receive ALERT data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    // send it to the queue for the dashboard publication
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

static esp_err_t espnow_init(void)
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
    
    rc = xTaskCreate(espnow_task, "espnow_task", 2048, buffer, 4, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Create espnow_task fail");
        return ESP_FAIL;
    }

    rc = xTaskCreate(localization_task, "localization_task", 2048, loc_buffer, 8, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Create localization_task fail");
        return ESP_FAIL;
    }

    // free the buffers at exit //!needs to call a function
    //atexit(free(buffer));
    //atexit(free(loc_buffer));

    return ESP_OK;
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
    ESP_LOGE(TAG, "length of espnow_data_t: %d", sizeof(espnow_data_t));

    wifi_init();
    espnow_init();
    peer_init(NUMBER_RX + NUMBER_TX);
    //todo: temperature sensor init
    //todo: sd card init

    //todo: create a task for sending measurements to the dashboard (only when they change more than a delta)
}
