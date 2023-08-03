#include "espnow.h"
#include "led_strip.h"
#include "aux_ctu_hw.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "MAIN";

static QueueHandle_t espnow_queue;
static uint8_t master_mac[ESP_NOW_ETH_ALEN] = { 0 };

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
esp_now_peer_num_t peer_num = {0, 0};

wpt_dynamic_payload_t dynamic_payload;
wpt_alert_payload_t alert_payload;

TimerHandle_t dynamic_timer = NULL;
TimerHandle_t alert_timer = NULL;

void espnow_data_prepare(espnow_data_t *buf, uint8_t *mac_addr, uint8_t loc);


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

void dynamic_timer_callback(TimerHandle_t xTimer)
{
    //save values to peer structure
    //todo: to be done into the hardware readings
    dynamic_payload.vrect = 0;
    dynamic_payload.irect = 0;
    dynamic_payload.temp1 = 0;
    dynamic_payload.temp2 = 0;
    //send them to master //
    espnow_data_t *buf = malloc(sizeof(espnow_data_t));
    if (buf == NULL) {
        ESP_LOGE(TAG, "Malloc  buffer fail");
        free(buf);
        return;
    }
    espnow_data_prepare(buf, master_mac, 0);
    if (esp_now_send(master_mac, (uint8_t *) buf, sizeof(espnow_data_t)) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
    }

    free(buf);
}

void alert_timer_callback(TimerHandle_t xTimer)
{
    //check if values are in range
    //if not, send alert to master

}

static void esp_now_register_master(uint8_t *mac_addr, bool encrypt)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
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
    //todo: make localization commands asyncronous
    //todo: send to normal queue every other command
    //todo: check the localization field into the received data and send it to the localization queue
    if (xQueueSend(espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
uint8_t espnow_data_parse(uint8_t *data, uint16_t data_len)
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
void espnow_data_prepare(espnow_data_t *buf, uint8_t *mac_addr, uint8_t loc)
{
    //initiliaze buf to 0
    memset(buf, 0, sizeof(espnow_data_t));
    // esp NOW data
    buf->id = UNIT_ROLE;
    buf->type = IS_BROADCAST_ADDR(mac_addr) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    
    //todo:localizarion data 

    //todo: measurements data
    buf->voltage = 2.2;
    //todo: alerts data

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
                addr_type = IS_BROADCAST_ADDR(send_cb->mac_addr) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
                esp_now_get_peer_num(&peer_num);

                ESP_LOGI(TAG, "Send data to "MACSTR". status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (addr_type == ESPNOW_DATA_BROADCAST)
                {
                    // if the master (encrypted) is registered, then stop sending broadcast data
                    if ((peer_num.encrypt_num) && (addr_type == ESPNOW_DATA_BROADCAST))
                    {
                        ESP_LOGI(TAG, "Stop sending broadcast data");
                        break;
                    }

                    /* Otherwise, Delay a while before sending the next data. */
                    vTaskDelay(BROADCAST_TIMEGAP);
                    
                    espnow_data_prepare(espnow_data, send_cb->mac_addr, 0);

                    /* Send the next data after the previous data is sent. */
                    if (esp_now_send(send_cb->mac_addr, (uint8_t *) espnow_data, sizeof(espnow_data_t)) != ESP_OK) {
                        ESP_LOGE(TAG, "Send error");
                    }
                } 
                else
                {
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
                if(!espnow_data_parse(recv_cb->data, recv_cb->data_len))
                {
                    ESP_LOGE(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                    break;
                }

                espnow_data_t *recv_data = (espnow_data_t *)recv_cb->data;
                addr_type = recv_data->type;
                //unitID probably not needed
                free(recv_cb->data);

                if (addr_type == ESPNOW_DATA_BROADCAST) 
                {
                    ESP_LOGI(TAG, "Receive broadcast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    //! ignore it as it comes from other peers
                }
            
                else if (addr_type == ESPNOW_DATA_UNICAST) 
                {
                    //todo: make switch case for different types of data (control highest priority) (localization for CRU highest priority)
                    ESP_LOGI(TAG, "Receive unicast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    //* Add MASTER 
                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) 
                    {
                        esp_now_register_master(recv_cb->mac_addr, true);
                        memcpy(master_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_LOGI(TAG, "Add master "MACSTR" to peer list", MAC2STR(master_mac));
                        //!start sending measurements data to the master
                        if (xTimerStart(dynamic_timer, 0) != pdPASS) {
                            ESP_LOGE(TAG, "Cannot start dynamic timer");
                        }
                        if (xTimerStart(alert_timer, 0) != pdPASS) {
                            ESP_LOGE(TAG, "Cannot start alert timer");
                        }
                    }
                    
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
        //todo: free espnow_data
    }
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
    espnow_data_prepare(buffer, broadcast_mac, 0);

    // create a task to handle espnow events
    xTaskCreate(espnow_task, "espnow_task", 2048, buffer, 4, NULL);

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

    //todo: initialize hardware
    //todo: start periodic alerts checking
    wifi_init();
    espnow_init();

    // freeRTOS timers
    dynamic_timer = xTimerCreate("dynamic_timer", DYNAMIC_TIMEGAP, pdTRUE, NULL, dynamic_timer_callback);
    ESP_ERROR_CHECK( dynamic_timer == NULL ? ESP_FAIL : ESP_OK);
    alert_timer = xTimerCreate("alert_timer", ALERT_TIMEGAP, pdTRUE, NULL, alert_timer_callback);
    ESP_ERROR_CHECK( alert_timer == NULL ? ESP_FAIL : ESP_OK);
    //todo: another timer to make readings and save values to peer structure

    //ESP_NOW_DEINIT()
    //WIFI_DEINIT()
}
