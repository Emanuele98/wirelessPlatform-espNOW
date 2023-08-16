#ifndef ESPNOW_H
#define ESPNOW_H

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "peer.h"

#define UNIT_ROLE MASTER


#define ESPNOW_QUEUE_SIZE           20
#define ESPNOW_MAXDELAY             pdMS_TO_TICKS(1000)


#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


typedef enum {
    ID_ESPNOW_SEND_CB,
    ID_ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

typedef enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_LOCALIZATION,
    ESPNOW_DATA_ALERT,
    ESPNOW_DATA_DYNAMIC,
    ESPNOW_DATA_CONTROL
} message_type;

typedef enum {
    LOCALIZATION_START,
    LOCALIZATION_CHECK,
    LOCALIZATION_STOP
} localization_message_type;

/* User defined field of ESPNOW data in this example. */
//todo: define the meaning of each field based on the message type
typedef struct { 
    uint8_t id;                           //Peer unit ID.
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    float field_1;                        
    float field_2;                        
    float field_3;                        
    float field_4;                        
} __attribute__((packed)) espnow_data_t;

#endif // ESPNOW_H
