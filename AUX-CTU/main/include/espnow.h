#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#define UNIT_ROLE PAD1

#define ESPNOW_QUEUE_SIZE           10
#define BROADCAST_TIMEGAP           pdMS_TO_TICKS(1000)
#define ALERT_TIMEGAP               pdMS_TO_TICKS(100)


#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

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
    MASTER,
    PAD1,
    PAD2,
    PAD3,
    PAD4,
    SCOOTER1,
    SCOOTER2,
    SCOOTER3,
    SCOOTER4,
} peer_id;

typedef enum {
    PAD_DISCONNECTED,       //when the pad is not connected
    PAD_CONNECTED,          //when the pad is connected 
    PAD_LOW_POWER,          //when the pad is on low power mode
    PAD_FULL_POWER,         //when the pad is on full power mode
    PAD_FULLY_CHARGED       //when the pad is off but a fully charged scooter is still present on it
} pad_status_t;

/** @brief Dynamic characteristic structure. This contains elements necessary for dynamic payload. */
typedef struct
{
    float             voltage;            /**< Voltage value from I2C (4 bytes). */
    float             current;            /**< Current value from I2C (4 bytes). */
    float             temp1;              /**< Temperature value from I2C (4 bytes). */
    float             temp2;              /**< Temperature value from I2C (4 bytes). */
} wpt_dynamic_payload_t;

/**@brief Alert characteristic structure. This contains elements necessary for alert payload. */
/* The union structure allows to check only 'internal', as it gets positive as soon as at one field of the struct is 1 */
typedef union
{
	struct {
		uint8_t           overtemperature:1;    
		uint8_t           overcurrent:1;        
		uint8_t           overvoltage:1;        
   		uint8_t           FOD:1;                
	};
	uint8_t internal;
} wpt_alert_payload_t;

/* ESP NOW PAYLOAD */
typedef struct { 
    uint8_t id;                           //Peer unit ID.
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    float field_1;                        
    float field_2;                        
    float field_3;                        
    float field_4;                        
} __attribute__((packed)) espnow_data_t;

#endif
