#ifndef _PEER_H_
#define _PEER_H_

#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include "esp_log.h"
#include "espnow.h"

#define MAX_PEERS 8
#define NUMBER_TX 4
#define RSSI_LIMIT -80

typedef enum {
    MASTER,
    PAD1,
    PAD2,
    PAD3,
    PAD4,
    SCOOTER1,
    SCOOTER2,
    SCOOTER3,
    SCOOTER4
} peer_id;

/** @brief Dynamic characteristic structure. This contains elements necessary for dynamic payload. */
typedef struct
{
    float             voltage;            /**< Voltage value from I2C (4 bytes). */
    float             current;            /**< Current Irect value from I2C (4 bytes). */
    float             temp1;              /**< Temperature value from I2C (4 bytes). */
    float             temp2;              /**< Temperature value from I2C (4 bytes). */
    float             rx_power;           /* Calculate received power if peer is CRU */
    float             tx_power;           /* Calculate transmitted power if peer is CTU */
    struct tm         dyn_time;           /** Time */
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

struct peer 
{
    SLIST_ENTRY(peer) next;

    peer_id id;
    uint8_t mac[6];

    /* STATUS ON/OFF */
    bool full_power;

    /* LOCALIZATION STATUS ON/OFF */
    bool low_power;
    
    /* IS THERE A FULLY CHARGED SCOOTER ON THE PAD? */
    bool fully_charged;

    /* POSITION OF THE PAD UPON WHICH THE PEER IS PLACED */
    int8_t position;

    /** Peripheral payloads. */
    wpt_dynamic_payload_t dyn_payload;
    wpt_alert_payload_t  alert_payload;
};




//* Functions to handle peer structures saved into a memory pool */
uint8_t peer_delete(peer_id id);

uint8_t peer_add(peer_id id, uint8_t *mac);

void peer_init(uint8_t max_peers);

struct peer * peer_find(peer_id id);

#endif // _PEER_H_