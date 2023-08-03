#ifndef PEER_H
#define PEER_H

#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include "esp_log.h"
#include "espnow.h"

#define MAX_PEERS 8
#define NUMBER_TX 4

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
    float             vrect;              /**< [mandatory] Vrect value from I2C (4 bytes). */
    float             irect;              /**< [mandatory] Irect value from I2C (4 bytes). */
    float             temp1;              /**< [optional] Temperature value from I2C (4 bytes). */
    float             temp2;              /**< [optional] Temperature value from I2C (4 bytes). */
	uint8_t	          alert;		      /**< CRU alert field for warnings to send to CTU (1 byte). */
    float             rx_power;           /* Calculate received power if peer is CRU */
    float             tx_power;           /* Calculate transmitted power if peer is CTU */
    struct tm         dyn_time;           /** Time */
} wpt_dynamic_payload_t;


struct peer 
{
    SLIST_ENTRY(peer) next;

    peer_id id;
    uint8_t mac[6];

    /* POSITION OF THE PAD UPON WHICH THE PEER IS PLACED */
    int8_t position;

    /** Peripheral payloads. */
    wpt_dynamic_payload_t dyn_payload;
};




//* Functions to handle peer structures saved into a memory pool */
uint8_t peer_delete(peer_id id);

uint8_t peer_add(peer_id id, uint8_t *mac);

void peer_init(uint8_t max_peers);

struct peer * peer_find(peer_id id);

#endif // PEER_H