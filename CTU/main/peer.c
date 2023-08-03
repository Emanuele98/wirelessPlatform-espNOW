#include "peer.h"


static const char *TAG = "PEER";

static SLIST_HEAD(, peer) peers;

struct peer * peer_find(peer_id id)
{
    struct peer *p;

    SLIST_FOREACH(p, &peers, next) {
        if (p->id == id) {
            return p;
        }
    }

    return NULL;
}

uint8_t peer_delete(peer_id id)
{
    struct peer *p;

    SLIST_FOREACH(p, &peers, next) {
        if (p->id == id) {
            SLIST_REMOVE(&peers, p, peer, next);
            free(p);
            return 0;
        }
    }

    return 1;
}

void delete_all_peers(void)
{
    struct peer *p;

    while (!SLIST_EMPTY(&peers)) {
        p = SLIST_FIRST(&peers);
        SLIST_REMOVE_HEAD(&peers, next);
        free(p);
    }
}

uint8_t peer_add(peer_id id, uint8_t *mac)
{
    struct peer *p;

    /*Make sure the peer does not exist in the memory pool yet*/
    p = peer_find(id);
    if (p) 
    {
        ESP_LOGE(TAG, "Peer already exists in the memory pool");
        return 1;
    }

    /*Allocate memory for the peer*/
    p = malloc(sizeof * p);
    if (!p) 
    {
        ESP_LOGE(TAG, "Failed to allocate memory for peer");
        return 1;
    }

    /* Set everything to 0 */
    memset(p, 0, sizeof * p);

    p->id = id;
    memcpy(p->mac, mac, ESP_NOW_ETH_ALEN);
    if (id <= NUMBER_TX) //if peer is a TX UNIT
        p->position = id;
    else        //if peer is a RX UNIT
        p->position = 0;
    
    SLIST_INSERT_HEAD(&peers, p, next);

    return 0;
}

void peer_init(uint8_t max_peers)
{
    SLIST_INIT(&peers);

    //register free memory function at the end of the program
    atexit(delete_all_peers);
}


