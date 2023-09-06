#include "peer.h"


static const char *TAG = "PEER";

static SLIST_HEAD(, peer) peers;

//Whether at least one TX unit is already connected
extern bool connected_pads;

struct peer * peer_find_by_id(peer_id id)
{
    struct peer *p;

    SLIST_FOREACH(p, &peers, next) {
        if (p->id == id) {
            return p;
        }
    }

    return NULL;
}

struct peer * peer_find_by_mac(uint8_t *mac)
{
    struct peer *p;

    SLIST_FOREACH(p, &peers, next) {
        if (memcmp(p->mac, mac, ESP_NOW_ETH_ALEN) == 0) {
            return p;
        }
    }

    return NULL;
}

struct peer * peer_find_by_position(int8_t position)
{
    struct peer *p;

    SLIST_FOREACH(p, &peers, next) {
        if ((p->position == position) && (p->type == SCOOTER))
            return p;
    }

    return NULL;

}


uint8_t peer_add(peer_id id, uint8_t *mac)
{
    struct peer *p;

    /*Make sure the peer does not exist in the memory pool yet*/
    p = peer_find_by_id(id);
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
    {
        p->position = id;
        connected_pads = true;
        p->type = PAD;
    }
    else        //if peer is a RX UNIT
    {
        p->position = 0;
        p->type = SCOOTER;
    }
    
    SLIST_INSERT_HEAD(&peers, p, next);

    return 0;
}

void peer_init(uint8_t max_peers)
{
    SLIST_INIT(&peers);
    connected_pads = false;

    //register free memory function at the end of the program
    atexit(delete_all_peers);
}

void peer_delete(peer_id id)
{
    struct peer *p = peer_find_by_id(id);
    if(p == NULL)
    {
        ESP_LOGE(TAG, "Peer cannot be deleted - not found");
        return;
    }

    // remove it from the list of espnow connections
    esp_now_del_peer(p->mac);

    //remove it from the list of saved peers
    SLIST_REMOVE(&peers, p, peer, next);
    free(p);
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

