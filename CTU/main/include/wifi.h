#ifndef WIFI_H
#define WIFI_H

//mqtt
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"

#include <math.h>

#include "espnow.h"
#include "peer.h"

#define MQTT_QoS              1         //MQTT Quality of Service
#define MQTT_MIN_DELTA        0.2       //minimum delta to publish a new value
#define MQTT_REFRESH_LIMIT    30        //mqqt is updated AT LEAST every 30 dynamic received messages


void wifi_init(void);

void mqtt_unit_reset(uint8_t id);






#endif // WIFI_H