# ESP-NOW COMMS PROTOCOL - BUMBLEBEE

This protocol is comprised of 3 units:
-   *CTU* (master)          [1]
-   *CRU* (kick scooters) [n]
-   *AUX-CTU* (ground pads)  [n]

The platform would be able to host simultaneously 10 (n) electric kick scooters: localize their position, enable the wireless charging until they are fully charged, and constantly monitoring their alerts threshold (overcurrent, overvoltage, overtemperature, and Foreign Object Detection).



## Main Features

-   *Need to declare one and one only WiFi Channel*: the router must be costumizable in order to allow both WiFi and ESPNOW protocols to coexist as they exploit the same physical stack;
-   *Security*: ESPNOW uses an encryption protocol called CCMP, also, every message is checked with CRC16;
-   *Alerts taken care locally*: then the master gets to know them (when its already safe);
-   *Retransmission*: every message is considered crucial. If the sending is not successfull, the unit is trying again for `MAX_COMMS_ERROR`. If still unsuccessfull, the unit must reset;
- *Remote monitoring*: Values can be seen in a NodeRed dashboard here http://warwick.zapto.org:1880/ui/#!/0?socketid=XoAqc-kbcSgZZ-wwAAmB. They are updated at least every `MQTT_REFRESH_LIMIT`


## Handshake
-   Initially, the peers are broadcasting 
-   Then, the master replies seeting alerts limits and the cadence at which the peer should send its sensor measurements
-   Both sides save the respecitive MAC addresses, dynamically, and start the encrypted exchange of messages.

## MESSAGE TYPE
```
/* ESP NOW PAYLOAD */
typedef struct { 
    uint8_t id;                           //Peer unit ID.
    uint8_t type;                         //Type of ESPNOW message.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    float field_1;                        
    float field_2;                        
    float field_3;                        
    float field_4;                        
} __attribute__((packed)) espnow_data_t;
```

The payload has been declared as small as possible (20 bytes). The 4 fields float value get different meaning based on the type of ESPNOW message.
The types are defined like:
```
typedef enum {
    ESPNOW_DATA_BROADCAST,              //peer advertising to the master
    ESPNOW_DATA_LOCALIZATION,           //bidirectional. Detect the scooter position
    ESPNOW_DATA_ALERT,                  //peer signals an alert. Master declares a peer reboot                              
    ESPNOW_DATA_DYNAMIC,                //peer sends sensor measurements
    ESPNOW_DATA_CONTROL                 //master control the on/off and LED status of the peers
} message_type;
```

## ALERTS HANDLING
The alerts are taken care with the help of the accelerometer in the scooter.
-   Alerts from pad: the pad will stay disconnected for a defined time, as well as the relative scooter if it does not move sooner than that time. If it moves, a reset will happen and a new localization process will start;
- Alerts from the scooter: the scooter stay disconnected for a defined time. The relative pad stay disconnected as well for the same time. However, it becomes available if the scooter moves earlier making it free again;
- Fully charged is treated as an alert from the scooter thus it behaves accordingly. The only difference will be noticed in the LEDs status.




## Installation

Making any unit work with an ESP32 chip requires a few steps. They are provided by the "Get Started" section of the ESP-IDF documentation at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html. It is important to take into account that the CTU has been tested with the *IDF v5.1* which can be found at https://github.com/espressif/esp-idf.

Every unit has its unique *identifier*, which needs to be changed in esp_now.h:`UNIT_ROLE`.
    