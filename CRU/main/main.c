#include "espnow.h"
#include "cru_hw.h"
#include "lis3dh.h"


/* -- use following constants to define the example mode ----------- */

// #define SPI_USED     // SPI interface is used, otherwise I2C
// #define FIFO_MODE    // multiple sample read mode
// #define INT_DATA     // data interrupts used (data ready and FIFO status)
#define INT_EVENT    // inertial event interrupts used (wake-up, free fall or 6D/4D orientation)
// #define INT_CLICK    // click detection interrupts used

#if defined(INT_DATA) || defined(INT_EVENT) || defined(INT_CLICK)
#define INT_USED
#endif

/** -- platform dependent definitions ------------------------------ */
// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

// SPI interface definitions for ESP32
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO  16
#define SPI_MOSI_GPIO 17
#define SPI_MISO_GPIO 18
#define SPI_CS_GPIO   19

// I2C interface defintions for ESP32 and ESP8266
#define I2C_BUS       0
#define I2C_SCL_PIN   14
#define I2C_SDA_PIN   13
#define I2C_FREQ      I2C_FREQ_100K

// interrupt GPIOs defintions for ESP8266 and ESP32
#define INT1_PIN      5
#define INT2_PIN      4

static lis3dh_sensor_t* sensor;

/**
 * Common function used to get sensor data.
 */
void read_data ()
{
    #ifdef FIFO_MODE

    lis3dh_float_data_fifo_t fifo;

    if (lis3dh_new_data (sensor))
    {
        uint8_t num = lis3dh_get_float_data_fifo (sensor, fifo);

        printf("%.3f LIS3DH num=%d\n", (double)sdk_system_get_time()*1e-3, num);

        for (int i=0; i < num; i++)
            // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
            printf("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
                   (double)sdk_system_get_time()*1e-3, 
                   fifo[i].ax, fifo[i].ay, fifo[i].az);
    }

    #else

    lis3dh_float_data_t  data;

    if (lis3dh_new_data (sensor) &&
        lis3dh_get_float_data (sensor, &data))
        // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
        printf("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
               (double)sdk_system_get_time()*1e-3, 
                data.ax, data.ay, data.az);
        
    #endif // FIFO_MODE
}


#ifdef INT_USED
/**
 * In this case, any of the possible interrupts on interrupt signal *INT1* is
 * used to fetch the data.
 *
 * When interrupts are used, the user has to define interrupt handlers that
 * either fetches the data directly or triggers a task which is waiting to
 * fetch the data. In this example, the interrupt handler sends an event to
 * a waiting task to trigger the data gathering.
 */

static xQueueHandle gpio_evt_queue = NULL;

// User task that fetches the sensor values.

void user_task_interrupt (void *pvParameters)
{
    uint8_t gpio_num;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            lis3dh_int_data_source_t  data_src  = {};
            lis3dh_int_event_source_t event_src = {};
            lis3dh_int_click_source_t click_src = {};

            // get the source of the interrupt and reset *INTx* signals
            #ifdef INT_DATA
            lis3dh_get_int_data_source  (sensor, &data_src);
            #endif
            #ifdef INT_EVENT
            lis3dh_get_int_event_source (sensor, &event_src, lis3dh_int_event1_gen);
            #endif
            #ifdef INT_CLICK
            lis3dh_get_int_click_source (sensor, &click_src);
            #endif
    
            // in case of DRDY interrupt or inertial event interrupt read one data sample
            if (data_src.data_ready)
                read_data ();
   
            // in case of FIFO interrupts read the whole FIFO
            else  if (data_src.fifo_watermark || data_src.fifo_overrun)
                read_data ();
    
            // in case of event interrupt
            else if (event_src.active)
            {
                printf("%.3f LIS3DH ", (double)sdk_system_get_time()*1e-3);
                if (event_src.x_low)  printf("x is lower than threshold\n");
                if (event_src.y_low)  printf("y is lower than threshold\n");
                if (event_src.z_low)  printf("z is lower than threshold\n");
                if (event_src.x_high) printf("x is higher than threshold\n");
                if (event_src.y_high) printf("y is higher than threshold\n");
                if (event_src.z_high) printf("z is higher than threshold\n");
            }

            // in case of click detection interrupt   
            else if (click_src.active)
               printf("%.3f LIS3DH %s\n", (double)sdk_system_get_time()*1e-3, 
                      click_src.s_click ? "single click" : "double click");
            
            static uint16_t *adc1, *adc2, *adc3;
            bool ret = lis3dh_get_adc(sensor, adc1, adc2, adc3);
        }
    }
}

// Interrupt handler which resumes user_task_interrupt on interrupt

void IRAM int_signal_handler (uint8_t gpio)
{
    // send an event with GPIO to the interrupt user task
    xQueueSendFromISR(gpio_evt_queue, &gpio, NULL);
}

#else // !INT_USED

/*
 * In this example, user task fetches the sensor values every seconds.
 */

void user_task_periodic(void *pvParameters)
{
    vTaskDelay (100/portTICK_PERIOD_MS);
    
    while (1)
    {
        // read sensor data
        read_data ();
        
        // passive waiting until 1 second is over
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

#endif // INT_USED

void accelerometer_init(void)
{
    #ifdef SPI_USED

    // init the sensor connnected to SPI
    spi_bus_init (SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);

    // init the sensor connected to SPI_BUS with SPI_CS_GPIO as chip select.
    sensor = lis3dh_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    
    #else

    // init all I2C bus interfaces at which LIS3DH  sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    
    // init the sensor with slave address LIS3DH_I2C_ADDRESS_1 connected to I2C_BUS.
    sensor = lis3dh_init_sensor (I2C_BUS, LIS3DH_I2C_ADDRESS_2, 0);

    #endif
    
    if (sensor) 
    {
        #ifdef INT_USED

        /** --- INTERRUPT CONFIGURATION PART ---- */
        
        // Interrupt configuration has to be done before the sensor is set
        // into measurement mode to avoid losing interrupts

        // create an event queue to send interrupt events from interrupt
        // handler to the interrupt task
        gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));

        // configure interupt pins for *INT1* and *INT2* signals and set the interrupt handler
        gpio_enable(INT1_PIN, GPIO_INPUT);
        gpio_set_interrupt(INT1_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);

        #endif  // INT_USED
        
        /** -- SENSOR CONFIGURATION PART --- */

        // set polarity of INT signals if necessary
        // lis3dh_config_int_signals (sensor, lis3dh_high_active);

        #ifdef INT_DATA
        // enable data interrupts on INT1 (data ready or FIFO status interrupts)
        // data ready and FIFO status interrupts must not be enabled at the same time
        #ifdef FIFO_MODE
        lis3dh_enable_int (sensor, lis3dh_int_fifo_overrun  , lis3dh_int1_signal, true);
        lis3dh_enable_int (sensor, lis3dh_int_fifo_watermark, lis3dh_int1_signal, true);
        #else
        lis3dh_enable_int (sensor, lis3dh_int_data_ready, lis3dh_int1_signal, true);
        #endif // FIFO_MODE
        #endif // INT_DATA
        
        #ifdef INT_EVENT
        // enable data interrupts on INT1 
        lis3dh_int_event_config_t event_config;
    
        event_config.mode = lis3dh_wake_up;
        // event_config.mode = lis3dh_free_fall;
        // event_config.mode = lis3dh_6d_movement;
        // event_config.mode = lis3dh_6d_position;
        // event_config.mode = lis3dh_4d_movement;
        // event_config.mode = lis3dh_4d_position;
        event_config.threshold = 10;
        event_config.x_low_enabled  = false;
        event_config.x_high_enabled = true;
        event_config.y_low_enabled  = false;
        event_config.y_high_enabled = true;
        event_config.z_low_enabled  = false;
        event_config.z_high_enabled = true;
        event_config.duration = 0;
        event_config.latch = true;
        
        lis3dh_set_int_event_config (sensor, &event_config, lis3dh_int_event1_gen);
        lis3dh_enable_int (sensor, lis3dh_int_event1, lis3dh_int1_signal, true);
        #endif // INT_EVENT

        #ifdef INT_CLICK
        // enable click interrupt on INT1
        lis3dh_int_click_config_t click_config;
        
        click_config.threshold = 10;
        click_config.x_single = false;
        click_config.x_double = false;        
        click_config.y_single = false;
        click_config.y_double = false;        
        click_config.z_single = true;
        click_config.z_double = false;
        click_config.latch = true;
        click_config.time_limit   = 1;
        click_config.time_latency = 1;
        click_config.time_window  = 3;
        
        lis3dh_set_int_click_config (sensor, &click_config);
        lis3dh_enable_int (sensor, lis3dh_int_click, lis3dh_int1_signal, true);
        #endif // INT_CLICK

        #ifdef FIFO_MODE
        // clear FIFO and activate FIFO mode if needed
        lis3dh_set_fifo_mode (sensor, lis3dh_bypass,  0, lis3dh_int1_signal);
        lis3dh_set_fifo_mode (sensor, lis3dh_stream, 10, lis3dh_int1_signal);
        #endif

        // configure HPF and reset the reference by dummy read
        lis3dh_config_hpf (sensor, lis3dh_hpf_normal, 0, true, true, true, true);
        lis3dh_get_hpf_ref (sensor);
        
        // enable ADC inputs and temperature sensor for ADC input 3
        lis3dh_enable_adc (sensor, true, true);
        
        // LAST STEP: Finally set scale and mode to start measurements
        lis3dh_set_scale(sensor, lis3dh_scale_2_g);
        lis3dh_set_mode (sensor, lis3dh_odr_10, lis3dh_high_res, true, true, true);

        /** -- TASK CREATION PART --- */

        // must be done last to avoid concurrency situations with the sensor
        // configuration part

        #ifdef INT_USED

        // create a task that is triggered only in case of interrupts to fetch the data
        xTaskCreate(user_task_interrupt, "user_task_interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);
        
        #else // INT_USED

        // create a user task that fetches data from sensor periodically
        xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);

        #endif
    }
    else
        printf("Could not initialize LIS3DH sensor\n");
}




static const char *TAG = "MAIN";

SemaphoreHandle_t send_semaphore = NULL;
static QueueHandle_t espnow_queue;
static uint8_t master_mac[ESP_NOW_ETH_ALEN] = { 0 };

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
esp_now_peer_num_t peer_num = {0, 0};

wpt_dynamic_payload_t dynamic_payload;
wpt_alert_payload_t alert_payload;

TimerHandle_t dynamic_timer, alert_timer;
message_type last_msg_type;
SemaphoreHandle_t i2c_sem;

scooter_status_t scooter_status = SCOOTER_DISCONNECTED;

//ALERTS LIMITS
float OVERCURRENT = 0;
float OVERTEMPERATURE = 0;
float OVERVOLTAGE = 0;
float MIN_VOLTAGE = 0;

static uint8_t alert_count = 0;
float voltage_window[AVG_ALERT_WINDOW], current_window[AVG_ALERT_WINDOW], temp1_window[AVG_ALERT_WINDOW], temp2_window[AVG_ALERT_WINDOW];
uint16_t fully_charged;


int i2c_master_port;
i2c_config_t conf;

static uint8_t comms_fail = 0;

void espnow_data_prepare(espnow_data_t *buf, message_type type);


/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

void dynamic_timer_callback(TimerHandle_t xTimer)
{
    //send them to master //
    espnow_data_t *buf = malloc(sizeof(espnow_data_t));
    if (buf == NULL) {
        ESP_LOGE(TAG, "Malloc  buffer fail");
        free(buf);
        return;
    }
    
    if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
    {
        espnow_data_prepare(buf, ESPNOW_DATA_DYNAMIC);
        esp_now_send(master_mac, (uint8_t *) buf, sizeof(espnow_data_t));
    } else  
        ESP_LOGE(TAG, "Could not take send semaphore");

    free(buf);
}

void alert_timer_callback(TimerHandle_t xTimer)
{
    espnow_data_t *buf = malloc(sizeof(espnow_data_t));
    if (buf == NULL) {
        ESP_LOGE(TAG, "Malloc  buffer fail");
        free(buf);
        return;
    }

    //make an average window about the last 10 measurements
    voltage_window[alert_count] = dynamic_payload.voltage;
    current_window[alert_count] = dynamic_payload.current;
    temp1_window[alert_count] = dynamic_payload.temp1;
    temp2_window[alert_count] = dynamic_payload.temp2;

    // alert count keeps going between 0 and AVG_ALERT_WINDOW - 1
    alert_count = (alert_count + 1) % AVG_ALERT_WINDOW;

    //never send an alert for the first (AVG_ALERT_WINDOW - 1) checks
    if (voltage_window[AVG_ALERT_WINDOW - 1] == 0)
        return;

    //calculate average values
    float avg_voltage = 0, avg_current = 0, avg_temp1 = 0, avg_temp2 = 0;
    for (int i = 0; i < AVG_ALERT_WINDOW; i++)
    {
        avg_voltage += voltage_window[i];
        avg_current += current_window[i];
        avg_temp1 += temp1_window[i];
        avg_temp2 += temp2_window[i];
    }
    avg_voltage /= AVG_ALERT_WINDOW;
    avg_current /= AVG_ALERT_WINDOW;
    avg_temp1 /= AVG_ALERT_WINDOW;
    avg_temp2 /= AVG_ALERT_WINDOW;


    
    //check if values are in range
    if (avg_voltage > OVERVOLTAGE)
        alert_payload.overvoltage = 1;
    if (avg_current > OVERCURRENT)
        alert_payload.overcurrent = 1;
    if ((avg_temp1 > OVERTEMPERATURE || avg_temp2 > OVERTEMPERATURE))
        alert_payload.overtemperature = 1;

    //fully-charged alert
    if (scooter_status == SCOOTER_CHARGING)
    {
        if ((dynamic_payload.voltage > FULLY_CHARGED_MIN_VOLTAGE) && (dynamic_payload.current < FULLY_CHARGED_MAX_CURRENT))
        {
            fully_charged++;
            if (fully_charged > MAX_FULLY_CHARGED_ALERT_CHECKS)
            {
                scooter_status = SCOOTER_FULLY_CHARGED;
                alert_payload.fully_charged = 1;
            }
        }
        else
        {
            fully_charged = 0;
        }
    }


    //* IF ANY, SEND ALERT TO THE MASTER
    if (alert_payload.internal)
    {
        scooter_status = SCOOTER_ALERT;
        //STOP TIMERS
        xTimerStop(dynamic_timer, 0);
        xTimerStop(alert_timer, 0);
        //DELETE TIMERS
        xTimerDelete(dynamic_timer, 0);
        xTimerDelete(alert_timer, 0);
        if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
        {
            espnow_data_prepare(buf, ESPNOW_DATA_ALERT);
            esp_now_send(master_mac, (uint8_t *) buf, sizeof(espnow_data_t));
        } else
            ESP_LOGE(TAG, "Could not take send semaphore");  
    }

    free(buf);
}

static void esp_now_register_master(uint8_t *mac_addr, bool encrypt)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
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
    if (xQueueSend(espnow_queue, &evt, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }

    //give back the semaphore if status is successfull
    if (status == ESP_NOW_SEND_SUCCESS)
        xSemaphoreGive(send_semaphore);
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

    if (xQueueSend(espnow_queue, &evt, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) != pdTRUE) {
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
void espnow_data_prepare(espnow_data_t *buf, message_type type)
{
    //initiliaze buf to 0
    memset(buf, 0, sizeof(espnow_data_t));
    // esp NOW data
    buf->id = UNIT_ROLE;
    buf->type = type;

    //save last message type to allow retranmission
    last_msg_type = type;
    
    switch (type)
    {
        case ESPNOW_DATA_BROADCAST:
            //nothing else to do
            break;
    
        case ESPNOW_DATA_LOCALIZATION:
            if (scooter_status == SCOOTER_DISCONNECTED)
            {
                buf->field_1 = buf->field_2 = buf->field_3 = buf->field_4 = LOC_START_MESSAGE; 
                scooter_status = SCOOTER_CONNECTED;
            }
            else if (scooter_status == SCOOTER_CONNECTED)
            {
                buf->field_1 = 1;
                buf->field_2 = buf->field_3 = buf->field_4 = 0;
                if (dynamic_payload.voltage > MIN_VOLTAGE) 
                {
                    scooter_status = SCOOTER_CHARGING;
                    buf->field_1 = buf->field_2 = buf->field_3 = buf->field_4 = LOC_STOP_MESSAGE;
                    if (xTimerStart(dynamic_timer, 0) != pdPASS) {
                        ESP_LOGE(TAG, "Cannot start dynamic timer");
                    }
                }
            }
            break;

        case ESPNOW_DATA_ALERT:
            // fill in alerts data
            buf->field_1 = alert_payload.overtemperature;
            buf->field_2 = alert_payload.overcurrent;
            buf->field_3 = alert_payload.overvoltage;
            buf->field_4 = alert_payload.fully_charged;

            break;
        
        case ESPNOW_DATA_DYNAMIC:
            // fill in dynamic data
            buf->field_1 = dynamic_payload.voltage;
            buf->field_2 = dynamic_payload.current;
            buf->field_3 = dynamic_payload.temp1;
            buf->field_4 = dynamic_payload.temp2;
            break;
        
        case ESPNOW_DATA_CONTROL:
            // only received - never sent
            break;

        default:
            ESP_LOGE(TAG, "Message type error: %d", type);
            break;
    }

    // CRC
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, sizeof(espnow_data_t));
}

static void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t addr_type;
    espnow_data_t *espnow_data = (espnow_data_t *)pvParameter;

    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ID_ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                addr_type = IS_BROADCAST_ADDR(send_cb->mac_addr) ? true : false;
                esp_now_get_peer_num(&peer_num);

                ESP_LOGI(TAG, "Send data to "MACSTR". status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (addr_type)
                {
                    // Broadcast message will never return status 0 as it does not wait for ack
                    // if the master (encrypted) is registered, then stop sending broadcast data
                    if (peer_num.encrypt_num)
                    {
                        ESP_LOGI(TAG, "Stop sending broadcast data");
                        break;
                    }

                    /* Otherwise, Delay a while before sending the next data. */
                    vTaskDelay(BROADCAST_TIMEGAP);
                    
                    //send broadcast data again
                    if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                    {
                        espnow_data_prepare(espnow_data, ESPNOW_DATA_BROADCAST);
                        esp_now_send(broadcast_mac, (uint8_t *) espnow_data, sizeof(espnow_data_t));
                    } else  
                        ESP_LOGE(TAG, "Could not take send semaphore");
                } 
                else
                {
                    if (send_cb->status != ESP_NOW_SEND_SUCCESS) 
                    {
                        ESP_LOGE(TAG, "ERROR SENDING DATA TO MASTER");
                        comms_fail++;

                        //*MAX_COMMS_CONSECUTIVE_ERRORS --> RESTART
                        if (comms_fail > MAX_COMMS_ERROR)
                        {
                            ESP_LOGE(TAG, "TOO MANY COMMS ERRORS, DISCONNECTING");
                            scooter_status = SCOOTER_DISCONNECTED;
                            esp_now_del_peer(master_mac);
                            xTimerStop(dynamic_timer, 0);
                            xTimerStop(alert_timer, 0);
                            xTimerDelete(dynamic_timer, 0);
                            xTimerDelete(alert_timer, 0);

                            //reboot
                            esp_restart();
                        }
                        else //*RETRANSMISSIONS
                        {
                            ESP_LOGW(TAG, "RETRANSMISSION n. %d", comms_fail);
                            espnow_data_prepare(espnow_data, last_msg_type);
                            esp_now_send(send_cb->mac_addr, (uint8_t *)espnow_data, sizeof(espnow_data_t));
                        }
                    }
                    else
                    {
                        comms_fail = 0;
                    }
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
                addr_type = recv_data->type;

                if (addr_type == ESPNOW_DATA_BROADCAST) 
                {
                    //ESP_LOGI(TAG, "Receive broadcast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    //! ignore it as it comes from other peers
                }
                else if(addr_type == ESPNOW_DATA_CONTROL) 
                {
                    ESP_LOGI(TAG, "Receive control data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    //* Add MASTER 
                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) 
                    {
                        if (recv_data->field_1 > 1) //meaning the payload contains the alerts thresholds
                        {
                            ESP_LOGW(TAG, "Setting alerts limits");
                            //parse data to set local alerts limits
                            OVERCURRENT = recv_data->field_1;
                            OVERVOLTAGE = recv_data->field_2;
                            OVERTEMPERATURE = recv_data->field_3;
                            MIN_VOLTAGE = recv_data->field_4;
                        }
                        else
                        {
                            memcpy(master_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            ESP_LOGW(TAG, "Add master "MACSTR" to peer list", MAC2STR(master_mac));
                            esp_now_register_master(master_mac, true);

                            if (dynamic_timer == NULL)
                            {
                                // freeRTOS timers
                                const int dyn_timegap = pdMS_TO_TICKS( (int) recv_data->field_3 );

                                dynamic_timer = xTimerCreate("dynamic_timer", dyn_timegap, pdTRUE, NULL, dynamic_timer_callback);
                                ESP_ERROR_CHECK( dynamic_timer == NULL ? ESP_FAIL : ESP_OK);
                                alert_timer = xTimerCreate("alert_timer", ALERT_TIMEGAP, pdTRUE, NULL, alert_timer_callback);
                                ESP_ERROR_CHECK( alert_timer == NULL ? ESP_FAIL : ESP_OK);
                                if (xTimerStart(alert_timer, 0) != pdPASS) {
                                    ESP_LOGE(TAG, "Cannot start alert timer");
                                }
                            }

                            // START LOCALIZATION PROCEDURE
                            if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                            {
                                espnow_data_prepare(espnow_data, ESPNOW_DATA_LOCALIZATION);
                                esp_now_send(master_mac, (uint8_t *) espnow_data, sizeof(espnow_data_t));
                            } else  
                                ESP_LOGE(TAG, "Could not take send semaphore");  
                        }                        
                    }
                    else
                    {
                        //TODO: handle the dynamic change of dynamic timer from field_3 -- compare dyn_timegap
                    }
                }
                else if (addr_type ==  ESPNOW_DATA_LOCALIZATION)
                {
                    ESP_LOGI(TAG, "Receive LOCALIZATION message");
                    
                    //send the voltage back after reaction  time
                    vTaskDelay(recv_data->field_1/portTICK_PERIOD_MS);

                    if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
                    {
                        espnow_data_prepare(espnow_data, ESPNOW_DATA_LOCALIZATION);
                        esp_now_send(master_mac, (uint8_t *) espnow_data, sizeof(espnow_data_t));
                    } else  
                        ESP_LOGE(TAG, "Could not take send semaphore");
                }
                else if (addr_type == ESPNOW_DATA_ALERT)
                {
                    //ESP_LOGI(TAG, "Receive alert data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                    //REBOOT
                    if ((recv_data->field_2 == ALERT_MESSAGE) && (recv_data->field_3 == ALERT_MESSAGE) && (recv_data->field_4 == ALERT_MESSAGE))
                    {
                        ESP_LOGE(TAG, "REBOOTING");
                        xTimerStop(dynamic_timer, 0);
                        xTimerStop(alert_timer, 0);
                        xTimerDelete(dynamic_timer, 0);
                        xTimerDelete(alert_timer, 0);

                        if (scooter_status != SCOOTER_ALERT)
                            vTaskDelay(pdMS_TO_TICKS(recv_data->field_1 * 1000));

                        esp_now_del_peer(master_mac);

                        //reboot
                        esp_restart();
                    }
                }
                else 
                    ESP_LOGI(TAG, "Receive unexpected message type %d data from: "MACSTR"", addr_type, MAC2STR(recv_cb->mac_addr));
                
                free(recv_cb->data);
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

    // create a task to handle espnow events
    rc = xTaskCreate(espnow_task, "espnow_task", 2048, buffer, 4, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Create espnow task fail");
        free(buffer);
        return ESP_FAIL;
    }

    send_semaphore = xSemaphoreCreateBinary();
    if (send_semaphore == NULL) {
        ESP_LOGE(TAG, "Create send semaphore fail");
        return ESP_FAIL;
    }

    //crucial! otherwise the first message is not sent
    xSemaphoreGive(send_semaphore);

        ESP_LOGI(TAG, "Start sending broadcast data");
    
    if (xSemaphoreTake(send_semaphore, pdMS_TO_TICKS(ESPNOW_MAXDELAY)) == pdTRUE)
    {
        espnow_data_prepare(buffer, ESPNOW_DATA_BROADCAST);
        esp_now_send(broadcast_mac, (uint8_t *) buffer, sizeof(espnow_data_t));
    } else  
        ESP_LOGE(TAG, "Could not take send semaphore");

    return ESP_OK;
}

void init_hw(void)
{
    /* INIT OUTPUT PIN */
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins
    io_conf.pin_bit_mask = (1ULL<<25);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //SET IT LOW
    gpio_set_level(25, 0);

    /* Init adc */
    init_adc();
    ESP_LOGI(TAG, "ADC initialized");

    /* init I2C*/
    esp_err_t err_code;

    i2c_master_port = I2C_MASTER_NUM;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_400K;
    err_code = i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(err_code);
    ESP_LOGI(TAG, "I2C initialized");

    /* Initialize I2C semaphore */
    i2c_sem = xSemaphoreCreateMutex();

    uint8_t err;
    // create tasks to get measurements as fast as possible
    err = xTaskCreatePinnedToCore(get_temp, "get_temp", 2048, NULL, 5, NULL, 1);
    if ( err != pdPASS )
    {
        ESP_LOGE(TAG, "Task get_temp was not created successfully");
        return;
    }
    err = xTaskCreatePinnedToCore(get_adc, "get_adc", 5000, NULL, 5, NULL, 1);
    if( err != pdPASS )
    {
        ESP_LOGE(TAG, "Task get_ was not created successfully");
        return;
    }
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
    /* Initialize ACCELEROMETER */
    //accelerometer_init();

    //! EITHER OR FOR NOW

    /* Initialize V-A-T sensors */
    init_hw();

    wifi_init();
    espnow_init();

    atexit(esp_now_deinit);
    atexit(esp_wifi_deinit);
}

