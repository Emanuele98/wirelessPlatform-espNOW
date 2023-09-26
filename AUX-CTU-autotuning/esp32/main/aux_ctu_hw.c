#include "driver/gpio.h"
#include "include/aux_ctu_hw.h"

static QueueHandle_t uart0_queue;

TimerHandle_t connected_leds_timer, misaligned_leds_timer, charging_leds_timer, hw_readings_timer;

extern wpt_dynamic_payload_t dynamic_payload;
extern wpt_alert_payload_t alert_payload;
extern wpt_tuning_params_t tuning_params;

static float last_duty_cycle = 0.30;

// ALERTS LIMITS
extern float OVERCURRENT;
extern float OVERVOLTAGE;
extern float OVERTEMPERATURE;
extern bool  FOD_ACTIVE;

static const char* TAG = "HARDWARE";

static void parse_received_UART(uint8_t *rx_uart)
{
    //parse json
    cJSON *root = cJSON_Parse((char*)rx_uart);
    
    dynamic_payload.temp1 = cJSON_GetObjectItem(root, "temperature1")->valuedouble;
    //ESP_LOGI(TAG, "TEMP1: %.2f", dynamic_payload.temp1);

    dynamic_payload.temp2 = cJSON_GetObjectItem(root, "temperature2")->valuedouble;
    //ESP_LOGI(TAG, "TEMP2: %.2f", dynamic_payload.temp2);

    dynamic_payload.voltage = cJSON_GetObjectItem(root, "voltage")->valuedouble;
    //ESP_LOGI(TAG, "VOLTAGE: %.2f", dynamic_payload.voltage);

    dynamic_payload.current = cJSON_GetObjectItem(root, "current")->valuedouble;
    //ESP_LOGI(TAG, "CURRENT: %.2f", dynamic_payload.current);

    alertType_t alertType = cJSON_GetObjectItem(root,"alert")->valueint;
    //ESP_LOGW(TAG, "ALERT: %d", alertType);

    //* handle alert
    if (alertType == OV)
        alert_payload.overvoltage = 1;
    else if (alertType == OC)
        alert_payload.overcurrent = 1;
    else if (alertType == OT)
        alert_payload.overtemperature = 1;
    else if ((alertType == HS) || (alertType == DC))
        alert_payload.FOD = 1;

    if (alert_payload.internal)
    {
        ESP_LOGE(TAG, "ALERT: %d", alertType);
        send_alert_message();
    }

    // send details to the master every time the duty cycle changes
    tuning_params.duty_cycle = cJSON_GetObjectItem(root, "duty")->valuedouble;
    //ESP_LOGI(TAG, "DUTY CYCLE: %.2f", tuning_params.duty_cycle);
    
    tuning_params.tuning = cJSON_GetObjectItem(root, "tuning")->valueint;
    //ESP_LOGW(TAG, "TUNING: %d", tuning_params.tuning);
    
    tuning_params.low_vds_threshold = cJSON_GetObjectItem(root, "low_vds_threshold")->valueint;
    //ESP_LOGW(TAG, "low_vds_threshold: %d", tuning_params.low_vds_threshold );

    tuning_params.low_vds = cJSON_GetObjectItem(root, "low_vds")->valueint;
    //ESP_LOGW(TAG, "low_vds: %d", tuning_params.low_vds);

    if (fabs(tuning_params.duty_cycle - last_duty_cycle) > MIN_DUTY_CYCLE_CHANGE)
    {
        send_tuning_message();
        last_duty_cycle = tuning_params.duty_cycle;
    }

    //print json
    /*
    char *out = cJSON_Print(root);
    ESP_LOGI(TAG, "JSON: %s", out);
    free(out);
    */
    //free json
    cJSON_Delete(root);
}


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGE(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGE(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGE(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGW(TAG, "[UART PATTERN DETECTED] buffered size");
                    break;
                //Others
                default:
                    ESP_LOGW(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}
static void rx_task(void)
{
    uint8_t buffer[UART_BUFFER_SIZE];
    uint8_t rxIndex = 0;
    uint8_t* data = (uint8_t*) malloc(UART_BUFFER_SIZE/2);
    bool json = false;
    while (1) {
        const int rxBytes = uart_read_bytes(EX_UART_NUM, data, 1, portMAX_DELAY);
        if (rxBytes > 0) {
            //create json object and print it
            switch(data[0])
            {
                case '{':
                    json = true;
                    rxIndex = 0;
                    buffer[rxIndex++] = data[0];
                    break;
                case '}':
                    buffer[rxIndex++] = data[0];
                    rxIndex = 0;
                    if (json)
                    {
                        parse_received_UART(buffer);
                        //ESP_LOGI(TAG, "JSON");
                        vTaskDelay(300 / portTICK_PERIOD_MS);
                    }
                    json = false;
                    break;
                default:
                    buffer[rxIndex++] = data[0];
                    break;
            }
        }
    }
    free(data);
}

esp_err_t write_STM_limits()
{
    //create json file
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "overcurrent", OVERCURRENT);
    cJSON_AddNumberToObject(root, "overvoltage", OVERVOLTAGE);
    cJSON_AddNumberToObject(root, "overtemperature", OVERTEMPERATURE);

    char *my_json_string = cJSON_Print(root);
    const uint8_t len = strlen(my_json_string);

    const uint8_t txBytes = uart_write_bytes(EX_UART_NUM, my_json_string, len);

    cJSON_Delete(root);
    free(my_json_string);

    if (txBytes > 0)
        return ESP_OK;
    else
        return ESP_FAIL;
}

esp_err_t write_STM_command(stm32_command_t command)
{
    //create json file
    cJSON *root = cJSON_CreateObject();
    if (command == SWITCH_ON)
        cJSON_AddStringToObject(root, "mode", "deploy");
    else if (command == SWITCH_LOC)
        cJSON_AddStringToObject(root, "mode", "localization");
    else if (command == SWITCH_OFF)
        cJSON_AddStringToObject(root, "mode", "off");

    char *my_json_string = cJSON_Print(root);
    const uint8_t len = strlen(my_json_string);

    const uint8_t txBytes = uart_write_bytes(EX_UART_NUM, my_json_string, len);

    cJSON_Delete(root);
    free(my_json_string);
    
    if (txBytes > 0)
        return ESP_OK;
    else
        return ESP_FAIL;
}

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(EX_UART_NUM, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void hw_init()
{
    esp_err_t err_code;

    //*LED STRIP
    install_strip(STRIP_PIN);
    ESP_LOGI(TAG, "LED strip initialized successfully");

    // LED strip timers
    connected_leds_timer = xTimerCreate("connected_leds", CONNECTED_LEDS_TIMER_PERIOD, pdTRUE, NULL, connected_leds);   
    misaligned_leds_timer = xTimerCreate("misaligned_leds", MISALIGNED_LEDS_TIMER_PERIOD, pdTRUE, NULL, misaligned_leds);
    charging_leds_timer = xTimerCreate("charging leds", CHARGING_LEDS_TIMER_PERIOD, pdTRUE, NULL, charging_state);

    if ( (connected_leds_timer == NULL) || (misaligned_leds_timer == NULL) || (charging_leds_timer == NULL))
    {
        ESP_LOGW(TAG, "Timers were not created successfully");
        return;
    }

    xTimerStart(connected_leds_timer, 10);
    xTimerStart(misaligned_leds_timer, 10);
    xTimerStart(charging_leds_timer, 10);
    
    //*UART CONNECTION TO STM32
    // the STM32 board sends sensor measurements to the ESP32 via UART, along with FOD alerts and other information
	uart_init();
	xTaskCreate(uart_event_task, "uart_event_task", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);

    // safely switch off
    err_code = write_STM_command(SWITCH_OFF);
    if (err_code != ESP_OK)
    {
        ESP_LOGW(TAG, "Could not switch off STM32 board");
        return;
    }
}
