#include "driver/gpio.h"
#include "include/aux_ctu_hw.h"

/* 
 * All local readings (on PTU) are not calibrated. Therefore, you must provide a
 * way to convert digital input data to the respective reading types. For example,
 * in the `read_temperature` function, you could convert raw digital data to real
 * temperature by using an intercept and a slope.
 *  
 * It is best to define such values as preprocessor definitions using the #define
 * directive.
 */



TimerHandle_t connected_leds_timer, misaligned_leds_timer, charging_leds_timer, hw_readings_timer;

/* Keeps output states in memory */
bool low_power, full_power, or_gate;

static float volt, curr, t1, t2;
static uint8_t counter = 0;

SemaphoreHandle_t i2c_sem;
extern wpt_dynamic_payload_t dynamic_payload;

static const char* TAG = "HARDWARE";

static esp_err_t i2c_master_init(void)
{
        int i2c_master_port = I2C_MASTER_NUM;

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief read sensor data
 *
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static float i2c_read_voltage_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, V_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret==ESP_OK)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
        i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }
    
    if(ret!=ESP_OK)
    {
        //ESP_LOGW(TAG, "voltage reading problem");
        value = -1;
        goto exit;
    }

    //printf("first byte: %02x\n", byte_1);
    //printf("second byte : %02x\n", byte_2);
    value = (int16_t)(first_byte << 8 | second_byte) * 0.00125 * 3.9412;
    //printf("voltage: %.02f [V]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        //!testing
        //return 67;
        return value;
}

static float i2c_read_current_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, A_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret==ESP_OK)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
        i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }
    
    if(ret!=ESP_OK)
    {
        //ESP_LOGW(TAG, "current reading problem");
        value = -1;
        goto exit;
    }

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    value = (int16_t)(first_byte << 8 | second_byte) * 0.0000025 / 0.012;
    //printf("current: %.02f [A]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        //!testing
        //return 1;
        return value;
}


static float i2c_read_temperature_sensor(bool n_temp_sens)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if(n_temp_sens)
    {
        i2c_master_write_byte(cmd, T1_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    } else {
        i2c_master_write_byte(cmd, T2_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    }
    i2c_master_write_byte(cmd, T_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret==ESP_OK)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        if(n_temp_sens)
        {
            i2c_master_write_byte(cmd, T1_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
        } else {
            i2c_master_write_byte(cmd, T2_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
        }    
        i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
        i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }

    if(ret!=ESP_OK)
    {
        //ESP_LOGW(TAG, "temperature reading problem");
        value = -1;
        goto exit;
    }

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    value = (int16_t)(first_byte << 4 | second_byte >> 4) * 0.0625 ;
    //printf("temperature: %.02f [°C]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        return value;
}

//read dynamic parameters from I2C sensors
static void hw_readings_timeout(void)
{
    if (xSemaphoreTake(i2c_sem, portMAX_DELAY) == pdTRUE)
    {
        switch(counter)
        {
            //voltage
            case 0:
                counter++;
                volt = i2c_read_voltage_sensor();
                if (volt !=-1)
                    dynamic_payload.voltage = volt;
                break;
            
            //current
            case 1:
                counter++;
                curr = i2c_read_current_sensor();
                if (curr != -1)
                    dynamic_payload.current = curr;
                break;

            //temperature 1
            case 2:
                counter++;
                t1 = i2c_read_temperature_sensor(0);
                if (t1 != -1)
                    dynamic_payload.temp1 = t1;
                break;

            //temperature 2
            case 3:
                counter = 0;
                t2 = i2c_read_temperature_sensor(1);
                if (t2 != -1)
                    dynamic_payload.temp2 = t2;
                break;
            default:
                xSemaphoreGive(i2c_sem);
                break;
        }
    }
}

static void enable_full_power_output(void)
{
    if ((!or_gate) && (!low_power))
    {
        //enable only if OR GATE is low
        full_power = true;
        gpio_set_level(FULL_POWER_OUT_PIN, 1);
    }
}

static void disable_full_power_output(void)
{
    full_power = false;
    gpio_set_level(FULL_POWER_OUT_PIN, 0);
}

static void enable_low_power_output(void)
{
    //enable only if OR GATE is low
    if ((!or_gate) && (!full_power))
    {
        low_power = true;
        gpio_set_level(LOW_POWER_OUT_PIN, 1);
    }
}

static void disable_low_power_output(void)
{
    low_power = false;
    gpio_set_level(LOW_POWER_OUT_PIN, 0);
}

static void enable_OR_output(void)
{
    //enable only if both low and full modes are
    if ((!low_power) && (!full_power))
    {
        or_gate = true;
        gpio_set_level(OR_GATE, 1);
    }
}

static void disable_OR_output(void)
{
    or_gate = false;
    gpio_set_level(OR_GATE, 0);
}

void safely_enable_full_power(void) 
{
    //disable interfaces
    disable_low_power_output();
    disable_OR_output();
    //wait
    vTaskDelay(OR_TIME_GAP);
    //enable power
    enable_full_power_output();
}

void safely_enable_low_power(void)
{
    //disable interfaces
    disable_full_power_output();
    disable_OR_output();
    //wait
    vTaskDelay(OR_TIME_GAP);
    //enable power
    enable_low_power_output();
}

void safely_switch_off(void)
{
    //disable interfaces
    disable_full_power_output();
    disable_low_power_output();
    //wait
    vTaskDelay(OR_TIME_GAP);
    //enable OR gate
    enable_OR_output();
}


void hw_init()
{
    esp_err_t err_code;

    install_strip(STRIP_PIN);
    ESP_LOGI(TAG, "LED strip initialized successfully");

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    //* init GPIOs
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //todo: how do we get FOD?

    safely_switch_off();

    /* Initialize I2C semaphore */
    i2c_sem = xSemaphoreCreateMutex();

    //* HW reading timer*/
    hw_readings_timer = xTimerCreate("hw_readings", HW_READINGS_TIMER_PERIOD, pdTRUE, NULL, hw_readings_timeout);
    if (xTimerStart(hw_readings_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Cannot start hw readings timer");
    }

    //* LED strip timers
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
}
