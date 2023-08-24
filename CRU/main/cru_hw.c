
#include "include/cru_hw.h"

static const char* TAG = "HARDWARE";

static esp_adc_cal_characteristics_t *adc_chars, *adc_chars2;
static const adc_channel_t channel = ADC_CHANNEL_4;     //GPIO32 
static const adc_channel_t channel2 = ADC_CHANNEL_5;   //GPIO33
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_atten_t atten2 = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;

/* Semaphore used to protect against I2C reading simultaneously */
extern SemaphoreHandle_t i2c_sem;
extern wpt_dynamic_payload_t dynamic_payload;
static uint8_t counter = 0;


void init_adc(void)
{
    /* init ADC */
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    adc1_config_channel_atten(channel2, atten2);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    adc_chars2 = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten2, width, DEFAULT_VREF, adc_chars2);

}

/**
 * @brief read sensor data
 *
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
float adc_read_voltage_sensor(void)
{
    float value;

    uint32_t voltage_reading = 0;
    voltage_reading += adc1_get_raw((adc1_channel_t)channel);
    
    //Convert voltage_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(voltage_reading, adc_chars);
    //printf("Raw: %d\tVoltage: %dmV\n", voltage_reading, voltage);
    value = (float)(voltage*42/1000.00);
    if (value < 10)
    {
        //printf("--VOLTAGE: 0.00 \n");
        return 0;
    }
    else
    {
        //printf("--VOLTAGE: %.2f\n", value);
        return value;
    }
}

float adc_read_current_sensor(void)
{
    float value;

    uint32_t current_reading = 0;
    current_reading += adc1_get_raw((adc1_channel_t)channel2);
    
    //Convert voltage_reading to voltage in mV
    uint32_t current = esp_adc_cal_raw_to_voltage(current_reading, adc_chars2);
    //printf("Raw: %d\tVoltage: %dmV\n", voltage_reading, voltage);
    value = (float)((current - 400)/360.00) * 0.94;

    if (current < 450)
    {
        //printf("--CURRENT: 0.00 \n");
        return 0;
    }
    else
    {
        //printf("--CURRENT: %.2f\n", value);
        return value;
    }
}    

float i2c_read_temperature_sensor(bool n_temp_sens)
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
    printf("temperature %d: %.02f [C]\n", n_temp_sens + 1 ,value);

    exit:
        xSemaphoreGive(i2c_sem);
        return value;
}

//read dynamic parameters from ADC sensors
void get_adc(void)
{
    static float volt = 0, curr = 0;
    static int adc_counter = 0;

    while(1)
    {
        adc_counter++;
        
        //voltage
        volt += adc_read_voltage_sensor();
        //current
        curr += adc_read_current_sensor();         
        
        //Multisampling
        if (adc_counter == NO_OF_SAMPLES)
        {
            //average
            dynamic_payload.voltage = volt / NO_OF_SAMPLES;
            dynamic_payload.current = curr / NO_OF_SAMPLES;

            volt = 0;
            curr = 0;
            adc_counter = 0;

            ESP_LOGW(TAG, "Voltage: %.2f, Current: %.2f", dynamic_payload.voltage, dynamic_payload.current );
            //vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    //if for any reason the loop terminates
    vTaskDelete(NULL); 
}

//read dynamic parameters from I2C sensors
void get_temp(void)
{
    while (1)
    {
        float t1 = 0, t2 = 0;
        if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            switch(counter)
            {
                //temperature 1
                case 0:
                    counter++;
                    t1 = i2c_read_temperature_sensor(0);
                    if (t1 != -1)
                        dynamic_payload.temp1 = t1;
                    break;

                //temperature 2
                case 1:
                    counter = 0;
                    t2 = i2c_read_temperature_sensor(1);
                    if (t2 != -1)
                        dynamic_payload.temp2= t2;
                    break;
                default:
                    xSemaphoreGive(i2c_sem);
                    break;
            }
        }
    }

    //if for any reason the loop terminates
    vTaskDelete(NULL); 
}