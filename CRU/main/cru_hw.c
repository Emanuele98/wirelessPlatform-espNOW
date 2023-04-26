#include "driver/gpio.h"
#include "include/cru_hw.h"

/* 
 * All local readings (on PTU) are not calibrated. Therefore, you must provide a
 * way to convert digital input data to the respective reading types. For example,
 * in the `read_temperature` function, you could convert raw digital data to real
 * temperature by using an intercept and a slope.
 *  
 * It is best to define such values as preprocessor definitions using the #define
 * directive.
 */

static const char* TAG = "HARDWARE";

/**
 * @brief read sensor data
 *
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
float i2c_read_voltage_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, V_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, DYNAMIC_PARAM_TIMER_INTERVAL / 2);
    i2c_cmd_link_delete(cmd);
    
    if (ret==ESP_OK)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
        i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, DYNAMIC_PARAM_TIMER_INTERVAL / 2);
        i2c_cmd_link_delete(cmd);
    }

    if(ret!=ESP_OK)
    {
        //ESP_LOGE(TAG, "voltage reading problem");
        value = -1;
        goto exit;
    }

    //printf("first byte: %02x\n", byte_1);
    //printf("second byte : %02x\n", byte_2);
    value = (int16_t)(first_byte << 8 | second_byte) * 0.00125 * 4.9216;
    printf("sensor val: %.02f [V]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        //! testing
        //return 130;
        return value;
}

float i2c_read_current_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, A_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, DYNAMIC_PARAM_TIMER_INTERVAL / 2);
    i2c_cmd_link_delete(cmd);

    if (ret==ESP_OK)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
        i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, DYNAMIC_PARAM_TIMER_INTERVAL / 2);
        i2c_cmd_link_delete(cmd);
    }
    
    if(ret!=ESP_OK)
    {
        //ESP_LOGE(TAG, "current reading problem");
        value = -1;
        goto exit;
    }

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    value = (int16_t)(first_byte << 8 | second_byte) * 0.0000025 / 0.012;
    printf("sensor val: %.02f [A]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        //! testing
        //return 0.5;
        return value;
}


float i2c_read_temperature_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, T_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, T_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, DYNAMIC_PARAM_TIMER_INTERVAL / 2);
    i2c_cmd_link_delete(cmd);

    if (ret==ESP_OK)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, T_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
        i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, DYNAMIC_PARAM_TIMER_INTERVAL / 2);
        i2c_cmd_link_delete(cmd);
    }
    
    if(ret!=ESP_OK)
    {
        //ESP_LOGE(TAG, "temperature reading problem");
        value = -1;
        goto exit;
    }

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    value = (int16_t)(first_byte << 4 | second_byte >> 4) * 0.0625 ;
    printf("sensor val: %.02f [°C]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        return value;
}