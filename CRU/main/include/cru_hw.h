#ifndef __PRU_HW_H__
#define __PRU_HW_H__

#include <math.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
//watchdog
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espnow.h"

#define AVG_ALERT_WINDOW                10
#define FULLY_CHARGED_MIN_VOLTAGE       50
#define FULLY_CHARGED_MAX_CURRENT       0.1
#define MAX_FULLY_CHARGED_ALERT_CHECKS  500 

/* adc */
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
// check specific eFuse Vref with espefuse.py --port COMx adc_info
#define NO_OF_SAMPLES   2000          //Multisampling


#define ALERT_PARAM_TIMER_INTERVAL      pdMS_TO_TICKS(100)				       /**< Timer synced to Alert parameter characteristic (60 ms). */

#define I2C_MASTER_SCL_IO 19                                  /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18                                  /*!< gpio number for I2C master data  */

#define I2C_MASTER_NUM     1                                  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000                             /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define V_A_SENSOR_ADDR 0x40  /*!< slave address for voltage and current sensor */
#define T1_SENSOR_ADDR 0x48    /*!< slave address for first temperature sensor */
#define T2_SENSOR_ADDR 0x49    /*!< slave address for second temperature sensor */
#define V_REGISTER_ADDR 0x02  /* bus voltage register address */
#define A_REGISTER_ADDR 0x01  /* shunt register address */
#define T_REGISTER_ADDR 0x00  /* temperature register address */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

void init_adc(void);
void get_temp(void);
void get_adc(void);


#endif