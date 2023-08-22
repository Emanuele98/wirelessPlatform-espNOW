#ifndef __PRU_HW_H__
#define __PRU_HW_H__

#include <math.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <sys/time.h>
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "led_strip.h"
#include "espnow.h"

#define OR_TIME_GAP                 pdMS_TO_TICKS(30)
#define HW_READINGS_TIMER_PERIOD    pdMS_TO_TICKS(20)

/* Arbitrary pin values */
#define FULL_POWER_OUT_PIN           GPIO_NUM_26           /* GPIO26 */
#define LOW_POWER_OUT_PIN            GPIO_NUM_25           /* GPIO25 */
#define OR_GATE                      GPIO_NUM_33           /* GPIO33 */
#define GPIO_OUTPUT_PIN_SEL          ((1ULL<<FULL_POWER_OUT_PIN) | (1ULL<<LOW_POWER_OUT_PIN) | (1ULL<<OR_GATE))
#define FOD_FPGA                     GPIO_NUM_27           /* GPIO27 */
#define GPIO_INPUT_PIN_SEL           (1ULL<<FOD_FPGA)



/* I2C */
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

void hw_readings_timeout(void);

void safely_enable_full_power(void); 

void safely_enable_low_power(void);

void safely_switch_off(void);

esp_err_t i2c_master_init(void);



#endif