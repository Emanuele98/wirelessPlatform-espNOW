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
#include "cJSON.h"
#include "driver/uart.h"

#define AVG_ALERT_WINDOW                10

#define UART_BUFFER_SIZE                1024*2

#define TXD_PIN                         (GPIO_NUM_4)
#define RXD_PIN                         (GPIO_NUM_5)

#define EX_UART_NUM                     UART_NUM_0
#define UART_TASK_STACK_SIZE            1024*10
#define UART_TASK_PRIORITY              5

typedef enum 
{
    SWITCH_ON,
    SWITCH_OFF,
} stm32_command_t;

typedef enum 
{
    NONE,
    OV,
    OC,
    OT,
    HS,
    DC,
} alertType_t;

/**
 * @brief Initialize the hardware
*/
void hw_init();

/**
 * @brief Send switch ON command to the STM32
*/
esp_err_t write_STM_command(stm32_command_t command);

/**
 * @brief Send the ALERTS limits to the STM32
*/
esp_err_t write_STM_limits();





#endif