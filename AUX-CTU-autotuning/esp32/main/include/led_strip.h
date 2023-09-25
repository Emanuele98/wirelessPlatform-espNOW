#ifndef _LED_STRIP_H_
#define _LED_STRIP_H_

#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "driver/rmt.h"
#include "aux_ctu_hw.h"

// LED STRIP
#define N_LEDS                  25
#define STRIP_PIN               15

/* Default Leds connected blinking duration */
#define CONNECTED_LEDS_TIMER_PERIOD pdMS_TO_TICKS(40)

/* Leds misaligned blinking duration */
#define MISALIGNED_LEDS_TIMER_PERIOD pdMS_TO_TICKS(300)

/* Leds charging timer duration*/
#define CHARGING_LEDS_TIMER_PERIOD pdMS_TO_TICKS(200)


/**
* @brief LED Strip Type
*
*/
typedef struct led_strip_s led_strip_t;

/**
* @brief LED Strip Device Type
*
*/
typedef void *led_strip_dev_t;

/**
* @brief Declare of LED Strip Type
*
*/
struct led_strip_s {
    /**
    * @brief Set RGB for a specific pixel
    *
    * @param strip: LED strip
    * @param index: index of pixel to set
    * @param red: red part of color
    * @param green: green part of color
    * @param blue: blue part of color
    *
    * @return
    *      - ESP_OK: Set RGB for a specific pixel successfully
    *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
    *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
    */
    esp_err_t (*set_pixel)(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);

    /**
    * @brief Refresh memory colors to LEDs
    *
    * @param strip: LED strip
    * @param timeout_ms: timeout value for refreshing task
    *
    * @return
    *      - ESP_OK: Refresh successfully
    *      - ESP_ERR_TIMEOUT: Refresh failed because of timeout
    *      - ESP_FAIL: Refresh failed because some other error occurred
    *
    * @note:
    *      After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.
    */
    esp_err_t (*refresh)(led_strip_t *strip, uint32_t timeout_ms);

    /**
    * @brief Clear LED strip (turn off all LEDs)
    *
    * @param strip: LED strip
    * @param timeout_ms: timeout value for clearing task
    *
    * @return
    *      - ESP_OK: Clear LEDs successfully
    *      - ESP_ERR_TIMEOUT: Clear LEDs failed because of timeout
    *      - ESP_FAIL: Clear LEDs failed because some other error occurred
    */
    esp_err_t (*clear)(led_strip_t *strip, uint32_t timeout_ms);

    /**
    * @brief Free LED strip resources
    *
    * @param strip: LED strip
    *
    * @return
    *      - ESP_OK: Free resources successfully
    *      - ESP_FAIL: Free resources failed because error occurred
    */
    esp_err_t (*del)(led_strip_t *strip);
};

/**
* @brief LED Strip Configuration Type
*
*/
typedef struct {
    uint32_t max_leds;   /*!< Maximum LEDs in a single strip */
    led_strip_dev_t dev; /*!< LED strip device (e.g. RMT channel, PWM channel, etc) */
} led_strip_config_t;

/**
 * @brief Default configuration for LED strip
 *
 */
#define LED_STRIP_DEFAULT_CONFIG(number, dev_hdl) \
    {                                             \
        .max_leds = number,                       \
        .dev = dev_hdl,                           \
    }

/**
 * @brief Construct a new install strip object
 * 
 * @param pin assigned GPIO pin
 */
void install_strip(uint8_t pin);


/**
 * @brief Function used by the timer for the default connected led state
 * 
 */
void connected_leds(void);

/**
 * @brief Function used for orange blinking when the scooter is misaligned 
 *  
 */
void misaligned_leds(void);

/**
 * @brief Function used for orange blinking when the scooter is charging
 * 
 */
void charging_state(void);


/**
 * @brief Set the led strip with one defined colour
 * 
 */

void set_strip(uint8_t r, uint8_t g, uint8_t b);  


#endif /* _LED_STRIP_H_ */
