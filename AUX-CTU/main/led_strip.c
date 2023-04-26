#include "include/led_strip.h"

led_strip_t *strip;
bool blink1 = true;
bool blink2 = true;
int l1 = 0, l2 = 0;


static const char *TAG = "LED STRIP";
#define STRIP_CHECK(a, str, goto_tag, ret_value, ...)                             \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define WS2812_T0H_NS (350)
#define WS2812_T0L_NS (1000)
#define WS2812_T1H_NS (1000)
#define WS2812_T1L_NS (350)
#define WS2812_RESET_US (280)

static uint32_t ws2812_t0h_ticks = 0;
static uint32_t ws2812_t1h_ticks = 0;
static uint32_t ws2812_t0l_ticks = 0;
static uint32_t ws2812_t1l_ticks = 0;

typedef struct {
    led_strip_t parent;
    rmt_channel_t rmt_channel;
    uint32_t strip_len;
    uint8_t buffer[0];
} ws2812_t;


void set_strip(uint8_t r, uint8_t g, uint8_t b)  
{
    for (int j = 0; j < N_LEDS; j++) 
    {
        strip->set_pixel(strip, j, r, g, b);
    }
    strip->refresh(strip, 10);
}


void install_strip(uint8_t pin)
{
    /* Initialize LED strip on PIN */ 
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(pin, 0);
    // set counter clock to 40MHz
    config.clk_div = 2;
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
    //install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(N_LEDS, (led_strip_dev_t)config.channel);

    strip = led_strip_new_rmt_ws2812(&strip_config);

    strip_enable = false;
    strip_misalignment = false;
    strip_charging = false;
    set_strip(0, 100, 100);
}

//todo: remove this function and use directly the rgb values

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}


/**
 * @brief Conver RGB data to RMT format.
 *
 * @note For WS2812, R,G,B each contains 256 different choices (i.e. uint8_t)
 *
 * @param[in] src: source data, to converted to RMT format
 * @param[in] dest: place where to store the convert result
 * @param[in] src_size: size of source data
 * @param[in] wanted_num: number of RMT items that want to get
 * @param[out] translated_size: number of source data that got converted
 * @param[out] item_num: number of RMT items which are converted from source data
 */
static void IRAM_ATTR ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ ws2812_t0h_ticks, 1, ws2812_t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ ws2812_t1h_ticks, 1, ws2812_t1l_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

static esp_err_t ws2812_set_pixel(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    if ((index < ws2812->strip_len) && (index > 0))
    {
        uint32_t start = index * 3;
        // In thr order of GRB
        ws2812->buffer[start + 0] = green & 0xFF;
        ws2812->buffer[start + 1] = red & 0xFF;
        ws2812->buffer[start + 2] = blue & 0xFF;
    }
    return ESP_OK;
}

static esp_err_t ws2812_refresh(led_strip_t *strip, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    STRIP_CHECK(rmt_write_sample(ws2812->rmt_channel, ws2812->buffer, ws2812->strip_len * 3, true) == ESP_OK,
                "transmit RMT samples failed", err, ESP_FAIL);
    return rmt_wait_tx_done(ws2812->rmt_channel, timeout_ms);
err:
    return ret;
}

static esp_err_t ws2812_clear(led_strip_t *strip, uint32_t timeout_ms)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    // Write zero to turn off all leds
    memset(ws2812->buffer, 0, ws2812->strip_len * 3);
    return ws2812_refresh(strip, timeout_ms);
}

static esp_err_t ws2812_del(led_strip_t *strip)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    free(ws2812);
    return ESP_OK;
}

led_strip_t *led_strip_new_rmt_ws2812(const led_strip_config_t *config)
{
    led_strip_t *ret = NULL;
    STRIP_CHECK(config, "configuration can't be null", err, NULL);

    // 24 bits per led
    uint32_t ws2812_size = sizeof(ws2812_t) + config->max_leds * 3;
    ws2812_t *ws2812 = calloc(1, ws2812_size);
    STRIP_CHECK(ws2812, "request memory for ws2812 failed", err, NULL);

    uint32_t counter_clk_hz = 0;
    STRIP_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev, &counter_clk_hz) == ESP_OK,
                "get rmt counter clock failed", err, NULL);
    // ns -> ticks
    float ratio = (float)counter_clk_hz / 1e9;
    ws2812_t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    ws2812_t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    ws2812_t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    ws2812_t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);

    // set ws2812 to rmt adapter
    rmt_translator_init((rmt_channel_t)config->dev, ws2812_rmt_adapter);

    ws2812->rmt_channel = (rmt_channel_t)config->dev;
    ws2812->strip_len = config->max_leds;

    ws2812->parent.set_pixel = ws2812_set_pixel;
    ws2812->parent.refresh = ws2812_refresh;
    ws2812->parent.clear = ws2812_clear;
    ws2812->parent.del = ws2812_del;

    return &ws2812->parent;
err:
    return ret;
}

void connected_leds(void *arg)
{

    if(strip_enable)
    {   
        if (blink1)
        {            
            for (int j = 0; j <= l1; j++) 
                strip->set_pixel(strip, N_LEDS - j, 0, 150, 150);
        } else
        {
            for (int j = 0; j <= l1; j++) 
                strip->set_pixel(strip, N_LEDS - j, 150, 150, 0);
        }
        strip->refresh(strip, 10);  

    if (l1==N_LEDS)
    {
        l1=0;
        blink1 = !blink1;
    } else 
        l1++;
    }  
}

void misaligned_leds(void *arg)
{
    if (strip_misalignment)
    {
        if (blink2)
        {
            for (int j = 0; j <= N_LEDS; j++) 
                strip->set_pixel(strip, N_LEDS - j, 200, 100, 0);
            strip->refresh(strip, 10);
        } else
            strip->clear(strip, 10);

    blink2 = !blink2;
    }
}

void charging_state(void *arg)
{
    if (strip_charging)
    {
        for (int j = 1; j < l2; j++) 
            strip->set_pixel(strip, j, 0, 205, 0);
        for (int y = l2; y <= N_LEDS; y++)
                strip->set_pixel(strip, y, 0, 20, 0);
        strip->refresh(strip, 10);      
    }

    if (l2==N_LEDS)
        l2=0;
    else 
        l2++;
}



