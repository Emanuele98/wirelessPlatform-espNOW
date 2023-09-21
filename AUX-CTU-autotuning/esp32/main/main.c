#include "espnow.h"
#include "led_strip.h"
#include "aux_ctu_hw.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    ret = espnow_init();

    ESP_ERROR_CHECK( ret );

    hw_init();

    ESP_LOGW(TAG, "\n[APP] Free memory: %d bytes\n", (int) esp_get_free_heap_size());

    atexit(esp_now_deinit);
    atexit(esp_wifi_deinit);
}
