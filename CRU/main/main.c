#include "espnow.h"
#include "cru_hw.h"
#include "lis3dh.h"

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

    ESP_LOGW(TAG, "\n[APP] Free memory: %d bytes\n", (int) esp_get_free_heap_size());

    /* Initialize V-A-T sensors */
    init_hw();

    wifi_init();
    ret = espnow_init();
    ESP_ERROR_CHECK( ret );

    atexit(esp_now_deinit);
    atexit(esp_wifi_deinit);
}

