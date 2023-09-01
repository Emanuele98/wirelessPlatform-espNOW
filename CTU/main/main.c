#include "espnow.h"
#include "peer.h"
#include "wifi.h"

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

    ESP_LOGE(TAG, "[APP] Free memory: %d bytes\n", (int) esp_get_free_heap_size());

    if (CONFIG_WIFI_EN)
        wifi_init();
    else
        wifi_init_connectionless();

    ret = espnow_init();
    ESP_ERROR_CHECK( ret );

    peer_init(NUMBER_RX + NUMBER_TX);

    atexit(delete_all_peers);
    atexit(esp_now_deinit);
    atexit(esp_wifi_deinit);

    //todo: tasks size -- watermark
    //todo: menuconfig to optimize everything
}
