#include "espnow.h"
#include "peer.h"
#include "wifi.h"

static const char *TAG = "MAIN";

time_t reconnection_time = 0;
time_t timePeer[NUMBER_TX+NUMBER_RX] = {0};
nvs_handle_t my_handle;
extern time_t now;

/**
 * @brief Init the values on NVS
 *          This allows to keep tarck of each peer's minimum reconnection time over reboots
 * 
 */
void init_NVS(void)
{
    //NVS reading
    esp_err_t err = nvs_open("reconnection", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else 
        {
            // save timePeer after connection to NTP Server
            time(&now);
            reconnection_time = now;
            for (uint8_t i = 0; i < NUMBER_TX+NUMBER_RX; i++)
            {
                char peer_name[5];
                sprintf(peer_name, "%d", i+1);
                err = nvs_get_i64(my_handle, peer_name, &timePeer[i]);
                if (err == ESP_ERR_NVS_NOT_FOUND)
                {   
                    timePeer[i] = now;
                    nvs_set_i64(my_handle, peer_name, timePeer[i]);
                }
            }

            err = nvs_commit(my_handle);
            if (!err)
                ESP_LOGI(TAG, "NVS INIT DONE");
            else
                ESP_LOGE(TAG, "NVS INIT FAILED");
            
            // Close
            nvs_close(my_handle);
        }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    /* Init wifi */
    if (CONFIG_WIFI_EN)
        wifi_init();
    else
        wifi_init_connectionless();

    /* Init values on NVS */ 
    init_NVS();

    /* Init espnow */
    ret = espnow_init();
    ESP_ERROR_CHECK( ret );

    /* Init peers */
    peer_init(NUMBER_RX + NUMBER_TX);

    /* Register free memory function at the end of the program */
    atexit(delete_all_peers);
    atexit(esp_now_deinit);
    atexit(esp_wifi_deinit);

    /* Print free memory */
    ESP_LOGE(TAG, "[APP] Free memory: %d bytes\n", (int) esp_get_free_heap_size());

    //todo: tasks size -- watermark
    //todo: menuconfig to optimize everything
}
