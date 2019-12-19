//
// Created by ignne on 29.10.19.
//

#include "firmware_connection.h"

#include <string.h>
#include "sdkconfig.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "tcpip_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define GOT_IPV4_BIT BIT(0)
#define CONNECTED_BITS (GOT_IPV4_BIT)

static EventGroupHandle_t s_connect_event_group;
static ip4_addr_t s_ip_addr;
static const char *s_connection_name;


/* set up connection, Wi-Fi or Ethernet */
static void start(void);

static void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    memcpy(&s_ip_addr, &event->ip_info.ip, sizeof(s_ip_addr));
    xEventGroupSetBits(s_connect_event_group, GOT_IPV4_BIT);
}

esp_err_t wifi_connect(void)
{
    if (s_connect_event_group != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_connect_event_group = xEventGroupCreate();
    start();
    xEventGroupWaitBits(s_connect_event_group, CONNECTED_BITS, true, true, portMAX_DELAY);
    printf("Connected to %s", s_connection_name);
    printf("IPv4 address: " IPSTR, IP2STR(&s_ip_addr));
    return ESP_OK;
}

static void on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    printf("Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return;
    }
    ESP_ERROR_CHECK(err);
}

static void start(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "WLAN",
            .password = "",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    s_connection_name = "WLAN";
}
