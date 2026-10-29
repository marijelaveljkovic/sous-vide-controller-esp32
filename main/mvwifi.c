#include "mvwifi.h"

#include <string.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "mvwifi";
static mvwifi_got_ip_cb_t s_got_ip_cb = NULL; // u main-u je to mvhttp_start

/* Jedan handler za sve WiFi/IP dogadjaje:
 *  STA_START     -> WiFi je spreman, krecem povezivanje
 *  DISCONNECTED  -> veza pukla (ili nije uspela), pokusavam ponovo
 *  STA_GOT_IP    -> povezana sam, javljam main-u preko callback-a */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // pokusava beskonacno, ali ne smeta jer sve ovo radi u pozadini
        ESP_LOGW(TAG, "Wifi konekcija prekinuta, pokusavam ponovo...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Povezan, IP adresa: " IPSTR, IP2STR(&event->ip_info.ip));
        if (s_got_ip_cb)
        {
            s_got_ip_cb();
        }
    }
}

/* Standardni redosled za WiFi u station modu (kao u ESP-IDF primeru):
 * NVS -> netif -> event loop -> wifi init -> handleri -> config -> start.
 * Nista ne ceka konekciju (esp_wifi_start samo pokrene wifi task).
 * Ako neki korak ne uspe, ne rusim program (nema ESP_ERROR_CHECK),
 * nego samo ispisem upozorenje i uredjaj radi dalje bez WiFi-ja. */
void mvwifi_connect(const char *ssid, const char *password, mvwifi_got_ip_cb_t on_got_ip)
{
    s_got_ip_cb = on_got_ip;

    // WiFi drajver cuva podesavanja u NVS flash-u, pa mora prvo on
    esp_err_t err = nvs_flash_init();
    // ako je NVS pun ili stare verzije, obrisem ga i probam ponovo
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Inicijalizacija NVS-a nije uspela (%s), preskacem WiFi", esp_err_to_name(err));
        return;
    }

    if (esp_netif_init() != ESP_OK)
    {
        ESP_LOGW(TAG, "esp_netif_init nije uspeo, preskacem WiFi");
        return;
    }

    // INVALID_STATE znaci da event loop vec postoji, i to je ok
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "Kreiranje event loop-a nije uspelo, preskacem WiFi");
        return;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&init_cfg) != ESP_OK)
    {
        ESP_LOGW(TAG, "esp_wifi_init nije uspeo, preskacem WiFi");
        return;
    }

    // isti handler slusa sve WIFI dogadjaje i dogadjaj dobijanja IP adrese
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    // -1 da ostane mesta za '\0' na kraju; min. zastita mreze WPA2
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // STA = uredjaj se kaci na postojeci ruter (nije sam access point)
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK ||
        esp_wifi_start() != ESP_OK)
    {
        ESP_LOGW(TAG, "Pokretanje WiFi-ja nije uspelo, nastavljam bez WiFi-ja");
        return;
    }

    ESP_LOGI(TAG, "Povezivanje na WiFi mrezu '%s' u pozadini...", ssid);
}
