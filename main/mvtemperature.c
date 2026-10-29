#include "mvtemperature.h"
#include "esp_log.h"
#include "onewire_bus.h"
#include "ds18b20.h"

static const char *TAG = "mvtemperature";

static ds18b20_device_handle_t s_ds18b20;

void mvtemperature_init(gpio_num_t pin)
{
    /* 1-Wire se radi preko RMT periferije - ona precizno tempira impulse,
     * pa procesor ne mora sam da ceka mikrosekunde */
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = pin,
        .flags.en_pull_up = true,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1 bajt komanda + 8 bajtova ROM adrese + 1 bajt komanda (iz primera)
    };
    onewire_bus_handle_t bus;
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    // imam samo jedan senzor na liniji, pa ne moram da trazim adrese
    ds18b20_config_t ds18b20_config = {};
    ESP_ERROR_CHECK(ds18b20_new_device_from_bus(bus, &ds18b20_config, &s_ds18b20));
    // 12 bita = 0.0625C rezolucija, ali konverzija traje ~750ms (zato citam na 1s)
    ESP_ERROR_CHECK(ds18b20_set_resolution(s_ds18b20, DS18B20_RESOLUTION_12B));

    ESP_LOGI(TAG, "DS18B20 initialized on GPIO %d", pin);
}

/* Dva koraka: prvo kazem senzoru da izmeri (konverzija), pa onda procitam
 * rezultat. Ako bilo sta ne uspe, vracam gresku i main gasi grejac. */
esp_err_t mvtemperature_read(float *out_temperature_c)
{
    esp_err_t err = ds18b20_trigger_temperature_conversion(s_ds18b20);
    if (err != ESP_OK)
    {
        return err;
    }

    return ds18b20_get_temperature(s_ds18b20, out_temperature_c);
}
