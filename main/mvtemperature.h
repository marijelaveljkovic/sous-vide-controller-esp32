#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

// DS18B20 senzor na 1-Wire magistrali (jedan pin + pull-up 4.7k)
void mvtemperature_init(gpio_num_t pin);
// vraca ESP_OK ako je citanje uspelo, temperatura se upisuje u *out_temperature_c
esp_err_t mvtemperature_read(float *out_temperature_c);
