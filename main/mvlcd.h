#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

/* Omotac oko i2c_lcd_pcf8574 biblioteke za LCD 16x2 sa I2C modulom.
 * Ovako main ne mora da zna nista o samoj biblioteci. */

// podesi I2C (SDA, SCL, frekvencija) i pokrene LCD na zadatoj adresi
void mvlcd_init(gpio_num_t pin_sda, gpio_num_t pin_scl, i2c_port_t i2c_port, uint32_t i2c_freq_hz,
                 uint8_t i2c_addr, uint8_t cols, uint8_t rows);

void mvlcd_clear(void);
void mvlcd_home(void);
void mvlcd_set_cursor(uint8_t col, uint8_t row); // kolona 0-15, red 0-1
void mvlcd_print(const char *str);
void mvlcd_set_backlight(uint8_t brightness);
