#include "mvlcd.h"
#include "esp_err.h"
#include "i2c_lcd_pcf8574.h"

static i2c_lcd_pcf8574_handle_t s_lcd;

/* ESP32 je master na I2C magistrali. Interni pull-up ukljucujem za svaki
 * slucaj, iako I2C modul na displeju vec ima svoje pull-up otpornike. */
static void mvlcd_i2c_master_init(gpio_num_t pin_sda, gpio_num_t pin_scl, i2c_port_t i2c_port, uint32_t i2c_freq_hz)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pin_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = pin_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_freq_hz,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &config));
    // master ne treba rx/tx bafere, zato su 0
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, config.mode, 0, 0, 0));
}

void mvlcd_init(gpio_num_t pin_sda, gpio_num_t pin_scl, i2c_port_t i2c_port, uint32_t i2c_freq_hz,
                 uint8_t i2c_addr, uint8_t cols, uint8_t rows)
{
    // prvo I2C, pa tek onda LCD (biblioteka salje komande preko I2C-a)
    mvlcd_i2c_master_init(pin_sda, pin_scl, i2c_port, i2c_freq_hz);

    lcd_init(&s_lcd, i2c_addr, i2c_port);
    lcd_begin(&s_lcd, cols, rows);
}

// ostale funkcije samo prosledjuju poziv biblioteci

void mvlcd_clear(void)
{
    lcd_clear(&s_lcd);
}

void mvlcd_home(void)
{
    lcd_home(&s_lcd);
}

void mvlcd_set_cursor(uint8_t col, uint8_t row)
{
    lcd_set_cursor(&s_lcd, col, row);
}

void mvlcd_print(const char *str)
{
    lcd_print(&s_lcd, str);
}

void mvlcd_set_backlight(uint8_t brightness)
{
    lcd_set_backlight(&s_lcd, brightness);
}
