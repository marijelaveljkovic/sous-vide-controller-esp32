#include "mvbeeper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// trajanje bipova u ms
#define MVBEEPER_SHORT_BEEP_MS 100
#define MVBEEPER_LONG_BEEP_MS 500

static gpio_num_t s_beeper_pin;

/* upali buzzer, sacekaj, ugasi. vTaskDelay blokira task koji je pozvao
 * funkciju (max 500ms), ali za ovako kratke bipove to nije problem */
static void mvbeeper_beep(int duration_ms)
{
    gpio_set_level(s_beeper_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(s_beeper_pin, 0);
}

void mvbeeper_init(gpio_num_t pin)
{
    s_beeper_pin = pin;
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0); // da ne pisti odmah na paljenju
}

void mvbeeper_short_beep(void)
{
    mvbeeper_beep(MVBEEPER_SHORT_BEEP_MS);
}

void mvbeeper_long_beep(void)
{
    mvbeeper_beep(MVBEEPER_LONG_BEEP_MS);
}
