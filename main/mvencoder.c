#include "mvencoder.h"
#include "esp_err.h"
#include "encoder.h"

static rotary_encoder_handle_t s_encoder;

// callback-ovi iz main-a, pamtim ih ovde da bih ih pozvala kad stigne dogadjaj
static mvencoder_rotate_cb_t s_on_rotate;
static mvencoder_button_cb_t s_on_button_click;

/* Ovo poziva biblioteka za svaki dogadjaj. Meni trebaju samo okretanje
 * i klik, ostalo (pritisnut/pusten/dugo drzanje) ignorisem. */
static void mvencoder_event_cb(const rotary_encoder_event_t *event, void *ctx)
{
    switch (event->type)
    {
    case RE_ET_CHANGED:
        if (s_on_rotate)
        {
            s_on_rotate(event->diff);
        }
        break;
    case RE_ET_BTN_CLICKED:
        if (s_on_button_click)
        {
            s_on_button_click();
        }
        break;
    default:
        break;
    }
}

void mvencoder_init(gpio_num_t pin_a, gpio_num_t pin_b, gpio_num_t pin_button,
                    mvencoder_rotate_cb_t on_rotate, mvencoder_button_cb_t on_button_click)
{
    s_on_rotate = on_rotate;
    s_on_button_click = on_button_click;

    // krecem od podrazumevane konfiguracije i menjam samo pinove i callback
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG();
    config.pin_a = pin_a;
    config.pin_b = pin_b;
    config.pin_btn = pin_button;
    config.callback = mvencoder_event_cb;

    ESP_ERROR_CHECK(rotary_encoder_create(&config, &s_encoder));
}

void mvencoder_enable_acceleration(uint16_t coeff)
{
    ESP_ERROR_CHECK(rotary_encoder_enable_acceleration(s_encoder, coeff));
}

void mvencoder_disable_acceleration(void)
{
    ESP_ERROR_CHECK(rotary_encoder_disable_acceleration(s_encoder));
}
