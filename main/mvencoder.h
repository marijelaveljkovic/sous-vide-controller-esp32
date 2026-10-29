#pragma once

#include <stdint.h>
#include "driver/gpio.h"

/* Moj omotac oko esp-idf-lib/encoder biblioteke (ona radi dekodiranje
 * kvadraturnog signala i debounce tastera, ja samo dobijam gotove dogadjaje). */

/* diff = za koliko koraka je enkoder pomeren (pozitivno = desno, negativno = levo).
 * Ako je ukljuceno ubrzanje, kod brzog okretanja diff moze biti i veci od 1. */
typedef void (*mvencoder_rotate_cb_t)(int32_t diff);
typedef void (*mvencoder_button_cb_t)(void);

// pin_a i pin_b su kanali A i B, pin_button je taster na osovini
void mvencoder_init(gpio_num_t pin_a, gpio_num_t pin_b, gpio_num_t pin_button,
                     mvencoder_rotate_cb_t on_rotate, mvencoder_button_cb_t on_button_click);

/* Ubrzanje: brzo okretanje daje veci diff, sporo i dalje +-1.
 * Sto je veci coeff, to je ubrzanje jace. */
void mvencoder_enable_acceleration(uint16_t coeff);
void mvencoder_disable_acceleration(void);
