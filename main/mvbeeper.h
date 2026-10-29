#pragma once

#include "driver/gpio.h"

// aktivni buzzer - ima svoj oscilator, pa ga samo palim/gasim preko GPIO
void mvbeeper_init(gpio_num_t pin);
void mvbeeper_short_beep(void); // potvrda unosa / dostignut cilj
void mvbeeper_long_beep(void);  // kraj kuvanja
