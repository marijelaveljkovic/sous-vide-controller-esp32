#pragma once

#include <stdint.h>

/* Zovem jednom na pocetku app_main, pre nego sto sensor_task krene,
 * jer ovde se pravi mutex za bafer sa uzorcima. */
void mvhttp_init(void);

// pokrece http server na portu 80 - zove se tek kad WiFi dobije IP adresu
void mvhttp_start(void);

/* Dodaje jedan uzorak (vreme, temperatura, cilj, PID izlaz) u kruzni bafer
 * koji cita /data. Moze da se zove iz sensor_task-a dok http task cita. */
void mvhttp_add_sample(uint32_t time_s, float temperature_c, float target_temperature_c, float pid_ratio);
