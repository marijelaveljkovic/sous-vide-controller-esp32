#pragma once

typedef void (*mvwifi_got_ip_cb_t)(void);

/* Pokrene povezivanje na WiFi i odmah se vrati, ne ceka da se poveze.
 * Ako ne uspe (pogresna lozinka, mreza daleko...) samo nastavlja da pokusava
 * u pozadini, a ostatak programa radi normalno.
 * on_got_ip se poziva (iz esp_event taska) svaki put kad se dobije IP. */
void mvwifi_connect(const char *ssid, const char *password, mvwifi_got_ip_cb_t on_got_ip);
