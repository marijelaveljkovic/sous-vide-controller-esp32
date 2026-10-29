#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "mvlcd.h"
#include "mvbeeper.h"
#include "mvtemperature.h"
#include "mvencoder.h"
#include "mvwifi.h"
#include "mvhttp.h"

// podaci za WiFi su u wifi_credentials.h (nije na GitHub-u, vidi wifi_credentials.example.h)
#include "wifi_credentials.h"

// SSR rele koji pali/gasi grejac
#define SSR_PIN GPIO_NUM_0

// LCD 16x2 preko I2C modula (PCF8574), adresa 0x27 je fabricka
#define LCD_PIN_SDA GPIO_NUM_7
#define LCD_PIN_SCL GPIO_NUM_6
#define LCD_I2C_PORT I2C_NUM_0
#define LCD_I2C_FREQ_HZ 100000
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// DS18B20 (1-Wire) i aktivni buzzer - svako na svom pinu
#define TEMP_SENSOR_PIN GPIO_NUM_5
#define BEEPER_PIN GPIO_NUM_4

// enkoder: SW je taster, DT i CLK su kanali A i B
#define ENCODER_PIN_SW GPIO_NUM_3
#define ENCODER_PIN_DT GPIO_NUM_2
#define ENCODER_PIN_CLK GPIO_NUM_1

// senzor citam jednom u sekundi (u mikrosekundama jer to trazi esp_timer)
#define TEMP_READ_PERIOD_US (1000 * 1000)
// na svakih 100 min korak za vreme se poveca za 1 (vidi time_step_for_value)
#define TIME_STEP_BRACKET 100

/* PWM za SSR - ne moze hardverski LEDC jer je on za kHz frekvencije, a SSR pali
 * na prolasku kroz nulu i to ne moze da prati. Zato radim "time proportioning":
 * prozor od 1s, i u njemu je grejac ukljucen onoliko % vremena koliko kaze PID.
 * Tajmer proverava na svakih 50ms, znaci imam 20 koraka po prozoru. */
#define SSR_PWM_WINDOW_MS 1000
#define SSR_PWM_TICK_MS 50

/* PID konstante - nasla sam ih eksperimentalno.
 * Prvo sam imala Kp=8, Ki=0.05, Kd=2 i na testu (skampi, 56C) temperatura je
 * otisla do ~59C. Problem je bio sto je integral rastao dok se polako prilazilo
 * cilju (anti-windup ga zaustavlja samo kad je izlaz na 100%), pa je gurao
 * dodatnu snagu bas kad je trebalo da koci. Zato:
 *  - Kp 8 -> 6 (i dalje glavni clan)
 *  - Ki 0.05 -> 0.02 (sporiji, sluzi samo da skloni ustaljenu gresku)
 *  - Kd 2 -> 4 (ranije koci, kompenzuje tromost grejaca i vode)
 * Posle ovoga overshoot je oko 1C umesto 3C. */
#define PID_KP 6.0f
#define PID_KI 0.02f
#define PID_KD 4.0f
#define PID_OUTPUT_MIN 0.0f
#define PID_OUTPUT_MAX 100.0f
#define PID_DT_S 1.0f // mora da bude isto kao period citanja senzora (1s)

/* Gornja granica za integral (C*s). Ovo je dodatno uz anti-windup, jer on ne
 * pokriva fazu sporog prilaska cilju. Max doprinos I clana je 500 * 0.02 = 10%. */
#define PID_INTEGRAL_LIMIT 500.0f

/* Filter za D clan (0..1, manje = jace filtriranje). Posto sam povecala Kd,
 * D clan je postao osetljiv na sum senzora i rele bi bez potrebe "cackao". */
#define PID_DERIVATIVE_FILTER_ALPHA 0.3f

// stanja automata: temp -> vreme -> kuvanje -> gotovo
typedef enum
{
    APP_STATE_SET_TEMPERATURE,
    APP_STATE_SET_TIME,
    APP_STATE_RUNNING,
    APP_STATE_DONE,
} app_state_t;

// pocetne vrednosti kad se upali uredjaj: 60C i 60 min
static app_state_t s_state = APP_STATE_SET_TEMPERATURE;
static int32_t s_target_temperature = 60;
static int32_t s_time_value_min = 60;
static int32_t s_remaining_seconds = 0;

// preheating = jos se zagreva, vreme ne tece dok se ne stigne do cilja
static bool s_preheating = false;
// koliko sekundi radi kuvanje (ovo ide kao x osa na grafiku)
static uint32_t s_run_elapsed_s = 0;

// poslednja procitana temperatura i da li je citanje uspelo
static float s_current_temperature_c = 0.0f;
static bool s_current_temperature_valid = false;

// stanje PID-a koje se pamti izmedju dva poziva
static float s_pid_integral = 0.0f;
static float s_pid_prev_error = 0.0f;
static bool s_pid_has_prev_error = false; // u prvom koraku nema prethodne greske pa nema ni D clana
static float s_pid_filtered_derivative = 0.0f;

/* Snaga grejaca 0-100%. PID je upisuje jednom u sekundi, a ssr_pwm_tick je
 * cita na 50ms da zna da li SSR treba da bude ukljucen. */
static float s_heater_duty_percent = 0.0f;
// koliko je ms proslo od pocetka trenutnog prozora od 1s
static uint32_t s_pwm_window_elapsed_ms = 0;

// direktno postavlja pin SSR-a (1 = grejac radi)
static void ssr_write(bool on)
{
    gpio_set_level(SSR_PIN, on ? 1 : 0);
}

static void ssr_init(void)
{
    gpio_reset_pin(SSR_PIN);
    gpio_set_direction(SSR_PIN, GPIO_MODE_OUTPUT);
    ssr_write(false); // na startu grejac mora biti ugasen
}

// samo upisuje zeljenu snagu (ogranicenu na 0-100), pin menja tek ssr_pwm_tick
static void ssr_set_power(float power_percent)
{
    if (power_percent < 0.0f)
    {
        power_percent = 0.0f;
    }
    if (power_percent > 100.0f)
    {
        power_percent = 100.0f;
    }
    s_heater_duty_percent = power_percent;
}

/* Gasi grejac ODMAH, ne ceka kraj prozora. Koristim kad senzor ne radi ili
 * kad je kuvanje gotovo - tu ne sme da se ceka jos do 1s. */
static void ssr_force_off(void)
{
    s_heater_duty_percent = 0.0f;
    ssr_write(false);
}

/* Softverski PWM - zove ga tajmer na svakih 50ms.
 * Na pocetku prozora pali SSR i drzi ga upaljenog dok ne prodje duty% * 1000ms,
 * onda gasi do kraja prozora. npr. 30% -> 300ms upaljen, 700ms ugasen.
 * Radi u esp_timer tasku, pa mora biti kratko i bez blokiranja. */
static void ssr_pwm_tick(void *arg)
{
    (void)arg;

    s_pwm_window_elapsed_ms += SSR_PWM_TICK_MS;
    if (s_pwm_window_elapsed_ms >= SSR_PWM_WINDOW_MS)
    {
        s_pwm_window_elapsed_ms = 0; // krece novi prozor
    }

    uint32_t on_time_ms = (uint32_t)((s_heater_duty_percent / 100.0f) * SSR_PWM_WINDOW_MS);
    ssr_write(s_pwm_window_elapsed_ms < on_time_ms);
}

// ogranicava vrednost na opseg [min, max] (za int)
static int32_t clamp_value(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/* Korak za vreme raste sto je vreme vece: ispod 100 min korak je 1,
 * 100-199 korak 2, 200-299 korak 3... da ne bih vrtela enkoder satima
 * kad hocu npr. 24h. */
static int32_t time_step_for_value(int32_t value)
{
    return (value / TIME_STEP_BRACKET) + 1;
}

// isto kao clamp_value samo za float
static float clamp_valuef(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

// brise sve sto je PID zapamtio - zovem pre svakog novog kuvanja
static void pid_reset(void)
{
    s_pid_integral = 0.0f;
    s_pid_prev_error = 0.0f;
    s_pid_has_prev_error = false;
    s_pid_filtered_derivative = 0.0f;
}

/* Diskretni PID: u = Kp*e + Ki*suma(e*dt) + Kd*(de/dt)
 * e = zadata - izmerena, pa kad je voda hladnija e > 0 i izlaz raste.
 * Vraca snagu grejaca 0-100%.
 *
 * Anti-windup (conditional integration): integral prihvatam samo ako izlaz
 * nije zasicen, ili ako bi greska vratila izlaz nazad u opseg. Inace bi tokom
 * zagrevanja (velika greska, izlaz stalno 100%) integral rastao bez kraja
 * i posle napravio ogroman overshoot. */
static float pid_compute(float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // "probni" integral - jos ga ne upisujem dok ne vidim da li je izlaz zasicen
    float unclamped_integral = s_pid_integral + error * PID_DT_S;
    // izvod = razlika dve uzastopne greske / dt
    float raw_derivative = s_pid_has_prev_error ? (error - s_pid_prev_error) / PID_DT_S : 0.0f;
    // eksponencijalni filter: novo = alfa*sirovo + (1-alfa)*staro
    s_pid_filtered_derivative = s_pid_has_prev_error
                                    ? (PID_DERIVATIVE_FILTER_ALPHA * raw_derivative + (1.0f - PID_DERIVATIVE_FILTER_ALPHA) * s_pid_filtered_derivative)
                                    : 0.0f;
    s_pid_prev_error = error;
    s_pid_has_prev_error = true;

    float output = PID_KP * error + PID_KI * unclamped_integral + PID_KD * s_pid_filtered_derivative;

    bool saturated_high = output > PID_OUTPUT_MAX;
    bool saturated_low = output < PID_OUTPUT_MIN;
    if (!saturated_high && !saturated_low)
    {
        // nije zasicen -> normalno integrisem
        s_pid_integral = unclamped_integral;
    }
    else if (saturated_high && error < 0)
    {
        // zasicen gore, ali greska je negativna pa integral ide nadole -> to je ok
        s_pid_integral = unclamped_integral;
    }
    else if (saturated_low && error > 0)
    {
        // zasicen dole, a greska pozitivna -> integral raste, i to je ok
        s_pid_integral = unclamped_integral;
    }
    // u ostalim slucajevima integral ostaje isti ("zamrznut")

    /* tvrda granica za integral - anti-windup gore ne pomaze dok se polako prilazi
     * cilju (izlaz nije zasicen), a bas tada je integral pravio overshoot */
    s_pid_integral = clamp_valuef(s_pid_integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    return clamp_valuef(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
}

/* Ispisuje oba reda na LCD u zavisnosti od stanja.
 * Zove se iz enkoder callback-a (odmah kad korisnik nesto promeni) i iz
 * sensor_task-a (jednom u sekundi). */
static void render_lcd(void)
{
    char content0[32];
    char content1[32];

    switch (s_state)
    {
    case APP_STATE_SET_TEMPERATURE:
        snprintf(content0, sizeof(content0), "Odaberi temp:");
        snprintf(content1, sizeof(content1), "%" PRId32 " C", s_target_temperature);
        break;

    case APP_STATE_SET_TIME:
        snprintf(content0, sizeof(content0), "Odaberi vreme:");
        snprintf(content1, sizeof(content1), "%" PRId32 " min", s_time_value_min);
        break;

    case APP_STATE_DONE:
        snprintf(content0, sizeof(content0), "Gotovo!");
        content1[0] = '\0';
        break;

    case APP_STATE_RUNNING:
    default:
        // prvi red: Z = zadata, T = trenutna (ili ERR ako senzor nije odgovorio)
        if (s_current_temperature_valid)
        {
            snprintf(content0, sizeof(content0), "Z:%" PRId32 " T:%.1fC", s_target_temperature, (double)s_current_temperature_c);
        }
        else
        {
            snprintf(content0, sizeof(content0), "Z:%" PRId32 " T:ERR", s_target_temperature);
        }
        // drugi red: dok se zagreva pise "Zagreva...", posle preostalo vreme mm:ss
        if (s_preheating)
        {
            snprintf(content1, sizeof(content1), "Zagreva...");
        }
        else
        {
            int32_t minutes = s_remaining_seconds / 60;
            int32_t seconds = s_remaining_seconds % 60;
            snprintf(content1, sizeof(content1), "Vreme: %" PRId32 ":%02" PRId32, minutes, seconds);
        }
        break;
    }

    /* dopunjavam razmacima do 16 karaktera - tako se pregaze stari znakovi
     * i ne moram da zovem lcd_clear (koji bi pravio treperenje) */
    char line0[32];
    char line1[32];
    snprintf(line0, sizeof(line0), "%-16s", content0);
    snprintf(line1, sizeof(line1), "%-16s", content1);

    mvlcd_set_cursor(0, 0);
    mvlcd_print(line0);
    mvlcd_set_cursor(0, 1);
    mvlcd_print(line1);
}

// okretanje enkodera: diff je broj koraka (+ desno, - levo)
static void on_encoder_rotate(int32_t diff)
{
    switch (s_state)
    {
    case APP_STATE_SET_TEMPERATURE:
        // temperatura ide direktno za diff, u opsegu 0-100C
        s_target_temperature = clamp_value(s_target_temperature + diff, 0, 100);
        printf("Cilj temperature: %" PRId32 " C\n", s_target_temperature);
        break;
    case APP_STATE_SET_TIME:
    {
        /* za vreme idem klik po klik, jer se korak menja u zavisnosti od
         * trenutne vrednosti (ako bih samo pomnozila diff sa korakom,
         * preskocila bih granicu od 100/200... sa pogresnim korakom) */
        int32_t direction = diff > 0 ? 1 : -1;
        int32_t clicks = diff > 0 ? diff : -diff;
        for (int32_t i = 0; i < clicks; i++)
        {
            int32_t step = time_step_for_value(s_time_value_min);
            s_time_value_min = clamp_value(s_time_value_min + direction * step, 1, 3600 * 24 * 5);
        }
        printf("Vreme: %" PRId32 " min\n", s_time_value_min);
        break;
    }
    case APP_STATE_RUNNING:
    case APP_STATE_DONE:
    default:
        return; // dok kuva ili je gotovo, enkoder ne radi nista
    }
    render_lcd();
}

// klik na enkoder = potvrda i prelazak u sledece stanje
static void on_encoder_button_click(void)
{
    switch (s_state)
    {
    case APP_STATE_SET_TEMPERATURE:
        s_state = APP_STATE_SET_TIME;
        printf("Potvrdjena temperatura: %" PRId32 " C, sada biramo vreme\n", s_target_temperature);
        break;
    case APP_STATE_SET_TIME:
        // krece kuvanje: pripremim vreme, PID od nule, i prvo faza zagrevanja
        s_state = APP_STATE_RUNNING;
        s_remaining_seconds = s_time_value_min * 60;
        s_preheating = true;
        s_run_elapsed_s = 0;
        pid_reset();
        printf("Potvrdjeno vreme: %" PRId32 " min, zagrevanje pocinje\n", s_time_value_min);
        break;
    case APP_STATE_RUNNING:
    case APP_STATE_DONE:
    default:
        return; // klik se ignorise dok kuva i kad je gotovo (restart vraca na pocetak)
    }
    mvbeeper_short_beep(); // kratak bip = potvrda unosa
    render_lcd();
}

static TaskHandle_t s_sensor_task_handle;

/* Callback tajmera (1s). Radi u esp_timer tasku, a u istom tasku enkoder
 * biblioteka proverava pinove na ~1ms. Zato ovde NE citam senzor (konverzija
 * traje ~750ms) niti pisem na LCD - samo probudim sensor_task. Da sam to radila
 * ovde, enkoder bi gubio korake i klikove. */
static void on_temperature_timer(void *arg)
{
    (void)arg;
    xTaskNotifyGive(s_sensor_task_handle);
}

/* Glavni task regulacije - budi se jednom u sekundi.
 * Redom: procita senzor -> ako kuvanje traje racuna PID i postavi snagu ->
 * upise uzorak za grafik -> broji vreme -> osvezi LCD.
 * Vreme pocinje da tece tek kad voda prvi put dostigne zadatu temperaturu. */
static void sensor_task(void *arg)
{
    (void)arg;

    while (1)
    {
        // spava dok ga tajmer ne probudi
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float temperature_c;
        esp_err_t err = mvtemperature_read(&temperature_c);
        s_current_temperature_valid = (err == ESP_OK);
        if (s_current_temperature_valid)
        {
            s_current_temperature_c = temperature_c;
            printf("Temperatura: %.1f C\n", temperature_c);
        }
        else
        {
            printf("Greska pri citanju temperature\n");
        }

        if (s_state == APP_STATE_RUNNING)
        {
            if (s_current_temperature_valid)
            {
                float power = pid_compute((float)s_target_temperature, temperature_c);
                ssr_set_power(power);
                printf("PID izlaz (snaga grejaca): %.1f%%\n", power);
                // uzorak za grafik na web stranici
                mvhttp_add_sample(s_run_elapsed_s, temperature_c, (float)s_target_temperature, power);
                s_run_elapsed_s++;

                if (s_preheating && temperature_c >= (float)s_target_temperature)
                {
                    /* prvi put dostignut cilj -> kraj zagrevanja, krece odbrojavanje.
                     * PID i dalje radi isto, samo sada drzi temperaturu. */
                    s_preheating = false;
                    mvbeeper_short_beep();
                    printf("Cilj dostignut, pocinje odbrojavanje\n");
                }
            }
            else
            {
                // senzor nije dao ispravnu vrednost -> ne pogadjam, gasim grejac odmah
                ssr_force_off();
            }

            // odbrojavanje (samo posle faze zagrevanja)
            if (!s_preheating && s_remaining_seconds > 0)
            {
                s_remaining_seconds--;
            }
            // isteklo vreme -> kraj, gasim grejac i dug bip
            if (!s_preheating && s_remaining_seconds <= 0)
            {
                s_state = APP_STATE_DONE;
                ssr_force_off();
                mvbeeper_long_beep();
                printf("Gotovo, vreme isteklo\n");
            }
        }

        render_lcd();
    }
}

void app_main(void)
{
    // SSR prvi, da grejac sigurno bude ugasen od samog starta
    ssr_init();

    /* mvhttp_init mora pre sensor_task-a jer pravi mutex za bafer uzoraka.
     * mvwifi_connect ne blokira - ako nema mreze, uredjaj radi normalno,
     * a http server se pokrene tek kad se dobije IP (mvhttp_start kao callback). */
    mvhttp_init();
    mvwifi_connect(WIFI_SSID, WIFI_PASSWORD, mvhttp_start);

    mvbeeper_init(BEEPER_PIN);
    mvtemperature_init(TEMP_SENSOR_PIN);

    mvlcd_init(LCD_PIN_SDA, LCD_PIN_SCL, LCD_I2C_PORT, LCD_I2C_FREQ_HZ,
               LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
    mvlcd_set_backlight(255);

    // CLK = kanal A, DT = kanal B
    mvencoder_init(ENCODER_PIN_CLK, ENCODER_PIN_DT, ENCODER_PIN_SW,
                   on_encoder_rotate, on_encoder_button_click);
    // ubrzanje: kad brzo okrecem, vrednost se menja vise po koraku
    mvencoder_enable_acceleration(50);

    // test na paljenju: bip + kratko upali SSR (200ms) da vidim/cujem da oba izlaza rade
    mvbeeper_short_beep();
    ssr_write(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    ssr_write(false);

    mvlcd_clear();
    render_lcd();

    // task za senzor + PID + LCD (stack 4096, prioritet 5)
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, &s_sensor_task_handle);

    // tajmer od 1s koji budi sensor_task
    const esp_timer_create_args_t timer_args = {
        .callback = &on_temperature_timer,
        .name = "temp_timer",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, TEMP_READ_PERIOD_US));

    /* drugi, brzi tajmer (50ms) samo za PWM prozor SSR-a - odvojen od tajmera
     * za temperaturu jer mora da otkucava mnogo cesce */
    const esp_timer_create_args_t ssr_timer_args = {
        .callback = &ssr_pwm_tick,
        .name = "ssr_pwm_timer",
    };
    esp_timer_handle_t ssr_timer;
    ESP_ERROR_CHECK(esp_timer_create(&ssr_timer_args, &ssr_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(ssr_timer, SSR_PWM_TICK_MS * 1000)); // ms -> us

    printf("Sous vide test spreman: enkoder -> temperatura -> klik -> vreme -> klik -> start.\n");
}
