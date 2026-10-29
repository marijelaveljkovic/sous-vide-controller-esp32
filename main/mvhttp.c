#include "mvhttp.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "mvhttp";

// 300 uzoraka po 1 u sekundi = poslednjih 5 minuta rada
#define MVHTTP_MAX_SAMPLES 300

// jedan red u CSV-u: vreme, temperatura, cilj, PID izlaz u %
typedef struct
{
    uint32_t time_s;
    float temperature_c;
    float target_temperature_c;
    float pid_ratio;
} mvhttp_sample_t;

/* Kruzni bafer - kad se napuni, novi uzorak pregazi najstariji.
 * sensor_task upisuje (1/s), http task cita kad stigne zahtev za /data.
 * To su dva razlicita taska, zato mutex.
 * s_snapshot je kopija za slanje, da ne drzim mutex dok saljem preko mreze. */
static mvhttp_sample_t s_samples[MVHTTP_MAX_SAMPLES];
static mvhttp_sample_t s_snapshot[MVHTTP_MAX_SAMPLES];
static size_t s_write_index = 0;
static size_t s_count = 0;
static SemaphoreHandle_t s_samples_mutex = NULL;
static httpd_handle_t s_server = NULL;

/* Cela web stranica kao jedan string (HTML + CSS + JS).
 * Na klik "Load" povuce CSV sa /data i nacrta grafik.
 * Chart.js se ucitava sa CDN-a u browseru, ESP ga ne cuva u sebi. */
static const char s_index_html[] =
    "<!DOCTYPE html>\n"
    "<html lang='en'>\n"
    "<head>\n"
    "<meta charset='UTF-8'>\n"
    "<title>Sous Vide Monitor</title>\n"
    "<script src='https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js'></script>\n"
    "<style>\n"
    "body { font-family: sans-serif; margin: 20px; background: #f5f5f5; }\n"
    "h1 { font-size: 20px; }\n"
    "#controls { margin-bottom: 16px; }\n"
    "input { padding: 6px; font-size: 14px; width: 160px; }\n"
    "button { padding: 6px 14px; font-size: 14px; margin-left: 8px; }\n"
    "#status { margin-left: 10px; font-size: 13px; color: #555; }\n"
    "#chartWrap { background: #fff; padding: 10px; border-radius: 6px; max-width: 900px; }\n"
    "</style>\n"
    "</head>\n"
    "<body>\n"
    "<h1>Sous Vide Monitor</h1>\n"
    "<div id='controls'>\n"
    "<input id='ipInput' type='text' placeholder='Device IP address' value=''>\n"
    "<button id='loadBtn'>Load</button>\n"
    "<span id='status'></span>\n"
    "</div>\n"
    "<div id='chartWrap'>\n"
    "<canvas id='chart'></canvas>\n"
    "</div>\n"
    "<script>\n"
    "document.getElementById('ipInput').value = window.location.hostname || '';\n"
    "\n"
    "var chart = null;\n"
    "\n"
    "function parseCsv(text) {\n"
    "  var lines = text.split('\\n');\n"
    "  var labels = [], temp = [], target = [], pid = [];\n"
    "  for (var i = 0; i < lines.length; i++) {\n"
    "    var line = lines[i].trim();\n"
    "    if (!line) continue;\n"
    "    var parts = line.split(',');\n"
    "    if (parts.length < 4) continue;\n"
    "    labels.push(parts[0]);\n"
    "    temp.push(parseFloat(parts[1]));\n"
    "    target.push(parseFloat(parts[2]));\n"
    "    pid.push(parseFloat(parts[3]));\n"
    "  }\n"
    "  return { labels: labels, temp: temp, target: target, pid: pid };\n"
    "}\n"
    "\n"
    "function loadData() {\n"
    "  var ip = document.getElementById('ipInput').value.trim();\n"
    "  var statusEl = document.getElementById('status');\n"
    "  if (!ip) { statusEl.textContent = 'Enter an IP address'; return; }\n"
    "  statusEl.textContent = 'Loading...';\n"
    "  fetch('http://' + ip + '/data')\n"
    "    .then(function(resp) {\n"
    "      if (!resp.ok) throw new Error('HTTP ' + resp.status);\n"
    "      return resp.text();\n"
    "    })\n"
    "    .then(function(text) {\n"
    "      var data = parseCsv(text);\n"
    "      statusEl.textContent = data.labels.length + ' samples';\n"
    "      renderChart(data);\n"
    "    })\n"
    "    .catch(function(err) {\n"
    "      statusEl.textContent = 'Error: ' + err.message;\n"
    "    });\n"
    "}\n"
    "\n"
    "function renderChart(data) {\n"
    "  var ctx = document.getElementById('chart').getContext('2d');\n"
    "  if (chart) chart.destroy();\n"
    "  chart = new Chart(ctx, {\n"
    "    type: 'line',\n"
    "    data: {\n"
    "      labels: data.labels,\n"
    "      datasets: [\n"
    "        { label: 'Temperature (C)', data: data.temp, borderColor: '#e74c3c', backgroundColor: 'transparent', yAxisID: 'yTemp', tension: 0.2, pointRadius: 0 },\n"
    "        { label: 'Target (C)', data: data.target, borderColor: '#3498db', backgroundColor: 'transparent', yAxisID: 'yTemp', borderDash: [5, 5], tension: 0.2, pointRadius: 0 },\n"
    "        { label: 'PID ratio (%)', data: data.pid, borderColor: '#2ecc71', backgroundColor: 'transparent', yAxisID: 'yPid', tension: 0.2, pointRadius: 0 }\n"
    "      ]\n"
    "    },\n"
    "    options: {\n"
    "      animation: false,\n"
    "      scales: {\n"
    "        x: { title: { display: true, text: 'Time (s)' } },\n"
    "        yTemp: { type: 'linear', position: 'left', title: { display: true, text: 'Temperature (C)' } },\n"
    "        yPid: { type: 'linear', position: 'right', title: { display: true, text: 'PID ratio (%)' }, min: 0, max: 100, grid: { drawOnChartArea: false } }\n"
    "      }\n"
    "    }\n"
    "  });\n"
    "}\n"
    "\n"
    "document.getElementById('loadBtn').addEventListener('click', loadData);\n"
    "</script>\n"
    "</body>\n"
    "</html>\n";

// GET / -> vraca HTML stranicu
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, s_index_html, HTTPD_RESP_USE_STRLEN);
}

// GET /data -> vraca sve uzorke kao CSV, od najstarijeg ka najnovijem
static esp_err_t data_get_handler(httpd_req_t *req)
{
    size_t count;

    // pod mutex-om samo brzo prekopiram uzorke, slanje ide posle
    xSemaphoreTake(s_samples_mutex, portMAX_DELAY);
    count = s_count;
    // indeks najstarijeg uzorka (+MAX da ne bi otislo u minus pre %)
    size_t start = (s_write_index + MVHTTP_MAX_SAMPLES - count) % MVHTTP_MAX_SAMPLES;
    for (size_t i = 0; i < count; i++)
    {
        s_snapshot[i] = s_samples[(start + i) % MVHTTP_MAX_SAMPLES];
    }
    xSemaphoreGive(s_samples_mutex);

    // saljem red po red (chunked), da ne pravim jedan veliki string u memoriji
    httpd_resp_set_type(req, "text/csv");
    char line[64];
    for (size_t i = 0; i < count; i++)
    {
        int len = snprintf(line, sizeof(line), "%" PRIu32 ",%.2f,%.2f,%.1f\n",
                            s_snapshot[i].time_s, s_snapshot[i].temperature_c,
                            s_snapshot[i].target_temperature_c, s_snapshot[i].pid_ratio);
        esp_err_t err = httpd_resp_send_chunk(req, line, len);
        if (err != ESP_OK)
        {
            // greska pri slanju -> zatvorim odgovor i prekidam
            httpd_resp_send_chunk(req, NULL, 0);
            return err;
        }
    }
    // prazan chunk = kraj odgovora
    return httpd_resp_send_chunk(req, NULL, 0);
}

void mvhttp_init(void)
{
    s_samples_mutex = xSemaphoreCreateMutex();
}

/* Zove se iz mvwifi svaki put kad se dobije IP - i posle ponovnog povezivanja
 * kad WiFi pukne. Zato proveravam s_server: server se pokrece samo prvi put,
 * svaki sledeci poziv odmah izlazi. */
void mvhttp_start(void)
{
    if (s_server != NULL)
    {
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGW(TAG, "Pokretanje http servera nije uspelo");
        return;
    }

    // dve rute: "/" za stranicu i "/data" za podatke
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_get_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &data_uri);

    s_server = server;
    ESP_LOGI(TAG, "Http server pokrenut na portu %d", config.server_port);
}

void mvhttp_add_sample(uint32_t time_s, float temperature_c, float target_temperature_c, float pid_ratio)
{
    // zastita ako neko zaboravi mvhttp_init - bez mutex-a ne upisujem nista
    if (s_samples_mutex == NULL)
    {
        return;
    }

    xSemaphoreTake(s_samples_mutex, portMAX_DELAY);
    s_samples[s_write_index] = (mvhttp_sample_t){
        .time_s = time_s,
        .temperature_c = temperature_c,
        .target_temperature_c = target_temperature_c,
        .pid_ratio = pid_ratio,
    };
    // pomeram indeks i vracam se na 0 kad dodjem do kraja niza
    s_write_index = (s_write_index + 1) % MVHTTP_MAX_SAMPLES;
    // count raste samo dok se bafer ne napuni prvi put
    if (s_count < MVHTTP_MAX_SAMPLES)
    {
        s_count++;
    }
    xSemaphoreGive(s_samples_mutex);
}
