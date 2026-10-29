# Sous-Vide Controller (ESP32-C3)

Sous-vide cooking device built around the ESP32-C3 microcontroller. It keeps the water temperature at a set value using PID control and provides live monitoring through a WiFi web dashboard.

Developed as my BSc thesis *"Practical Implementation of a Sous-Vide Food Preparation System"* at the Faculty of Technical Sciences, University of Novi Sad (2026). 

## Features
- **PID temperature control** with anti-windup, an integral limit and a filtered derivative term
- **Time-proportional heater control** of a 2 kW heater through a solid-state relay (SSR)
- **Local user interface** – 16x2 LCD, rotary encoder with push button and a buzzer
- **Cooking workflow** – set temperature and time; the countdown starts only after the water reaches the set temperature
- **Safety** – the heater is switched off immediately if the temperature sensor fails or the cooking time expires
- **WiFi web dashboard** – live chart of temperature, setpoint and controller output

## Hardware
| Component | Interface | GPIO |
|---|---|---|
| ESP32-C3 | – | – |
| DS18B20 temperature sensor | 1-Wire | 5 |
| Solid-state relay (heater) | Digital output | 0 |
| 16x2 LCD with PCF8574 I2C module (address 0x27) | I2C | SDA 7, SCL 6 |
| Rotary encoder with push button | Digital inputs | CLK 1, DT 2, SW 3 |
| Active buzzer | Digital output | 4 |

## How it works

### Operation
The device runs as a state machine:

```
Set temperature ──click──► Set time ──click──► Running ──time expired──► Done
   (0–100 °C)              (minutes)        (preheating,
                                             then countdown)
```

The encoder sets the values and the button confirms them. While setting the time, the step grows with the value, so long cooking times can be set quickly.

### Temperature control
- The DS18B20 is read once per second with 12-bit resolution (0.0625 °C).
- A discrete PID controller (Kp = 6, Ki = 0.02, Kd = 4, sample time 1 s) computes the heater power from 0 to 100 %.
- **Anti-windup:** conditional integration stops the integral from growing while the output is saturated, and an additional integral limit prevents overshoot during the slow approach to the setpoint.
- **Derivative filter:** an exponential filter on the derivative term reduces the effect of sensor noise.
- **Time-proportional control:** the SSR cannot follow a high-frequency PWM signal, so the heater is switched in a 1 s window with 50 ms resolution (e.g. 30 % = 300 ms on, 700 ms off).

The PID parameters were tuned experimentally in real cooking tests, which **reduced temperature overshoot from about 3 °C to about 1 °C**.

### Firmware structure
- A periodic timer (1 s) wakes the control task through a FreeRTOS task notification. Sensor reading, PID calculation and display updates run in that task, so the timer callback stays short and does not block the encoder handling.
- A second timer (50 ms) drives the SSR time-proportional output.
- WiFi connects in the background; the device works normally without a network, and the web server starts once an IP address is obtained.
- Samples are stored in a circular buffer (last 300 samples, i.e. 5 minutes) protected by a mutex, because it is written by the control task and read by the web server.

| Module | Responsibility |
|---|---|
| `main.c` | State machine, PID controller, SSR control, control task |
| `mvtemperature` | DS18B20 driver wrapper (1-Wire over RMT) |
| `mvlcd` | I2C LCD driver wrapper |
| `mvencoder` | Rotary encoder and button handling |
| `mvbeeper` | Buzzer |
| `mvwifi` | WiFi connection in station mode |
| `mvhttp` | HTTP server, web dashboard and sample buffer |

## Web dashboard
When the device connects to WiFi, its IP address is printed in the serial log. Open `http://<device-ip>/` in a browser and click **Load** to plot temperature, setpoint and PID output.

| Endpoint | Description |
|---|---|
| `/` | Web page with the chart (Chart.js) |
| `/data` | Recorded samples as CSV: `time_s, temperature, target, pid_output` |

## Building and flashing

### Requirements
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/index.html) v6.0.2
- ESP32-C3 board and the hardware listed above

### WiFi configuration
WiFi credentials are kept out of the repository. Copy the template and enter your network details:
```bash
cp main/wifi_credentials.example.h main/wifi_credentials.h
```

### Build and flash
```bash
idf.py build
idf.py -p PORT flash monitor
```
Required libraries (DS18B20, 1-Wire bus, rotary encoder) are downloaded automatically by the ESP-IDF Component Manager.

## Documentation
- [BSc thesis (Serbian)](docs/thesis-sr.pdf)

## Tech stack
C · ESP-IDF · FreeRTOS · ESP32-C3 · Chart.js

## Acknowledgments
LCD driver: [iamflinks/i2c_lcd_pcf8574](https://github.com/iamflinks/i2c_lcd_pcf8574) (Apache License 2.0), included in `components/`.