# Palomar Dome Node

### Experimental Sky Quality & Atmospheric Monitoring Prototype

Palomar Dome Node is an ESP32-based embedded monitoring prototype built for an amateur astronomy setup.

The project integrates local atmospheric sensing, ambient light measurement, mathematical estimation of observing conditions, an onboard OLED display, and a local Wi-Fi dashboard.

The system was constructed as a functional engineering prototype on a development testbed to explore sensor fusion, I²C bus integration, non-blocking task scheduling, and adaptive optical gain control.

---

## System Overview

The node uses an **ESP32 NodeMCU** development board as the central microcontroller.

The hardware acquires data from:
- **BME280** — Measures ambient temperature, relative humidity, and barometric pressure.
- **TSL2591** — Measures ambient light across visible and near-infrared spectra.
- **SSD1306 OLED (0.96")** — Displays real-time station metrics locally.

The ESP32 processes measurements locally, computes environmental and derived astronomical estimates, renders the output to the OLED screen, and hosts a local web dashboard. 

External weather metrics (cloud cover, precipitation probability) are retrieved periodically from the **Open-Meteo REST API**.

---

## Hardware Architecture

| Component | Interface | Role |
|---|---|---|
| ESP32 NodeMCU (30-pin) | Master / Wi-Fi | Core microcontroller, edge logic, local HTTP server |
| BME280 | I²C (`0x76`) | Atmospheric sensing (Temperature, Humidity, Pressure) |
| TSL2591 | I²C (`0x29`) | Ambient optical sensing (Dual photodiode) |
| SSD1306 0.96" OLED | I²C (`0x3C`) | Local telemetry display |
| 3D-Printed Model | Mechanical | Scale replica of the Palomar dome for sensor mounting |

### I²C Connections

All peripherals share a single 3.3V I²C bus:

| ESP32 Pin | Connection |
|---|---|
| 3V3 | VCC of BME280, TSL2591, SSD1306 |
| GND | GND of BME280, TSL2591, SSD1306 |
| GPIO 22 (SCL) | SCL lines of all three modules |
| GPIO 21 (SDA) | SDA lines of all three modules |

---

## Firmware Features

### 1. Environmental Sensing & Dew Point Tracking
The BME280 reads ambient temperature ($T$), relative humidity ($RH$), and barometric pressure ($P$). 

The firmware calculates the dew point locally using the **Magnus-Tetens empirical approximation**:
- $\alpha(T, RH) = \frac{17.27 \cdot T}{237.7 + T} + \ln\left(\frac{RH}{100}\right)$
- $T_{\text{dew}} = \frac{237.7 \cdot \alpha(T, RH)}{17.27 - \alpha(T, RH)}$

If the margin between ambient temperature and dew point narrows ($\Delta T \le 3^\circ\text{C}$), a `DEW ALERT` flag is raised to warn of potential condensation on telescope lenses or primary mirrors.

### 2. Low-Light Estimation & Derived Metrics
Using the dual-channel photodiode data from the TSL2591, the system isolates visible photon counts ($\text{Visible} = \text{Full} - \text{IR}$) and computes:
- **Estimated MPSAS** ($\text{mag/arcsec}^2$): Logarithmic conversion from lux readings.
- **Estimated NELM** (Naked-Eye Limiting Magnitude): Theoretical naked-eye detection limit.
- **Estimated Bortle Class**: Categorized from Class 1 (Pristine) to Class 9 (Inner-City).

*Notice: These astronomical figures are software-derived approximations based on broad ambient light levels, not calibrated laboratory-grade Sky Quality Meter (SQM) measurements.*

### 3. Non-Blocking Cooperative Scheduling
The firmware operates entirely without blocking `delay()` functions in the main loop. Time-based tasks are managed using `millis()` intervals:
- Sensor polling & adaptive ranging: **1.5 s**
- OLED frame refresh: **1.5 s** (synchronized after sensor updates)
- Satellite forecast synchronization: **10 minutes**
- Web client processing (`server.handleClient()`): **Immediate execution per loop cycle**

### 4. Telemetry Interfaces
- **Local:** The 0.96" OLED provides a quick-glance status overview (Climate, estimated Bortle class, and operating status).
- **Remote:** A lightweight, dark-themed dashboard hosted from ESP32 Flash memory (`PROGMEM`), retrieving dynamic data via an asynchronous JSON endpoint (`/data`).

---

## Engineering Challenges & Solutions

### 1. Photodiode Saturation & Gain Lockup
When transitioning from a dark environment to a lit room, the TSL2591 photodiode registers clipped at their 16-bit ceiling (`65535`), generating a combined overflow word (`0xFFFFFFFF`).

In early firmware revisions, an `if (lum != 0xFFFFFFFF)` guard condition discarded the packet completely. Because the auto-gain logic was placed inside that block, the sensor locked permanently in `GAIN_MAX` (9876x, 600 ms), freezing the UI.

**Solution:**
A dedicated overflow handler was implemented. If `lum == 0xFFFFFFFF` is detected, the firmware immediately bypasses normal calculation, overrides the state to `DAYTIME`, and shifts the gain configuration down to lower levels (`GAIN_LOW` or `GAIN_MED`) within a single loop cycle.

### 2. Low-Light Underflow & Mathematical Errors
In very dark environments, standard library conversion functions often truncated tiny fractional values to `0.000 lux`. Passing `0` into the logarithmic equation $\ln(\text{Lux})$ generated negative infinity ($-\infty$) and corrupted the OLED display with `NaN` artifacts.

**Solution:**
- An **Auto-Gain Finite State Machine (FSM)** increases sensor sensitivity up to `GAIN_MAX` (9876x) and integration time to `600 ms` when raw counts drop below 150.
- When calculated values fall below $0.0001\,\text{lux}$ while raw photon counts remain non-zero, the software calculates an uncalibrated low-light floor estimate using the hardware Counts-Per-Lux (CPL) ratio.
- Numerical clamp guards prevent negative or zero inputs into logarithmic functions.

---

## Functional Verification

The prototype was validated through iterative bench testing:

| Test Item | Verification Method | Result |
|---|---|---|
| I²C Shared Bus | Bus address scanner (`0x29`, `0x3C`, `0x76`) | All three devices acknowledged concurrently |
| Dark-to-Light Transition | Exposing high-gain sensor to room lighting | Recovered from `0xFFFFFFFF` overflow without freezing |
| Light-to-Dark Transition | Enclosing sensor in a darkened environment | Escalated gain to maximum without UI flicker |
| Display Output | Inspecting OLED output across edge cases | Eliminated `NaN` and rapid zero-jumping artifacts |
| Web Dashboard | Polling `/data` endpoint via local network | Continuous JSON delivery at 2-second intervals |
| Weather Fetch | HTTP GET requests to Open-Meteo REST API | Successfully parsed cloud cover and rain probability |

---

## Limitations

- **Uncalibrated Optical Path:** The TSL2591 lacks an optical bandpass filter (e.g., Johnson-Cousins V filter) and has an unbaffled field of view ($\approx 180^\circ$), making it sensitive to wide-angle stray light.
- **Thermal Noise Floor:** At room temperature without thermoelectric cooling, photodiode thermal leakage introduces noise counts in near-total darkness. Readings at this level serve as software estimates rather than absolute photometric data.
- **Hardware Maturity:** The project is currently assembled as a functional engineering prototype on a breadboard development platform rather than an enclosed custom PCB.

---

## Future Roadmap

- Design a custom 2-layer PCB to replace the breadboard wiring.
- Design an optical baffle tube to restrict the sensor field of view to $\approx 20^\circ$ (matching standard SQM instruments).
- Implement hardware-driven dew heater relay control based on the calculated dew margin.
- Add local data logging via a MicroSD card breakout or EEPROM.

---

## Project Structure

```text
Palomar-Dome-Node/
├── README.md
├── LICENSE
├── firmware/
│   └── PalomarDomeNode.ino
└── media/
    ├── prototype-front.jpg
    ├── breadboard-wiring.jpg
    └── dark-chamber-test.jpg
```

---

## License

This project is released under the [MIT License](LICENSE).