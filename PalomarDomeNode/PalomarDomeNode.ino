/*
 * Palomar Dome Node v1.0
 * Embedded Sky Quality & Atmospheric Monitoring System
 *
 * Hardware:
 *   - ESP32 NodeMCU (30-pin)
 *   - BME280
 *   - TSL2591
 *   - 0.96" SSD1306 OLED (128x64)
 *
 * Core functions:
 *   - Local atmospheric sensing
 *   - Adaptive TSL2591 gain / integration-time control
 *   - Dew-point and sky-brightness calculations at the edge
 *   - Local OLED interface
 *   - Local web dashboard + JSON endpoint
 *   - Optional Open-Meteo weather-data integration
 *
 * v1.0 release notes:
 *   - Removed hard-coded Wi-Fi credentials.
 *   - Added explicit sensor / weather availability state.
 *   - Added non-blocking Wi-Fi reconnect attempts.
 *   - Added HTTP timeout for external API requests.
 *   - Corrected low-light fallback integration-time scaling.
 *   - Renamed/clarified derived values as estimates where appropriate.
 *   - Uses send_P() for the HTML page stored in PROGMEM.
 *
 * IMPORTANT:
 *   Replace the Wi-Fi placeholders below before uploading to the ESP32.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2591.h>
#include <math.h>

// ===================== CONFIGURATION =====================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Observation-site coordinates used by the external weather API.
const float LATITUDE  = 21.0285F;
const float LONGITUDE = 105.8542F;

constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint8_t BME_ADDR_PRIMARY = 0x76;
constexpr uint8_t BME_ADDR_SECONDARY = 0x77;
constexpr uint8_t TSL_ADDR = 0x29;

constexpr uint32_t SENSOR_INTERVAL_MS = 1500UL;
constexpr uint32_t API_INTERVAL_MS = 600000UL;       // 10 minutes
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000UL;
constexpr uint16_t HTTP_TIMEOUT_MS = 5000;

constexpr float DEW_MARGIN_ALERT_C = 3.0F;
constexpr float DAYLIGHT_LUX_THRESHOLD = 1.0F;
constexpr float AUTO_GAIN_LOW_THRESHOLD = 150.0F;
constexpr uint16_t AUTO_GAIN_HIGH_THRESHOLD = 60000;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BME280 bme;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
WebServer server(80);

// ===================== SENSOR STATE =====================
bool oledAvailable = false;
bool bmeAvailable = false;
bool tslAvailable = false;
bool weatherAvailable = false;

uint8_t bmeAddress = 0;

// Current TSL2591 configuration.
tsl2591Gain_t currentGain = TSL2591_GAIN_MED;
tsl2591IntegrationTime_t currentTime = TSL2591_INTEGRATIONTIME_300MS;

// Local environmental measurements.
float localTemp = NAN;
float localHumidity = NAN;
float localPressure = NAN;
float localDewPoint = NAN;
float dewMargin = NAN;
bool dewRisk = false;

// Light measurements and derived astronomy metrics.
float luxVal = NAN;
float mpsasVal = NAN;
float nelmVal = NAN;
uint16_t visVal = 0;
uint16_t irVal = 0;
int bortleClass = 0;
String skyRating = "NO LIGHT DATA";

// External weather data.
int cloudCover = 0;
int precipitation = 0;

String obsStatus = "INITIALIZING";

// ===================== TIMING STATE =====================
uint32_t lastSensorRead = 0;
uint32_t lastApiFetch = 0;
uint32_t lastWiFiAttempt = 0;
uint32_t lastWeatherUpdate = 0;

// ===================== UTILITY FUNCTIONS =====================

float calculateDewPoint(float temp, float hum) {
  if (!isfinite(temp) || !isfinite(hum) || hum <= 0.0F) {
    return NAN;
  }

  // Magnus-Tetens approximation.
  constexpr float A = 17.27F;
  constexpr float B = 237.7F;
  const float alpha = ((A * temp) / (B + temp)) + log(hum / 100.0F);
  return (B * alpha) / (A - alpha);
}

void calculateSkyQuality(float lux) {
  if (!isfinite(lux) || lux < 0.0F) {
    mpsasVal = NAN;
    nelmVal = NAN;
    bortleClass = 0;
    skyRating = "NO VALID LIGHT DATA";
    return;
  }

  if (lux >= DAYLIGHT_LUX_THRESHOLD) {
    mpsasVal = NAN;
    nelmVal = NAN;
    bortleClass = 0;
    skyRating = "Daylight / Bright Environment";
    return;
  }

  // Convert illuminance to an estimated sky brightness in mag/arcsec^2.
  // This is a derived estimate, not a calibrated professional SQM reading.
  const float safeLux = (lux < 0.00001F) ? 0.00001F : lux;
  mpsasVal = 12.6F - (1.0857F * log(safeLux));
  if (mpsasVal > 22.0F) mpsasVal = 22.0F;

  // Empirical naked-eye limiting magnitude estimate.
  const float exponent = 4.316F - (0.2F * mpsasVal);
  nelmVal = 7.93F - (5.0F * log10(pow(10.0F, exponent) + 1.0F));
  if (!isfinite(nelmVal) || nelmVal < 0.0F) nelmVal = 0.0F;

  // Approximate Bortle classification from estimated MPSAS.
  if (mpsasVal >= 21.75F) {
    bortleClass = 1;
    skyRating = "Estimated Class 1: Pristine Dark";
  } else if (mpsasVal >= 21.50F) {
    bortleClass = 2;
    skyRating = "Estimated Class 2: Truly Dark Sky";
  } else if (mpsasVal >= 21.30F) {
    bortleClass = 3;
    skyRating = "Estimated Class 3: Rural Sky";
  } else if (mpsasVal >= 20.40F) {
    bortleClass = 4;
    skyRating = "Estimated Class 4: Rural/Suburban";
  } else if (mpsasVal >= 19.50F) {
    bortleClass = 5;
    skyRating = "Estimated Class 5: Suburban Sky";
  } else if (mpsasVal >= 18.50F) {
    bortleClass = 6;
    skyRating = "Estimated Class 6: Bright Suburban";
  } else if (mpsasVal >= 18.00F) {
    bortleClass = 7;
    skyRating = "Estimated Class 7: Suburban/Urban";
  } else if (mpsasVal >= 17.00F) {
    bortleClass = 8;
    skyRating = "Estimated Class 8: City Sky";
  } else {
    bortleClass = 9;
    skyRating = "Estimated Class 9: Inner-City";
  }
}

float integrationTimeMs(tsl2591IntegrationTime_t integration) {
  switch (integration) {
    case TSL2591_INTEGRATIONTIME_100MS: return 100.0F;
    case TSL2591_INTEGRATIONTIME_200MS: return 200.0F;
    case TSL2591_INTEGRATIONTIME_300MS: return 300.0F;
    case TSL2591_INTEGRATIONTIME_400MS: return 400.0F;
    case TSL2591_INTEGRATIONTIME_500MS: return 500.0F;
    case TSL2591_INTEGRATIONTIME_600MS: return 600.0F;
    default: return 100.0F;
  }
}

float gainFactor(tsl2591Gain_t gain) {
  switch (gain) {
    case TSL2591_GAIN_LOW: return 1.0F;
    case TSL2591_GAIN_MED: return 25.0F;
    case TSL2591_GAIN_HIGH: return 428.0F;
    case TSL2591_GAIN_MAX: return 9876.0F;
    default: return 1.0F;
  }
}

float lowLightLuxEstimate(uint16_t visibleCounts) {
  // CPL follows the same basic scaling used by the Adafruit TSL2591 library:
  // CPL = (integration_time_ms * gain) / 408.
  // This fallback uses visible-channel counts when the library's lux result
  // becomes too small to be useful. It remains an estimate and is not a
  // calibrated photometric measurement.
  const float cpl = (integrationTimeMs(currentTime) * gainFactor(currentGain)) / 408.0F;
  if (cpl <= 0.0F) return NAN;

  float estimate = static_cast<float>(visibleCounts) / cpl;
  if (estimate < 0.00001F) estimate = 0.00001F;
  return estimate;
}

void applyLightConfiguration() {
  if (!tslAvailable) return;
  tsl.setGain(currentGain);
  tsl.setTiming(currentTime);
}

void setLightConfiguration(tsl2591Gain_t gain, tsl2591IntegrationTime_t integration) {
  currentGain = gain;
  currentTime = integration;
  applyLightConfiguration();
}

void handleAutoGain(uint16_t full, bool isOverflow) {
  if (!tslAvailable) return;

  // Bright / saturated condition: reduce sensitivity quickly.
  if (isOverflow || full > AUTO_GAIN_HIGH_THRESHOLD) {
    if (currentGain == TSL2591_GAIN_MAX) {
      setLightConfiguration(TSL2591_GAIN_MED, TSL2591_INTEGRATIONTIME_200MS);
    } else if (currentGain == TSL2591_GAIN_HIGH) {
      setLightConfiguration(TSL2591_GAIN_LOW, TSL2591_INTEGRATIONTIME_100MS);
    } else if (currentGain == TSL2591_GAIN_MED) {
      setLightConfiguration(TSL2591_GAIN_LOW, TSL2591_INTEGRATIONTIME_100MS);
    }
    return;
  }

  // Very dark condition: increase sensitivity one step at a time.
  if (full < AUTO_GAIN_LOW_THRESHOLD) {
    if (currentGain == TSL2591_GAIN_LOW) {
      setLightConfiguration(TSL2591_GAIN_MED, TSL2591_INTEGRATIONTIME_300MS);
    } else if (currentGain == TSL2591_GAIN_MED) {
      setLightConfiguration(TSL2591_GAIN_HIGH, TSL2591_INTEGRATIONTIME_400MS);
    } else if (currentGain == TSL2591_GAIN_HIGH) {
      setLightConfiguration(TSL2591_GAIN_MAX, TSL2591_INTEGRATIONTIME_600MS);
    }
  }
}

void updateObservationStatus() {
  if (!tslAvailable || !isfinite(luxVal)) {
    obsStatus = "LIGHT SENSOR ERROR";
  } else if (precipitation > 30 && weatherAvailable) {
    obsStatus = "RAIN RISK";
  } else if (dewRisk) {
    obsStatus = "DEW ALERT";
  } else if (luxVal >= DAYLIGHT_LUX_THRESHOLD) {
    obsStatus = "DAYTIME";
  } else if (weatherAvailable && cloudCover > 60) {
    obsStatus = "OVERCAST";
  } else if (bortleClass > 0 && bortleClass <= 4) {
    obsStatus = "PRIME SKY";
  } else {
    obsStatus = "PLANETARY";
  }
}

// ===================== SENSOR ACQUISITION =====================

void readSensors() {
  // --- BME280 ---
  if (bmeAvailable) {
    const float t = bme.readTemperature();
    const float h = bme.readHumidity();
    const float p = bme.readPressure() / 100.0F;

    if (isfinite(t) && isfinite(h) && h > 0.0F && h <= 100.0F && isfinite(p)) {
      localTemp = t;
      localHumidity = h;
      localPressure = p;
      localDewPoint = calculateDewPoint(localTemp, localHumidity);
      dewMargin = localTemp - localDewPoint;
      dewRisk = isfinite(dewMargin) && (dewMargin <= DEW_MARGIN_ALERT_C);
    }
  }

  // --- TSL2591 ---
  if (tslAvailable) {
    const uint32_t lum = tsl.getFullLuminosity();

    if (lum == 0xFFFFFFFFUL) {
      handleAutoGain(65535, true);
      luxVal = 500.0F;
      calculateSkyQuality(luxVal);
    } else if (lum != 0UL) {
      const uint16_t currentIr = static_cast<uint16_t>(lum >> 16);
      const uint16_t currentFull = static_cast<uint16_t>(lum & 0xFFFFUL);

      if (currentFull == 0xFFFFU || currentIr == 0xFFFFU) {
        handleAutoGain(65535, true);
        luxVal = 500.0F;
        calculateSkyQuality(luxVal);
      } else if (currentFull >= currentIr) {
        irVal = currentIr;
        visVal = static_cast<uint16_t>(currentFull - currentIr);

        float calculatedLux = tsl.calculateLux(currentFull, currentIr);

        // Low-light fallback. The library's normal lux calculation is retained
        // as the primary result; this fallback only activates near its floor.
        if ((calculatedLux <= 0.0001F || !isfinite(calculatedLux)) && currentFull > 0U) {
          calculatedLux = lowLightLuxEstimate(visVal);
        }

        if (isfinite(calculatedLux) && calculatedLux >= 0.0F) {
          luxVal = calculatedLux;
          calculateSkyQuality(luxVal);
        }

        // Apply a new gain/timing configuration for the next acquisition cycle.
        handleAutoGain(currentFull, false);
      }
    }
  }

  updateObservationStatus();
}

// ===================== NETWORK / API =====================

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttempt = millis();
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  const uint32_t now = millis();
  if (now - lastWiFiAttempt >= WIFI_RETRY_INTERVAL_MS) {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWiFiAttempt = now;
  }
}

void fetchApiWeather() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  const String url = String("http://api.open-meteo.com/v1/forecast?latitude=") + LATITUDE +
                     "&longitude=" + LONGITUDE +
                     "&current=cloud_cover,precipitation_probability";

  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(url)) return;

  const int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument doc(1024);
    const DeserializationError error = deserializeJson(doc, http.getString());

    if (!error && doc["current"].is<JsonObject>()) {
      cloudCover = doc["current"]["cloud_cover"] | 0;
      precipitation = doc["current"]["precipitation_probability"] | 0;
      weatherAvailable = true;
      lastWeatherUpdate = millis();
    }
  }

  http.end();
  updateObservationStatus();
}

// ===================== OLED =====================

void updateOLED() {
  if (!oledAvailable) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("PALOMAR DOME NODE ");
  display.print(WiFi.status() == WL_CONNECTED ? "[ON]" : "[NC]");

  display.setCursor(0, 13);
  if (isfinite(localTemp) && isfinite(localHumidity) && isfinite(localPressure)) {
    display.printf("T:%.1fC H:%.0f%% P:%.0f", localTemp, localHumidity, localPressure);
  } else {
    display.print("BME280: NO DATA");
  }

  display.setCursor(0, 26);
  if (isfinite(luxVal) && luxVal < DAYLIGHT_LUX_THRESHOLD && bortleClass > 0) {
    display.printf("MPSAS:%.2f BTL:%d", mpsasVal, bortleClass);
  } else if (isfinite(luxVal)) {
    display.printf("Lux:%.2f [DAYLIGHT]", luxVal);
  } else {
    display.print("TSL2591: NO DATA");
  }

  display.setCursor(0, 39);
  if (isfinite(nelmVal) && bortleClass > 0) {
    display.printf("NELM:%.1fm Cld:%d%%", nelmVal, cloudCover);
  } else if (isfinite(localDewPoint)) {
    display.printf("Dew:%.1fC Cld:%d%%", localDewPoint, cloudCover);
  } else {
    display.print("Weather: NO DATA");
  }

  display.setCursor(0, 52);
  display.printf("STS: %s", obsStatus.c_str());
  display.display();
}

// ===================== WEB DASHBOARD =====================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Palomar Dome Node</title>
  <style>
    :root { --bg:#090d16; --card:#121826; --cyan:#00f2fe; --accent:#4facfe; --text:#e2e8f0; --dim:#64748b; }
    body { font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; background:var(--bg); color:var(--text); margin:0; padding:20px; display:flex; flex-direction:column; align-items:center; }
    .container { width:100%; max-width:800px; }
    .header { display:flex; justify-content:space-between; align-items:center; margin-bottom:20px; border-bottom:1px solid #1e293b; padding-bottom:14px; gap:16px; }
    .header h1 { font-size:1.3rem; margin:0; color:var(--cyan); }
    .status-badge { padding:6px 14px; border-radius:20px; font-weight:700; font-size:.85rem; background:#1e293b; border:1px solid var(--cyan); color:var(--cyan); white-space:nowrap; }
    .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:16px; }
    .card { background:var(--card); border-radius:12px; padding:16px; border:1px solid #1e293b; }
    .card-title { font-size:.75rem; color:var(--dim); text-transform:uppercase; margin-bottom:8px; }
    .card-value { font-size:1.6rem; font-weight:700; color:#fff; }
    .unit { font-size:.85rem; color:var(--accent); margin-left:4px; }
    .desc { font-size:.8rem; color:var(--dim); margin-top:6px; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <div>
        <h1>PALOMAR DOME NODE</h1>
        <small style="color:var(--dim)">Embedded Sky Quality &amp; Atmospheric Monitoring System</small>
      </div>
      <div class="status-badge" id="status">CONNECTING</div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="card-title">Estimated Sky Brightness</div>
        <div class="card-value"><span id="mpsas">--</span><span class="unit">mag/arcsec²</span></div>
        <div class="desc" id="bortle_text">Estimated Bortle Class: --</div>
      </div>
      <div class="card">
        <div class="card-title">Estimated Limiting Magnitude</div>
        <div class="card-value"><span id="nelm">--</span><span class="unit">mag</span></div>
        <div class="desc">Naked-Eye Limiting Magnitude (NELM)</div>
      </div>
      <div class="card">
        <div class="card-title">Ambient Light (TSL2591)</div>
        <div class="card-value"><span id="lux">--</span><span class="unit">Lux</span></div>
        <div class="desc">IR: <span id="ir">--</span> | Vis: <span id="vis">--</span></div>
      </div>
      <div class="card">
        <div class="card-title">Barometric Pressure</div>
        <div class="card-value"><span id="press">--</span><span class="unit">hPa</span></div>
        <div class="desc">BME280 Station Pressure</div>
      </div>
      <div class="card">
        <div class="card-title">Temperature &amp; Dew Point</div>
        <div class="card-value"><span id="temp">--</span><span class="unit">°C</span></div>
        <div class="desc">Dew: <span id="dew">--</span>°C (Margin: <span id="margin">--</span>°C)</div>
      </div>
      <div class="card">
        <div class="card-title">Humidity &amp; External Weather</div>
        <div class="card-value"><span id="hum">--</span><span class="unit">%</span></div>
        <div class="desc">Clouds: <span id="cloud">--</span>% | Rain: <span id="rain">--</span>%</div>
      </div>
    </div>
  </div>

  <script>
    function updateData() {
      fetch('/data')
        .then(res => res.json())
        .then(d => {
          document.getElementById('temp').innerText = Number.isFinite(d.temp) ? d.temp.toFixed(1) : '--';
          document.getElementById('hum').innerText = Number.isFinite(d.hum) ? d.hum.toFixed(0) : '--';
          document.getElementById('press').innerText = Number.isFinite(d.press) ? d.press.toFixed(0) : '--';
          document.getElementById('dew').innerText = Number.isFinite(d.dew) ? d.dew.toFixed(1) : '--';
          document.getElementById('margin').innerText = Number.isFinite(d.dew_margin) ? d.dew_margin.toFixed(1) : '--';

          if (Number.isFinite(d.lux)) {
            document.getElementById('lux').innerText = d.lux < 0.01 && d.lux > 0 ? d.lux.toFixed(5) : d.lux.toFixed(2);
          } else {
            document.getElementById('lux').innerText = '--';
          }

          document.getElementById('vis').innerText = d.vis;
          document.getElementById('ir').innerText = d.ir;
          document.getElementById('cloud').innerText = d.weather_available ? d.cloud : '--';
          document.getElementById('rain').innerText = d.weather_available ? d.rain_prob : '--';

          if (d.bortle > 0 && Number.isFinite(d.mpsas)) {
            document.getElementById('mpsas').innerText = d.mpsas.toFixed(2);
            document.getElementById('nelm').innerText = Number.isFinite(d.nelm) ? d.nelm.toFixed(1) : '--';
            document.getElementById('bortle_text').innerText = d.sky_desc;
          } else {
            document.getElementById('mpsas').innerText = 'DAYLIGHT';
            document.getElementById('nelm').innerText = 'N/A';
            document.getElementById('bortle_text').innerText = 'Not applicable in bright conditions';
          }

          const st = document.getElementById('status');
          st.innerText = d.status;
          if (d.dew_risk || (d.weather_available && d.rain_prob > 30)) {
            st.style.color = '#ff3366';
            st.style.borderColor = '#ff3366';
          } else {
            st.style.color = '#00f2fe';
            st.style.borderColor = '#00f2fe';
          }
        })
        .catch(() => {
          const st = document.getElementById('status');
          st.innerText = 'NODE OFFLINE';
        });
    }

    setInterval(updateData, 2000);
    updateData();
  </script>
</body>
</html>
)rawliteral";

void handleDataJson() {
  DynamicJsonDocument doc(768);

  doc["temp"] = localTemp;
  doc["hum"] = localHumidity;
  doc["press"] = localPressure;
  doc["dew"] = localDewPoint;
  doc["dew_margin"] = dewMargin;
  doc["dew_risk"] = dewRisk;

  doc["lux"] = luxVal;
  doc["vis"] = visVal;
  doc["ir"] = irVal;
  doc["mpsas"] = mpsasVal;
  doc["nelm"] = nelmVal;
  doc["bortle"] = bortleClass;
  doc["sky_desc"] = skyRating;

  doc["cloud"] = cloudCover;
  doc["rain_prob"] = precipitation;
  doc["weather_available"] = weatherAvailable;
  doc["status"] = obsStatus;

  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // --- OLED ---
  oledAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledAvailable) {
    Serial.println("[ERROR] OLED initialization failed.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("PALOMAR DOME NODE");
    display.println("Starting Node...");
    display.display();
  }

  // --- BME280 ---
  bmeAvailable = bme.begin(BME_ADDR_PRIMARY, &Wire);
  if (bmeAvailable) {
    bmeAddress = BME_ADDR_PRIMARY;
  } else {
    bmeAvailable = bme.begin(BME_ADDR_SECONDARY, &Wire);
    if (bmeAvailable) bmeAddress = BME_ADDR_SECONDARY;
  }

  if (!bmeAvailable) {
    Serial.println("[ERROR] BME280 not found at 0x76 or 0x77.");
  } else {
    Serial.printf("[OK] BME280 found at 0x%02X.\n", bmeAddress);
  }

  // --- TSL2591 ---
  tslAvailable = tsl.begin(&Wire, TSL_ADDR);
  if (!tslAvailable) {
    Serial.println("[ERROR] TSL2591 initialization failed.");
  } else {
    applyLightConfiguration();
    Serial.println("[OK] TSL2591 initialized.");
  }

  // --- Wi-Fi ---
  startWiFi();

  // Give the first connection a short startup window. Normal reconnects are
  // handled asynchronously by maintainWiFi().
  const uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000UL) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[OK] Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WARN] Wi-Fi not connected. Node will continue locally.");
  }

  // --- Web server ---
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.on("/data", HTTP_GET, handleDataJson);
  server.begin();
  Serial.println("[OK] Web server started.");

  // Initial data acquisition.
  readSensors();
  updateOLED();

  // Initial weather request only if Wi-Fi is already available.
  if (WiFi.status() == WL_CONNECTED) {
    fetchApiWeather();
  }

  // Start intervals relative to completed setup work.
  const uint32_t now = millis();
  lastSensorRead = now;
  lastApiFetch = now;
}

// ===================== MAIN LOOP =====================

void loop() {
  server.handleClient();
  maintainWiFi();

  const uint32_t now = millis();

  // Sensor acquisition / OLED refresh.
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSensors();
    updateOLED();
  }

  // External weather data refresh.
  if (now - lastApiFetch >= API_INTERVAL_MS) {
    lastApiFetch = now;
    fetchApiWeather();
    updateOLED();
  }
}
