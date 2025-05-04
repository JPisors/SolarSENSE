#include <p0tted_plant_inferencing.h>
#include <algorithm>
#include <iostream>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Button2.h>
#include <BH1750.h>
#include <cJSON.h>
#include <OneWire.h>
#include <LoRa.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_NeoPixel.h>
#include "DHT.h"
#include "configuration.h" // Ensure pin defs, WiFi creds are here. Remove #define BOOT_PIN (0) if present.

#define TIME_TO_SLEEP 60 * 60 * 1000
#define SEALEVELPRESSURE_HPA (1013.25)

typedef enum {
  BME280_SENSOR_ID,
  DHTxx_SENSOR_ID,
  SHT3x_SENSOR_ID,
  BHT1750_SENSOR_ID,
  SOIL_SENSOR_ID,
  SALT_SENSOR_ID,
  DS18B20_SENSOR_ID,
  VOLTAGE_SENSOR_ID,
} sensor_id_t;

typedef struct {
  uint32_t timestamp;
  float temperature;
  float light;
  float pressure;
  float humidity;
  float altitude;
  float voltage;
  uint8_t soli;
  uint8_t salt;
  float primary_temp;
  float primary_hum;
} higrow_sensors_event_t;

AsyncWebServer server(80);
ESPDash dashboard(&server);

Button2 button(BOOT_PIN); // BOOT_PIN is usually GPIO0
Button2 useButton(USER_BUTTON); // Ensure USER_BUTTON is defined in configuration.h

// Sensor Objects
BH1750 lightMeter(OB_BH1750_ADDRESS); // Address defined in configuration.h
DHT dht(DHT1x_PIN, DHTTYPE); // Ensure DHT1x_PIN and DHTTYPE are defined in configuration.h
OneWire ds(DS18B20_PIN); // Ensure DS18B20_PIN is defined in configuration.h
Adafruit_SHT31 sht31 = Adafruit_SHT31(&Wire1); // Assumes Wire1 is used for SHT31
Adafruit_BME280 bme; // Uses default Wire object

Adafruit_NeoPixel *pixels = NULL;

// Sensor Availability Flags
bool has_lora_shield = false;
bool has_bmeSensor = false;
bool has_lightSensor = false;
bool has_dht11 = false;
bool has_sht3xSensor = false;
bool has_ds18b20 = false;

uint64_t timestamp = 0;
uint8_t ds18b20Addr[8];
uint8_t ds18b20Type;

// ESPDash Card Pointers
Card *lightCard = NULL;
Card *soilValue = NULL;
Card *saltValue = NULL;
Card *batteryValue = NULL;
Card *bmeAltitudeCard = NULL;
Card *bmePressureCard = NULL;
Card *dhtTemperature = NULL;
Card *dhtHumidity = NULL;
Card *bmeTemperature = NULL;
Card *bmeHumidity = NULL;
Card *sht3xTemperature = NULL;
Card *sht3xHumidity = NULL;
Card *dsTemperature = NULL;
Card *motorButton = NULL;

// Edge Impulse Dashboard Cards
Card *mlClassificationCard = NULL;
Card *mlConfidenceCard = NULL;
Card *mlAnomalyCard = NULL;

// Edge Impulse Global Variables
static bool debug_nn = false; // Set true for more EI debug logging
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; // Needs EI include to work
size_t feature_ix = 0;

// --- Function Prototypes ---
void setupLoRa();
void loopLoRa(higrow_sensors_event_t *val, ei_impulse_result_t *result); // Needs EI include
float getDsTemperature(void);
void smartConfigStart(Button2 &b);
void deviceSleep();
void sleepHandler(Button2 &b);
bool dhtSensorProbe();
void wifi_ap_connect_timeout(WiFiEvent_t event, WiFiEventInfo_t info);
void setupWiFi();
bool get_higrow_sensors_event(sensor_id_t id, higrow_sensors_event_t &val);
bool ds18b20Begin();
#ifdef AUTO_WATER
void WateringCallback(bool value);
#endif

// --- Function Implementations ---

void smartConfigStart(Button2 &b) {
    Serial.println("ESP-Touch started. Use ESP-Touch app to configure WiFi.");
    WiFi.disconnect();
    WiFi.beginSmartConfig();
    while (!WiFi.smartConfigDone()) {
        Serial.print(".");
        delay(200);
        // Add timeout?
    }
    WiFi.stopSmartConfig();
    Serial.println("\nSmartConfig complete. Connected to: " + WiFi.SSID());
}

void deviceSleep() {
    if (has_lora_shield) {
        LoRa.sleep();
        SPI.end();
    }
    Serial.println("Entering deep sleep...");
    // *** FIX: Use GPIO_NUM_35 ***
    esp_sleep_enable_ext1_wakeup(GPIO_NUM_35 /* Change to your desired wakeup pin mask */, ESP_EXT1_WAKEUP_ALL_LOW);
    delay(100); // Short delay before sleeping
    esp_deep_sleep_start();
}


void sleepHandler(Button2 &b) { deviceSleep(); }

TimerHandle_t sleepTimer = NULL; // Initialize timer handle

void wifi_ap_connect_timeout(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("WiFi client connected to AP!");
    if (sleepTimer != NULL) {
        xTimerDelete(sleepTimer, portMAX_DELAY);
        sleepTimer = NULL;
    }
}

void setupWiFi() {
#ifdef SOFTAP_MODE
    Serial.println("Configuring Access Point...");
    uint8_t mac[6];
    char apName[64];
    WiFi.macAddress(mac);
    snprintf(apName, sizeof(apName), "T-Higrow-EI-%02X%02X", mac[4], mac[5]);
    WiFi.softAP(apName);
    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("AP '%s' started. Open http://%s\n", apName, myIP.toString().c_str());

    WiFi.onEvent(wifi_ap_connect_timeout, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

    sleepTimer = xTimerCreate("ap_timeout", pdMS_TO_TICKS(TIME_TO_SLEEP), pdFALSE, NULL, [](TimerHandle_t t) {
        Serial.println("AP Timeout: Entering deep sleep.");
        WiFi.mode(WIFI_MODE_NULL);
        deviceSleep();
    });
    if (sleepTimer) xTimerStart(sleepTimer, portMAX_DELAY); else Serial.println("ERROR: Failed to create sleep timer!");
#else
    WiFi.mode(WIFI_STA);
    Serial.print("Connecting to WiFi: " + String(WIFI_SSID));
    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    uint8_t wifi_tries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_tries < 20) { // Timeout after ~10 seconds
        Serial.print(".");
        delay(500);
        wifi_tries++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed! Restarting...");
        delay(3000);
        ESP.restart();
    }
    Serial.println("\nWiFi connected! IP Address: " + WiFi.localIP().toString());
#endif
    server.begin();
}

bool get_higrow_sensors_event(sensor_id_t id, higrow_sensors_event_t &val) {
    float temp_reading = NAN;
    float hum_reading = NAN;

    // Initialize specific values only if they are not set by other calls
    if (id == BHT1750_SENSOR_ID) val.light = NAN;
    if (id == BME280_SENSOR_ID) { val.pressure = NAN; val.altitude = NAN; }
    if (id == VOLTAGE_SENSOR_ID) val.voltage = NAN;
    if (id == SOIL_SENSOR_ID) val.soli = 0;
    if (id == SALT_SENSOR_ID) val.salt = 0;

    switch (id) {
        case BME280_SENSOR_ID:
            if (has_bmeSensor) {
                temp_reading = bme.readTemperature();
                val.pressure = bme.readPressure() / 100.0F;
                val.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
                hum_reading = bme.readHumidity();
                if (!isnan(temp_reading)) val.primary_temp = temp_reading;
                if (!isnan(hum_reading)) val.primary_hum = hum_reading;
            }
            break;
        case SHT3x_SENSOR_ID:
             if (has_sht3xSensor) {
                temp_reading = sht31.readTemperature();
                hum_reading = sht31.readHumidity();
                if (!isnan(temp_reading)) val.primary_temp = temp_reading;
                if (!isnan(hum_reading)) val.primary_hum = hum_reading;
            }
            break;
        case DHTxx_SENSOR_ID:
            if (has_dht11) {
                temp_reading = dht.readTemperature();
                hum_reading = dht.readHumidity();
                if (!isnan(temp_reading)) val.primary_temp = temp_reading;
                if (!isnan(hum_reading)) val.primary_hum = hum_reading;
            }
            break;
        case BHT1750_SENSOR_ID:
            if (has_lightSensor) val.light = lightMeter.readLightLevel();
            break;
        case SOIL_SENSOR_ID: {
            uint16_t soil_raw = analogRead(SOIL_PIN);
            val.soli = map(soil_raw, 0, 4095, 100, 0); // Adjust map range if needed
        } break;
        case SALT_SENSOR_ID: {
            uint8_t samples = 120; // Reduced samples for faster reading
            uint32_t sum = 0;
            uint16_t readings[samples];
            for (int i = 0; i < samples; i++) {
                readings[i] = analogRead(SALT_PIN);
                delay(1); // Shorter delay
            }
            std::sort(readings, readings + samples);
            uint8_t trim = 10; // Trim 5 lowest/highest
            for (int i = trim; i < samples - trim; i++) {
                sum += readings[i];
            }
            val.salt = sum / (samples - 20); // Average of trimmed values
        } break;
        case DS18B20_SENSOR_ID:
            if (has_ds18b20) {
                temp_reading = getDsTemperature();
                if (!isnan(temp_reading)) val.primary_temp = temp_reading;
            }
            break;
        case VOLTAGE_SENSOR_ID: {
            uint16_t volt = analogRead(BAT_ADC);
            // Calibrate this formula! Example for 3.3V ref, 12bit ADC, 100k/100k divider
            val.voltage = ((float)volt / 4095.0) * 3.3 * 2.0;
        } break;
        default: return false;
    }

    // Assign primary temp/hum to the main structure fields if they were updated
    if (!isnan(val.primary_temp)) val.temperature = val.primary_temp;
    if (!isnan(val.primary_hum)) val.humidity = val.primary_hum;
    return true;
}

bool ds18b20Begin() {
    ds.reset_search();
    if (!ds.search(ds18b20Addr)) { Serial.println("No DS18B20 found."); return false; }
    Serial.print("DS18B20 ROM =");
    for (int i = 0; i < 8; i++) { Serial.print(" "); Serial.print(ds18b20Addr[i], HEX); }
    Serial.println();
    if (OneWire::crc8(ds18b20Addr, 7) != ds18b20Addr[7]) { Serial.println("DS18B20 CRC invalid!"); return false; }
    switch (ds18b20Addr[0]) {
        case 0x10: Serial.println("  Chip = DS18S20"); ds18b20Type = 1; break;
        case 0x28: Serial.println("  Chip = DS18B20"); ds18b20Type = 0; break;
        case 0x22: Serial.println("  Chip = DS1822"); ds18b20Type = 0; break;
        default: Serial.println("Device is not DS18x20 family."); return false;
    }
    return true;
}

float getDsTemperature(void) {
    byte data[9];
    ds.reset();
    ds.select(ds18b20Addr);
    ds.write(0x44, 1); // Start conversion
    delay(750); // Wait (adjust based on resolution if changed)
    ds.reset();
    ds.select(ds18b20Addr);
    ds.write(0xBE); // Read Scratchpad
    for (int i = 0; i < 9; i++) { data[i] = ds.read(); }
    if (OneWire::crc8(data, 8) != data[8]) { Serial.println("DS18B20 Data CRC failed!"); return NAN; }
    int16_t raw = (data[1] << 8) | data[0];
    if (ds18b20Type) { /* DS18S20 calculation */ raw = raw << 3; if (data[7] == 0x10) raw = (raw & 0xFFF0) + 12 - data[6]; }
    else { /* DS18B20/DS1822 calculation */ byte cfg = (data[4] & 0x60); if (cfg == 0x00) raw &= ~7; else if (cfg == 0x20) raw &= ~3; else if (cfg == 0x40) raw &= ~1; }
    return (float)raw / 16.0;
}

bool dhtSensorProbe() {
    dht.begin();
    delay(1000); // Shorter stabilization delay
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    // Sometimes first read fails, try again
    if (isnan(h) || isnan(t)) {
        delay(1000);
        h = dht.readHumidity();
        t = dht.readTemperature();
    }
    return !isnan(h) && !isnan(t);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\nT-Higrow Sensor Monitor w/ Edge Impulse - Starting Setup...");

    button.setLongClickHandler(smartConfigStart);
    useButton.setLongClickHandler(sleepHandler);

    pinMode(POWER_CTRL, OUTPUT);
    digitalWrite(POWER_CTRL, HIGH);
    delay(100);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire1.begin(I2C1_SDA, I2C1_SCL); // Ensure pins defined in config

    Serial.println("Initializing sensors...");

    // BH1750 (Light) - *** FIX: Corrected begin() call ***
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        has_lightSensor = true;
        Serial.println("BH1750 initialized.");
        lightCard = new Card(&dashboard, GENERIC_CARD, "Light", "lx");
    } else {
        Serial.println("BH1750 not found!");
    }

    // Analog Sensors
    soilValue = new Card(&dashboard, GENERIC_CARD, "Soil Moisture", "%");
    saltValue = new Card(&dashboard, GENERIC_CARD, "Soil Salinity", "%");
    batteryValue = new Card(&dashboard, GENERIC_CARD, "Battery", "V");

    // BME280 (Temp/Hum/Press/Alt)
    if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
        has_bmeSensor = true;
        Serial.printf("BME280 initialized (Addr: 0x%02X).\n", bme.sensorID() == 0x60 ? 0x76 : 0x77); // ID 0x60 corresponds to 0x76 or 0x77 address
        bmeTemperature = new Card(&dashboard, TEMPERATURE_CARD, "BME Temp", "°C");
        bmeHumidity = new Card(&dashboard, HUMIDITY_CARD, "BME Humidity", "%");
        bmeAltitudeCard = new Card(&dashboard, GENERIC_CARD, "Altitude", "m");
        bmePressureCard = new Card(&dashboard, GENERIC_CARD, "Pressure", "hPa");
    } else {
        Serial.println("BME280 not found!");
    }

    // DHT11 (Temp/Hum)
    if (!has_bmeSensor && dhtSensorProbe()) {
        has_dht11 = true;
        Serial.println("DHT11 initialized.");
        dhtTemperature = new Card(&dashboard, TEMPERATURE_CARD, "DHT11 Temp", "°C");
        dhtHumidity = new Card(&dashboard, HUMIDITY_CARD, "DHT11 Humidity", "%");
    } else if (!has_bmeSensor) {
        Serial.println("DHT11 not found/failed!");
    }

    // SHT3x (Temp/Hum)
    if (!has_bmeSensor && !has_dht11 && sht31.begin(OB_SHT3X_ADDRESS)) { // Uses Wire1
        has_sht3xSensor = true;
        Serial.println("SHT3x initialized.");
        sht3xTemperature = new Card(&dashboard, TEMPERATURE_CARD, "SHT3X Temp", "°C");
        sht3xHumidity = new Card(&dashboard, HUMIDITY_CARD, "SHT3X Humidity", "%");
    } else if (!has_bmeSensor && !has_dht11) {
        Serial.println("SHT3x not found!");
        Wire1.end(); // Release Wire1
        // DS18B20 (Temp)
        if (ds18b20Begin()) {
            has_ds18b20 = true;
            Serial.println("DS18B20 initialized.");
            dsTemperature = new Card(&dashboard, TEMPERATURE_CARD, "DS18B20 Temp", "°C");
        } else {
            Serial.println("DS18B20 not found!");
        }
    }

    // Edge Impulse Dashboard Cards
    mlClassificationCard = new Card(&dashboard, GENERIC_CARD, "Plant Status", "");
    mlConfidenceCard = new Card(&dashboard, GENERIC_CARD, "Confidence", "%");
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
        mlAnomalyCard = new Card(&dashboard, GENERIC_CARD, "Anomaly Score", "");
        Serial.println("Anomaly detection card created.");
    #endif

  // LoRa (optional)
  setupLoRa();

  // NeoPixel and Motor (if LoRa is not used)
  if (!has_lora_shield) {
    pixels = new Adafruit_NeoPixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
    if (pixels) {
      pixels->begin();
      pixels->setBrightness(50);
      pixels->clear(); // Initialize to off
      pixels->show();
    }

    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);

    // IO19 is initialized as motor drive pin
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);

    motorButton = new Card(&dashboard, BUTTON_CARD, DASH_MOTOR_CTRL_STRING);

    motorButton->attachCallback([&](bool value) {

        Serial.println("motorButton Triggered: " + String((value) ? "ON" : "OFF"));

        digitalWrite(MOTOR_PIN, value);

        if (pixels) {
            pixels->setPixelColor(0, value ? pixels->Color(0, 255, 0) : pixels->Color(0, 0, 0));
            pixels->show();
        }

        motorButton->update(value);

        dashboard.sendUpdates();
    });
  }

    setupWiFi(); // Setup WiFi and start web server

    Serial.println("--- Setup Complete ---");
}

void loop() {
    button.loop();
    useButton.loop();

    static unsigned long last_run = 0;
    unsigned long interval = 5000; // Run every 5 seconds

    if (millis() - last_run >= interval) {
        last_run = millis();
        Serial.println("\n--- Sensor Read & Inference Cycle ---");

        higrow_sensors_event_t sens_data;
        sens_data.primary_temp = NAN; sens_data.primary_hum = NAN;
        sens_data.temperature = NAN; sens_data.humidity = NAN;

        // --- Read Sensors ---
        if (has_bmeSensor) get_higrow_sensors_event(BME280_SENSOR_ID, sens_data);
        if (has_dht11) get_higrow_sensors_event(DHTxx_SENSOR_ID, sens_data);
        if (has_sht3xSensor) get_higrow_sensors_event(SHT3x_SENSOR_ID, sens_data);
        if (has_ds18b20) get_higrow_sensors_event(DS18B20_SENSOR_ID, sens_data);
        if (has_lightSensor) get_higrow_sensors_event(BHT1750_SENSOR_ID, sens_data);
        get_higrow_sensors_event(SOIL_SENSOR_ID, sens_data);
        get_higrow_sensors_event(SALT_SENSOR_ID, sens_data);
        get_higrow_sensors_event(VOLTAGE_SENSOR_ID, sens_data);

        // --- Update Sensor Cards ---
        if (lightCard && has_lightSensor && !isnan(sens_data.light)) lightCard->update(sens_data.light);
        if (soilValue) soilValue->update(sens_data.soli);
        if (saltValue) saltValue->update(sens_data.salt);
        if (batteryValue && !isnan(sens_data.voltage)) batteryValue->update(sens_data.voltage);
        if (bmeTemperature && has_bmeSensor && !isnan(sens_data.temperature)) bmeTemperature->update(sens_data.temperature);
        if (bmeHumidity && has_bmeSensor && !isnan(sens_data.humidity)) bmeHumidity->update(sens_data.humidity);
        if (bmeAltitudeCard && has_bmeSensor && !isnan(sens_data.altitude)) bmeAltitudeCard->update(sens_data.altitude);
        if (bmePressureCard && has_bmeSensor && !isnan(sens_data.pressure)) bmePressureCard->update(sens_data.pressure);
        if (dhtTemperature && has_dht11 && !isnan(sens_data.temperature)) dhtTemperature->update(sens_data.temperature);
        if (dhtHumidity && has_dht11 && !isnan(sens_data.humidity)) dhtHumidity->update(sens_data.humidity);
        if (sht3xTemperature && has_sht3xSensor && !isnan(sens_data.temperature)) sht3xTemperature->update(sens_data.temperature);
        if (sht3xHumidity && has_sht3xSensor && !isnan(sens_data.humidity)) sht3xHumidity->update(sens_data.humidity);
        if (dsTemperature && has_ds18b20 && !isnan(sens_data.temperature)) dsTemperature->update(sens_data.temperature);

        // --- Prepare Edge Impulse Features ---
        feature_ix = 0;
        Serial.print("Features: ");
        // IMPORTANT: This order MUST match your Edge Impulse model input!
        features[feature_ix++] = sens_data.soli; Serial.printf(" Moisture:%d", sens_data.soli); //#1 Moisture
        features[feature_ix++] = sens_data.salt; Serial.printf(" Salinity:%d", sens_data.salt); //#4 Phosphorous
        //if (!isnan(sens_data.temperature)) { features[feature_ix++] = sens_data.temperature; Serial.printf(" Temp:%.1f", sens_data.temperature); } else { features[feature_ix++] = 0; Serial.print(" Temp:NaN");} //#2 Temp
        //if (!isnan(sens_data.humidity)) { features[feature_ix++] = sens_data.humidity; Serial.printf(" Humid:%.1f", sens_data.humidity); } else { features[feature_ix++] = 0; Serial.print(" Humid:NaN");} //#3 Nitrogen
        //if (has_lightSensor && !isnan(sens_data.light)) { features[feature_ix++] = sens_data.light; Serial.printf(" Light:%.0f", sens_data.light); } else { features[feature_ix++] = 0; Serial.print(" Light:NaN");}
        // Add more features here if needed, ensuring total matches EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
        Serial.printf(" (%d/%d)\n", feature_ix, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

        // --- Run Inference ---
        if (feature_ix != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            ei_printf("ERR: Feature count mismatch! Expected %d, got %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, feature_ix);
        } else {
            signal_t signal;
            int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
            if (err != 0) {
                ei_printf("ERR: signal_from_buffer failed (%d)\n", err);
            } else {
                ei_printf("Running classifier...\n");
                ei_impulse_result_t result = { 0 };
                err = run_classifier(&signal, &result, debug_nn);
                if (err != EI_IMPULSE_OK) {
                    ei_printf("ERR: run_classifier failed (%d)\n", err);
                } else {
                    ei_printf("Timing: DSP %dms, Classify %dms, Anomaly %dms\n", result.timing.dsp, result.timing.classification, result.timing.anomaly);
                    float max_confidence = 0.0;
                    int max_index = -1;
                    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                        ei_printf("  %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
                        if (result.classification[ix].value > max_confidence) {
                            max_confidence = result.classification[ix].value;
                            max_index = ix;
                        }
                    }

                    // Update ML Cards - *** FIX: Cast double to float for update() ***
                    if (mlClassificationCard && max_index != -1) mlClassificationCard->update(result.classification[max_index].label); else if (mlClassificationCard) mlClassificationCard->update("N/A");
                    if (mlConfidenceCard && max_index != -1) mlConfidenceCard->update( (float)(max_confidence * 100.0) ); else if (mlConfidenceCard) mlConfidenceCard->update(0.0f);
                    #if EI_CLASSIFIER_HAS_ANOMALY == 1
                        ei_printf("Anomaly score: %.3f\n", result.anomaly);
                        if (mlAnomalyCard) mlAnomalyCard->update( (float)result.anomaly ); // Cast to float
                    #endif

                    // NeoPixel feedback
                    if (pixels && max_index != -1) {
                        const char* label = result.classification[max_index].label;
                        Serial.printf("Prediction: %s (%.1f%%)\n", label, max_confidence * 100.0);
                        // Customize labels/colors
                        if (strcmp(label, "Healthy") == 0) pixels->setPixelColor(0, pixels->Color(0, 50, 0)); // Green
                        else if (strcmp(label, "Needs_Water") == 0) pixels->setPixelColor(0, pixels->Color(50, 50, 0)); // Yellow
                        else pixels->setPixelColor(0, pixels->Color(50, 0, 0)); // Red
                        pixels->show();
                    } else if (pixels) { pixels->clear(); pixels->show(); }

                    // LoRa Update
                    loopLoRa(&sens_data, &result);
                }
            }
        }

        // --- Automatic Watering ---
        #ifdef AUTO_WATER
            uint8_t soli_val_for_water = sens_data.soli;
            uint8_t water_on_threshold = 26; // Example threshold
            uint8_t water_off_threshold = 40; // Example threshold
            bool current_pump_state = digitalRead(MOTOR_PIN);
            if (soli_val_for_water < water_on_threshold && !current_pump_state) {
                Serial.println("AutoWater: Soil dry, turning ON pump.");
                WateringCallback(true);
            } else if (soli_val_for_water >= water_off_threshold && current_pump_state) {
                Serial.println("AutoWater: Soil moist, turning OFF pump.");
                WateringCallback(false);
            }
        #endif

        // --- Send Dashboard Updates ---
        Serial.println("Sending dashboard updates...");
        dashboard.sendUpdates();
        Serial.println("--- Cycle End ---");
    }
}

// --- LoRa Loop ---
void loopLoRa(higrow_sensors_event_t *val, ei_impulse_result_t *result) {
    if (!has_lora_shield) return;
    cJSON *root = cJSON_CreateObject();
    if (!root) { Serial.println("LoRa JSON create failed"); return; }

    // Sensor Data (short keys)
    if (has_lightSensor && !isnan(val->light)) cJSON_AddNumberToObject(root, "L", roundf(val->light));
    cJSON_AddNumberToObject(root, "S", val->soli);
    cJSON_AddNumberToObject(root, "Sa", val->salt);
    if (!isnan(val->voltage)) cJSON_AddNumberToObject(root, "V", roundf(val->voltage * 100.0) / 100.0);
    if (!isnan(val->temperature)) cJSON_AddNumberToObject(root, "T", roundf(val->temperature * 10.0) / 10.0);
    if (!isnan(val->humidity)) cJSON_AddNumberToObject(root, "H", roundf(val->humidity * 10.0) / 10.0);

    // ML Result (short keys)
    if (result != NULL && EI_CLASSIFIER_LABEL_COUNT > 0) {
        float max_conf = 0.0; int max_idx = -1;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) { if (result->classification[ix].value > max_conf) { max_conf = result->classification[ix].value; max_idx = ix; } }
        if(max_idx != -1) { cJSON_AddNumberToObject(root, "MLi", max_idx); cJSON_AddNumberToObject(root, "MLc", roundf(max_conf * 100.0)); }
    }
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
        if (result != NULL) cJSON_AddNumberToObject(root, "AN", roundf(result->anomaly * 100.0));
    #endif

    char *packet = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); // Delete JSON object now, whether print worked or not
    if (!packet) { Serial.println("LoRa JSON print failed"); return; }

    Serial.print("LoRa TX: "); Serial.println(packet);
    if (LoRa.beginPacket()) { LoRa.print(packet); LoRa.endPacket(); Serial.println("LoRa TX Complete."); }
    else { Serial.println("LoRa TX Failed!"); }
    free(packet);
}

// --- LoRa Setup ---
void setupLoRa() {
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN); // Pins from config
    LoRa.setPins(RADIO_CS_PIN, RADIO_RESET_PIN, RADIO_DI0_PIN); // Pins from config
    if (!LoRa.begin(LoRa_frequency)) { // Freq from config
        Serial.println("LoRa Init FAILED!");
        SPI.end(); has_lora_shield = false; return;
    }
    // Optional: Configure LoRa parameters (SF, BW, CR, Preamble, Sync Word, Power, CRC)
    // LoRa.setTxPower(14); // Example
    // LoRa.setSpreadingFactor(7); // Example
    has_lora_shield = true;
    Serial.println("LoRa Init OK.");
}