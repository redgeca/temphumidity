// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#define DHTPIN GPIO_NUM_16     // Digital pin connected to the DHT sensor

// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void pushMessage(char *topic, char*message) {

}

void sensorTask(void *parameters) {
    // Initialize device.
    dht.begin();
    delayMS = 2000; // Delay between measurements.

    float oldTemperature = 0.0f;
    float oldHumidity = 0.0f;
    float currentTemperature = 0.0f;
    float currentHumidity = 0.0f;

    while(true) {
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        if (isnan(event.temperature)) {
            Serial.println(F("Error reading temperature!"));
            continue;
        }
        if (isnan(event.relative_humidity)) {
            Serial.println(F("Error reading humidity!"));
            continue;
        }
        currentTemperature = event.temperature;
        currentHumidity = event.relative_humidity;

        if (oldTemperature != currentTemperature || oldHumidity != currentHumidity) {
            oldTemperature = currentTemperature;
            oldHumidity = currentHumidity;

            JsonDocument doc;
            doc["temp"] = oldTemperature;
            doc["humidity"] = oldHumidity;
            char outputJson[256];

            Serial.println();
            serializeJsonPretty(doc, Serial);
            serializeJsonPretty(doc, outputJson);
        }

        vTaskDelay(delayMS / portTICK_PERIOD_MS);
    }

}

void setup() {
    Serial.begin(115200);
    // Little delay, to have serial displaying
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    // Create the WiFiManager
    WiFiManager wifiManager;
    wifiManager.autoConnect("ESP32Minikit-dht11");


    // Create the sensor task
    xTaskCreate(sensorTask, "Temperature/Humidity sensor Task", 2048, NULL, 1, NULL);

}


void loop() {

}
