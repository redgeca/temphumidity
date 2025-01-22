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
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <time.h>
#include <WiFiClientSecure.h>
#include "certificate.h"

#define DHTPIN GPIO_NUM_16     // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT11

WiFiClientSecure securedClient;
PubSubClient mqttClient(securedClient);


void pushMessage(char *topic, char*message) {

}

void iotserverCommandsCallback(char *topic, byte *payload, unsigned int length) {
    payload[length + 2] = '\0';
    Serial.printf("\nMessage [%s] with content [%s]\n", topic, payload);
}

void sensorTask(void *parameters) {
    DHT_Unified dht(DHTPIN, DHTTYPE);

    Serial.printf("MQTT connected.  Reason : %d\n...", mqttClient.state());
    // Initialize device.
    dht.begin();

    float oldTemperature = 0.0f;
    float currentTemperature = 0.0f;

    while(true) {
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        if (isnan(event.temperature)) {
            Serial.println(F("Error reading temperature!"));
            mqttClient.publish("iotserver/errors", "Error reading temperature!");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        currentTemperature = event.temperature;

        if (oldTemperature != currentTemperature) {
            oldTemperature = currentTemperature;

            JsonDocument doc;
            doc["temp"] = oldTemperature;
            char outputJson[256];

            Serial.println();
//            serializeJsonPretty(doc, Serial);
            serializeJsonPretty(doc, outputJson);

            mqttClient.publish("iotserver/commands", outputJson);
        }

        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}

/**
void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
}
*/


void setDateTime() {
  // You can use your own timezone, but the exact time is not used at all.
  // Only the date is needed for validating the certificates.
  configTime(-18000, 3600, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(100);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println();

  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.printf("%s %s", tzname[0], asctime(&timeinfo));
}

void setup() {
    Serial.begin(115200);

    // Little delay, to have serial displaying
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    // Create the WiFiManager
    WiFiManager wifiManager;
    wifiManager.autoConnect("ESP32Minikit-dht11");

    setDateTime();
    //printLocalTime();

    securedClient.setCACert(HiveMQClientCertificate);
    securedClient.setCertificate(HiveMQClientCertificate);
    mqttClient.setServer("6ff99c4d5207411b94d3dcebbaa9e437.s1.eu.hivemq.cloud", 8883);

    while (!mqttClient.connect("ESP32Minikit-dht11", "rcaron", "Apsodi81!")) {
         Serial.printf("MQTT connection failed, retrying in 5 seconds.  Reason : %d\n...", mqttClient.state());
         vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    Serial.println("Setting callback and subscriptions");
    mqttClient.setCallback(iotserverCommandsCallback);
    mqttClient.subscribe("iotserver/commands", 1);

    // Create the sensor task
    xTaskCreate(sensorTask, "Temperature/Humidity sensor Task", 4096, NULL, 1, NULL);
}


void loop() {
    mqttClient.loop();
}
