// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "google-mqtt.h"

#include <Client.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_https_ota.h>

#include "google-cloud-iot-arduino/src/CloudIoTCore.h"
#include "google-cloud-iot-arduino/src/CloudIoTCoreMqtt.h"
#include "mqtt/src/MQTT.h"
#include "sensor.pb-c.h"
#include "sensor_types.h"
#include "sensor_wrapper.h"

const char kNtpPrimary[] = "pool.ntp.org";
const char kNtpSecondary[] = "time.nist.gov";

extern const char googleca_pem_start[] asm("_binary_googleca_pem_start");

const size_t kPrivateKeySize = 95;
char private_key[kPrivateKeySize + 1];

const size_t kDeviceIdSize = 12;
char device_id[kDeviceIdSize + 1];

Client *client_;
CloudIoTCoreDevice *device_;
CloudIoTCoreMqtt *mqtt_;
MQTTClient *mqtt_client_;
unsigned long iss_ = 0;
const int jwt_exp_secs = 3600;  // Maximum 24H (3600*24)
String jwt_;
std::vector<SensorWrapper *> sensors_;
std::vector<TemperatureSensor *> temperature_sensors_;

unsigned long last_update = 0;

String getJwt() {
  iss_ = time(nullptr);
  Serial.println("MQTT: Refreshing JWT");
  jwt_ = device_->createJWT(iss_, jwt_exp_secs);
  return jwt_;
}

bool GetFile(const char *path, size_t expected_size, char *contents) {
  File file = SPIFFS.open(path);
  if (!file || file.isDirectory()) {
    Serial.printf("SPIFFS: File not present: %s\n", path);
    return false;
  }

  if (file.readBytes(contents, expected_size) < expected_size) {
    Serial.printf("SPIFFS: File %s not correct length.\n", path);
    return false;
  }
  return true;
}

void messageReceived(String &topic, String &payload) {
  static const String update_prefix = "update: ";
  static const String temp_calibration_prefix = "temp_calibration: ";
  Serial.printf("MQTT: Received data for topic %s: %s\n", topic.c_str(),
                payload.c_str());
  if (topic.endsWith("commands") && payload.startsWith(update_prefix)) {
    const char *url =
        payload.c_str() + update_prefix.length();  // Len of "update: "
    Serial.printf("MQTT: Got OTA update command using URL %s\n", url);
    esp_http_client_config_t config{};
    config.url = url;
    config.cert_pem = googleca_pem_start;
    const esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
      Serial.println("OTA update okay, restarting.");
      esp_restart();
    } else {
      Serial.println("OTA update failed.");
    }
  }
  if (topic.endsWith("commands") &&
      payload.startsWith(temp_calibration_prefix)) {
    const float actual_temp =
        payload.substring(temp_calibration_prefix.length()).toFloat();
    for (TemperatureSensor *sensor : temperature_sensors_) {
      reinterpret_cast<TemperatureSensor *>(sensor)->CalibrateTemperature(
          actual_temp);
    }
  }
  if (topic.endsWith("commands") && payload.startsWith("reset")) {
    esp_restart();
  }
}

void WifiConnect() {
  Serial.printf("WiFi: Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(CONFIG_GIOT_WIFI_AP, CONFIG_GIOT_WIFI_PASSWORD);

  int iterations_left = 100;
  while (iterations_left > 0 && WiFi.status() != WL_CONNECTED) {
    vTaskDelay(250 / portTICK_PERIOD_MS);
    Serial.print(".");
    iterations_left--;
  }
  Serial.printf("\n");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFI: Connection failed, restarting.");
    esp_restart();
    return;
  }
  Serial.printf("WiFi: Connected: %s\n", WiFi.localIP().toString().c_str());
}

void WifiReconnect() {
  Serial.printf("WiFi: Disconnecting from WiFi.");
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  Serial.printf("WiFi: Delaying 500ms.");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  WifiConnect();
}

void SetupGoogleMQTT(std::vector<SensorWrapper *> sensors,
                     std::vector<TemperatureSensor *> temperature_sensors) {
  // Give everything else a chance to start up properly.
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  WifiConnect();
  sensors_ = sensors;
  temperature_sensors_ = temperature_sensors;
  Serial.println("MQTT: Setting up Google MQTT");
  configTime(0, 0, kNtpPrimary, kNtpSecondary);
  Serial.println("MQTT: Waiting on time sync...");
  while (time(nullptr) < 1510644967) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  Serial.printf("MQTT: Time synced @%ld\n", time(nullptr));

  device_id[kDeviceIdSize] = 0;
  if (!GetFile("/devid.dat", kDeviceIdSize, device_id)) {
    Serial.println("MQTT: Couldn't get device ID.");
    vTaskDelete(NULL);
    return;
  }

  private_key[kPrivateKeySize] = 0;
  if (!GetFile("/ec_private.dat", kPrivateKeySize, private_key)) {
    Serial.println("MQTT: Couldn't get private key.");
    vTaskDelete(NULL);
    return;
  }

  device_ =
      new CloudIoTCoreDevice(CONFIG_GIOT_PROJECT_ID, CONFIG_GIOT_LOCATION,
                             CONFIG_GIOT_REGISTRY_ID, device_id, private_key);
  client_ = new WiFiClientSecure();
  mqtt_client_ = new MQTTClient(512);
  mqtt_client_->setOptions(180, true, 1000);
  mqtt_ = new CloudIoTCoreMqtt(mqtt_client_, client_, device_);
  mqtt_->setUseLts(true);
  mqtt_->startMQTT();
  Serial.println("MQTT: initialized.");
}

void UpdateSensors() {
  SensorUpdate update = SENSOR_UPDATE__INIT;

  BMP280Reading bmp280 = BMP280_READING__INIT;
  update.bmp280 = &bmp280;
  SCD30Reading scd30 = SCD30_READING__INIT;
  update.scd30 = &scd30;
  SGP30Reading sgp30 = SGP30_READING__INIT;
  update.sgp30 = &sgp30;
  SMUART04LReading smuart04l = SMUART04_LREADING__INIT;
  update.smuart04l = &smuart04l;

  update.has_timestamp = true;
  update.timestamp = time(nullptr);

  for (SensorWrapper *sensor : sensors_) {
    sensor->UpdateSensor(&update);
  }

  size_t update_size = sensor_update__get_packed_size(&update);
  uint8_t update_packed[update_size + 1];
  update_packed[update_size] = 0;
  sensor_update__pack(&update, update_packed);

  const bool result =
      mqtt_->publishTelemetry((char *)update_packed, update_size);
  if (result) {
    Serial.println("MQTT: Published update.");
  } else {
    Serial.println("MQTT: Update false?");
  }
}

void LoopGoogleMQTT() {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      WifiReconnect();
    }

    if (!mqtt_client_->connected()) {
      Serial.println("MQTT: Google MQTT disconnected, reconnecting");
      mqtt_->mqttConnect();
      continue;
    }
    mqtt_client_->loop();

    Serial.println("MQTT: Google MQTT connected");
    const time_t time_now = time(nullptr);
    if (time_now - last_update > 10) {
      Serial.println("MQTT: Publishing telemetry");
      UpdateSensors();
      last_update = time_now;
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void GoogleMQTTWatchdog() {
  while (true) {
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    const time_t time_now = time(nullptr);
    if (last_update == 0) {
      continue;
    }
    if (time_now - last_update > 120) {
      Serial.println("MQTT: >60s since last update, resetting.");
      esp_restart();
    }
  }
}