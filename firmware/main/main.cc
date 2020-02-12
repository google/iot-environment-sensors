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

#include <EEPROM.h>
#include <FS.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cinttypes>
#include <string>

#include "google-mqtt.h"
#include "sensor_bmp280.h"
#include "sensor_scd30.h"
#include "sensor_sgp30.h"
#include "sensor_smuart04l.h"

SemaphoreHandle_t i2c_mutex;

#define WIFI_REPORTING_ENABLED
static const int num_sensors = 4;
HardwareSerial smuart04l_serial(1);

std::array<SensorWrapper *, num_sensors> sensors;
std::array<std::array<StackType_t, 8 * 1024>, num_sensors + 2> stacks;
std::array<StaticTask_t, num_sensors + 2> tasks;
std::vector<TemperatureSensor *> temperature_sensors;

void ServerUpdateLoopMqtt(void *pv_parameters) {
  std::vector<SensorWrapper *> sensor_vector;
  for (SensorWrapper *sensor : sensors) {
    sensor_vector.push_back(sensor);
  }
  SetupGoogleMQTT(sensor_vector, temperature_sensors);
  LoopGoogleMQTT();
}

void ServerWatchdogLoopMqtt(void *pv_parameters) { GoogleMQTTWatchdog(); }

void ScanI2c() {
  byte error, address;
  int num_devices;

  Serial.println("Scanning...");

  num_devices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      num_devices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (num_devices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void loop() {
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

void SensorWrapperLoop(void *pv_parameters) {
  SensorWrapper *wrapper = reinterpret_cast<SensorWrapper *>(pv_parameters);
  wrapper->Loop();
}

void setup() {
  i2c_mutex = xSemaphoreCreateMutex();

  /* Init I2C and serial communication */
  Serial.begin(115200);

  uint8_t mac[6];
  if (esp_efuse_mac_get_default(mac) != ESP_OK) {
    Serial.println("Failed to get MAC");
  } else {
    Serial.printf("MAC: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%x", mac[i]);
    }
    Serial.printf("\n");
  }

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS failed");
    esp_restart();
  }

  {
    File file = SPIFFS.open("/mac.dat");
    if (!file || file.isDirectory()) {
      Serial.println("MAC file not present.");
      esp_restart();
    }

    uint8_t read_mac[6];
    if (file.readBytes((char *)(read_mac), 6) < 6) {
      Serial.println("MAC file too small.");
      esp_restart();
    }

    for (int i = 0; i < 6; i++) {
      if (read_mac[i] != mac[i]) {
        Serial.println("MAC mismatch.");
        esp_restart();
      }
    }
  }

  EEPROM.begin(4096);

  Wire.begin();
  ScanI2c();

  Bmp280Sensor *bmp280 = new Bmp280Sensor(&Serial, &i2c_mutex);
  Scd30Sensor *scd30 = new Scd30Sensor(&Serial, &i2c_mutex, bmp280);
  sensors = {
      bmp280,
      scd30,
      new Sgp30Sensor(&Serial, &i2c_mutex, scd30, scd30),
      new Smuart04lSensor(&Serial, &smuart04l_serial),
  };
  temperature_sensors = {scd30, bmp280};

  for (SensorWrapper *sensor : sensors) {
    sensor->Init();
  }

  for (int i = 0; i < sensors.size(); i++) {
    SensorWrapper *sensor = sensors[i];
    xTaskCreateStaticPinnedToCore(SensorWrapperLoop, "SensorLoop",
                                  stacks[i].size(), sensor, /*priority=*/1,
                                  stacks[i].data(), &tasks[i], 1);
  }

#ifdef WIFI_REPORTING_ENABLED
  xTaskCreateStaticPinnedToCore(ServerUpdateLoopMqtt, "ServerUpdateLoopMqtt",
                                stacks[sensors.size()].size(), nullptr,
                                /*priority=*/2, stacks[sensors.size()].data(),
                                &tasks[sensors.size()], 1);
  xTaskCreateStaticPinnedToCore(
      ServerWatchdogLoopMqtt, "ServerWatchdogLoopMqtt",
      stacks[sensors.size() + 1].size(), nullptr,
      /*priority=*/2, stacks[sensors.size() + 1].data(),
      &tasks[sensors.size() + 1], 1);
#endif
}
