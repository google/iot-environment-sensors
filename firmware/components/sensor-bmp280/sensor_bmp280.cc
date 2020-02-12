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

#include "sensor_bmp280.h"

#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SPIFFS.h>

#include "adafruit/Adafruit_BMP280.h"
#include "sensor_types.h"
#include "sensor_wrapper.h"

bool GetBMPFile(const char* path, size_t expected_size, char* contents) {
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

Bmp280Sensor::Bmp280Sensor(HardwareSerial* debug_serial,
                           SemaphoreHandle_t* i2c_mutex)
    : reading_{}, debug_serial_(*debug_serial), i2c_mutex_(*i2c_mutex), temperature_offset_(0) {
  reading_mutex_ = xSemaphoreCreateMutex();
}

void Bmp280Sensor::Init() {
  if (xSemaphoreTake(i2c_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
    debug_serial_.println("BMP280: Couldn't acquire mutex for bmp280 init.");
    return;
  }
  if (!bmp280_.begin(BMP280_ADDRESS_ALT)) {
    debug_serial_.println("BMP280: Failed to initialize bmp280");
  }

  bmp280_.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X16, /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  xSemaphoreGive(i2c_mutex_);

  float new_offset;
  if(GetBMPFile("/bmp280_offset.dat", 4, (char*)&new_offset)) {
    debug_serial_.printf("BMP280: Read temperature offset %f\n", new_offset);
    temperature_offset_ = new_offset;
  }

  debug_serial_.println("BMP280: BMP280 init complete.");
}

void Bmp280Sensor::Loop() {
  while (1) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (xSemaphoreTake(i2c_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
      debug_serial_.println(
          "BMP280: Couldn't acquire mutex for BMP280 update.");
      continue;
    }
    float pressure = bmp280_.readPressure();
    float temperature = bmp280_.readTemperature();
    xSemaphoreGive(i2c_mutex_);

    debug_serial_.printf("BMP280: data %f, %f, %f\n", pressure, temperature, temperature + temperature_offset_);
    if (xSemaphoreTake(reading_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
      debug_serial_.println(
          "BMP280: Couldn't acquire environment mutex for BMP280 update.");
      continue;
    }
    reading_.pressure = pressure;
    reading_.temperature = temperature;
    xSemaphoreGive(reading_mutex_);
  }
}

void Bmp280Sensor::CalibrateTemperature(float actual_temp) {
  if (xSemaphoreTake(reading_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
    debug_serial_.println(
        "BMP280: Couldn't acquire environment mutex for BMP280 temp calibration.");
    return;
  }
  const float last_reading = reading_.temperature;
  const float new_offset = actual_temp - last_reading;
  debug_serial_.printf("BMP280: Changing temperature offset. Old: %f, new: %f\n", temperature_offset_, new_offset);
  temperature_offset_ = new_offset;
  xSemaphoreGive(reading_mutex_);

  File file = SPIFFS.open("/bmp280_offset.dat", "w");
  if (!file || file.isDirectory()) {
    debug_serial_.println("Couldn't open offset file.");
  } else {
    if (file.write((uint8_t*)&temperature_offset_, 4) != 4) {
      debug_serial_.println("BMP280: Failed to write temperature offset.");
    }
  }
}