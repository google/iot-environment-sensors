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

#include "sensor_sgp30.h"

#include <SPIFFS.h>

#include <array>
#include <cmath>

Sgp30Sensor::Sgp30Sensor(HardwareSerial* debug_serial,
                         SemaphoreHandle_t* i2c_mutex,
                         TemperatureSensor* temperature_sensor,
                         HumiditySensor* humidity_sensor)
    : reading_{},
      failed_readings_(0),
      debug_serial_(*debug_serial),
      i2c_mutex_(*i2c_mutex),
      reading_mutex_(xSemaphoreCreateMutex()),
      temperature_sensor_(temperature_sensor),
      humidity_sensor_(humidity_sensor),
      abs_humidity_hardware_format_(0),
      abs_humidity_(0) {}

const char* baseline_filename = "/sgp30_baselines.dat";

bool GetSgpFile(const char* path, size_t expected_size, char* contents) {
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

void Sgp30Sensor::Init() {
  uint16_t baselines[2];
  const bool baselines_read =
      GetSgpFile(baseline_filename, 4, (char*)baselines);

  if (xSemaphoreTake(i2c_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
    debug_serial_.println("SGP30: Couldn't acquire mutex for SGP30 init.");
    return;
  }
  if (!sgp30_.begin()) {
    debug_serial_.println("SGP30: Failed to init SGP30.");
  }
  sgp30_.initAirQuality();
  if (baselines_read) {
    debug_serial_.println("SGP30: Setting baselines.");
    sgp30_.setBaseline(baselines[0], baselines[1]);
  }
  xSemaphoreGive(i2c_mutex_);
  debug_serial_.println("SGP30: SGP30 init complete.");
}

uint16_t Sgp30Sensor::AbsHumidityForSensor(float relative_humidity,
                                           float temperature) {
  // From
  // https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
  abs_humidity_ = ((6.112 * exp((17.67 * temperature) / (temperature + 243.5)) *
                    relative_humidity * 2.1674) /
                   (273.15 + temperature));
  debug_serial_.printf(
      "SGP30: Temp: %.2f°C, Hum: %.2f%%, Abs. Hum: %.2fg/m^3\n", temperature,
      relative_humidity, abs_humidity_);
  const uint32_t abs_humidity = abs_humidity_ * 1000.0f;
  const uint16_t ah_scaled = (uint16_t)((abs_humidity * 16777) >> 16);
  return ah_scaled;
}

void Sgp30Sensor::Loop() {
  int iterations_until_humidity_update = 20;
  int iterations_until_baseline_update = 30;
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    if (iterations_until_humidity_update == 0 &&
        temperature_sensor_ != nullptr && humidity_sensor_ != nullptr) {
      float relative_humidity = humidity_sensor_->GetHumidity();
      float temperature = temperature_sensor_->GetTemperature();
      if (relative_humidity != 0 && temperature != 0) {
        abs_humidity_hardware_format_ =
            AbsHumidityForSensor(relative_humidity, temperature);
      }
      iterations_until_humidity_update = 60;
    } else {
      iterations_until_humidity_update--;
    }

    if (xSemaphoreTake(i2c_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
      debug_serial_.println("SGP30: Couldn't acquire mutex for SGP30 update.");
      continue;
    }

    if (abs_humidity_hardware_format_ != 0) {
      sgp30_.setHumidity(abs_humidity_hardware_format_);
    }

    SGP30ERR result = sgp30_.measureAirQuality();

    if (result != SGP30ERR::SUCCESS) {
      debug_serial_.printf("SGP30: SGP30 status was not success, was %u\n",
                           result);
      failed_readings_++;
      xSemaphoreGive(i2c_mutex_);
      continue;
    }

    result = sgp30_.getBaseline();
    if (result != SGP30ERR::SUCCESS) {
      debug_serial_.printf(
          "SGP30: SGP30 baseline status was not success, was %u\n", result);
      failed_readings_++;
    }

    result = sgp30_.measureRawSignals();
    if (result != SGP30ERR::SUCCESS) {
      debug_serial_.printf(
          "SGP30: SGP30 raw signal status was not success, was %u\n", result);
      failed_readings_++;
    }
    xSemaphoreGive(i2c_mutex_);

    if (xSemaphoreTake(reading_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
      debug_serial_.println(
          "SGP30: Couldn't acquire environment mutex for SGP30 update.");
    }
    reading_.co2 = sgp30_.CO2;
    reading_.tvoc = sgp30_.TVOC;
    reading_.baseline_co2 = sgp30_.baselineCO2;
    reading_.baseline_tvoc = sgp30_.baselineTVOC;
    reading_.h2 = sgp30_.H2;
    reading_.ethanol = sgp30_.ethanol;
    xSemaphoreGive(reading_mutex_);

    debug_serial_.printf("SGP30: Got readings from SGP30: ");
    for (int i = 0; i < 6; i++) {
      debug_serial_.printf("%u ", *(&reading_.co2 + i));
    }
    debug_serial_.printf("\n");

    if (iterations_until_baseline_update == 0) {
      debug_serial_.println("SGP30: Saving baselines.");

      File file = SPIFFS.open(baseline_filename, "w");
      if (!file || file.isDirectory()) {
        debug_serial_.printf("SGP30: File not present: %s\n",
                             baseline_filename);
      } else {
        const uint16_t baselines[] = {sgp30_.baselineCO2, sgp30_.baselineTVOC};
        if (file.write((uint8_t*)baselines, 4) != 4) {
          debug_serial_.println("SGP30: Failed to write baselines.");
        }
      }
      iterations_until_baseline_update = 3600 * 24;
    } else {
      iterations_until_baseline_update--;
    }
  }
}

void Sgp30Sensor::UpdateSensor(SensorUpdate* update) {
  xSemaphoreTake(reading_mutex_, portMAX_DELAY);
  update->sgp30->co2 = reading_.co2;
  update->sgp30->has_co2 = true;
  update->sgp30->tvoc = reading_.tvoc;
  update->sgp30->has_tvoc = true;
  update->sgp30->baseline_co2 = reading_.baseline_co2;
  update->sgp30->has_baseline_co2 = true;
  update->sgp30->baseline_tvoc = reading_.baseline_tvoc;
  update->sgp30->has_baseline_tvoc = true;
  update->sgp30->h2 = reading_.h2;
  update->sgp30->has_h2 = true;
  update->sgp30->ethanol = reading_.ethanol;
  update->sgp30->has_ethanol = true;
  xSemaphoreGive(reading_mutex_);
};