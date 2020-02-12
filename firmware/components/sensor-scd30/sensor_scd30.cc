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

#include "sensor_scd30.h"

#include <array>
#include <string>

Scd30Sensor::Scd30Sensor(HardwareSerial* debug_serial,
                         SemaphoreHandle_t* i2c_mutex,
                         PressureSensor* pressure_sensor)
    : reading_{},
      failed_readings_(0),
      debug_serial_(*debug_serial),
      i2c_mutex_(*i2c_mutex),
      pressure_sensor_(pressure_sensor) {
  reading_mutex_ = xSemaphoreCreateMutex();
}

void Scd30Sensor::Init() {
  if (xSemaphoreTake(i2c_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
    debug_serial_.println("SCD30: Couldn't acquire mutex for SCD30 init.");
    return;
  }
  if (!scd30_.begin()) {
    debug_serial_.println("SCD30: Failed to initialize SCD30.");
    xSemaphoreGive(i2c_mutex_);
    return;
  }
  if (!scd30_.setAutoSelfCalibration(true)) {
    debug_serial_.println("SCD30: Failed to enable SCD30 self calibration.");
    xSemaphoreGive(i2c_mutex_);
    return;
  }
  xSemaphoreGive(i2c_mutex_);
  debug_serial_.println("SCD30: SCD30 init complete.");
}

void Scd30Sensor::Loop() {
  vTaskDelay(10000 /
             portTICK_PERIOD_MS);  // Give it 10s for the BME680 to stabilize.
  int remaining_pressure_iterations = 0;
  uint16_t last_pressure = 0;
  while (1) {
    // Serial.println("Starting SCD30 iteration.");
    assert(heap_caps_check_integrity_all(true));
    if (xSemaphoreTake(i2c_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
      debug_serial_.println("SCD30: Couldn't acquire mutex for SCD30 update.");
      continue;
    }
    if (scd30_.dataAvailable()) {
      if (!scd30_.readMeasurement()) {
        debug_serial_.println("SCD30: Failed to read SCD30 data.");
        failed_readings_++;
        xSemaphoreGive(i2c_mutex_);
        continue;
      }
      if (xSemaphoreTake(reading_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
        debug_serial_.println(
            "SCD30: Couldn't acquire environment mutex for SCD30 update.");
        xSemaphoreGive(i2c_mutex_);
        continue;
      }
      reading_.co2 = scd30_.getCO2();
      reading_.temperature = scd30_.getTemperature();
      reading_.humidity = scd30_.getHumidity();
      debug_serial_.printf("SCD30: data %f, %f, %f\n", reading_.co2,
                           reading_.humidity, reading_.temperature);
      xSemaphoreGive(reading_mutex_);
    }
    if (remaining_pressure_iterations == 0) {
      if (pressure_sensor_ != nullptr) {
        const float raw_pressure = pressure_sensor_->GetPressure();
        if (raw_pressure < 800.0 || raw_pressure > 1200.0) {
          debug_serial_.printf("SCD30: Pressure too high to update: %f\n",
                               raw_pressure);
        } else {
          const uint16_t new_pressure =
              static_cast<uint16_t>(raw_pressure + 0.5f);
          if (new_pressure == last_pressure) {
            debug_serial_.println("SCD30: Skipping pressure update.");
          } else {
            debug_serial_.printf("SCD30: Setting pressure to %d\n",
                                 new_pressure);
            scd30_.setAmbientPressure(new_pressure);
            last_pressure = new_pressure;
          }
        }
      }
      remaining_pressure_iterations = 60;  // 10 minutes.
    } else {
      remaining_pressure_iterations--;
    }
    xSemaphoreGive(i2c_mutex_);
    assert(heap_caps_check_integrity_all(true));
    const uint32_t t_ms = 2000;
    const TickType_t t_ticks = t_ms / portTICK_PERIOD_MS;
    // debug_serial_.printf("SCD30 sleeping for %ums, %u ticks\n", t_ms,
    // t_ticks);
    vTaskDelay(t_ticks);
  }
}

void Scd30Sensor::CalibrateTemperature(float actual_temp) {
  debug_serial_.printf("SCD30: Calibrating temperature - %f\n", actual_temp);
  if (xSemaphoreTake(reading_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
    debug_serial_.println(
        "SCD30: Couldn't acquire environment mutex for calibration.");
    return;
  }
  const float original_reading = reading_.temperature;
  xSemaphoreGive(reading_mutex_);
  const float delta = actual_temp - original_reading;
  debug_serial_.printf("SCD30: Temperature delta %f - %f = %f\n", actual_temp,
                       original_reading, delta);
  if (xSemaphoreTake(i2c_mutex_, 2000 / portTICK_PERIOD_MS) != pdPASS) {
    debug_serial_.println("SCD30: Couldn't acquire mutex for calibration.");
    return;
  }
  scd30_.sendCommand(COMMAND_SET_TEMPERATURE_OFFSET);
  uint8_t res[3];
  Wire.requestFrom(SCD30_ADDRESS, 3);
  const size_t bytes_read = Wire.readBytes(res, 3);
  if (bytes_read != 3) {
    debug_serial_.printf(
        "SCD30: Couldn't get temperature offset, read failed (%d != 3)\n",
        bytes_read);
    xSemaphoreGive(i2c_mutex_);
    return;
  }

  const uint8_t crc = scd30_.computeCRC8(res, 2);
  if (crc != res[2]) {
    debug_serial_.println("SCD30: temperature offset CRC failed.");
    xSemaphoreGive(i2c_mutex_);
    return;
  }

  const uint8_t original_offset_msb = res[0];
  const uint8_t original_offset_lsb = res[1];
  const uint16_t original_offset =
      ((uint16_t)original_offset_msb << 8 | original_offset_lsb);
  const int32_t new_offset = original_offset + (delta * -100.0f);
  const uint16_t final_new_offset =
      (new_offset < 0 || new_offset > 2000) ? 0 : new_offset;

  debug_serial_.printf(
      "SDC30: Original offset %d, new offset %d, final offset %d\n",
      original_offset, new_offset, final_new_offset);
  scd30_.sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, final_new_offset);
  xSemaphoreGive(i2c_mutex_);
}