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

#include "sensor_smuart04l.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>

Smuart04lSensor::Smuart04lSensor(HardwareSerial* debug_serial,
                                 HardwareSerial* uart)
    : reading_{},
      checksum_mismatches_(0),
      status_(0),
      debug_serial_(*debug_serial),
      uart_(*uart) {
  reading_mutex_ = xSemaphoreCreateMutex();
}

void Smuart04lSensor::Init() {
  uart_.begin(9600);
  debug_serial_.println("SM-UART-04L init complete.");
}

void Smuart04lSensor::Loop() {
  while (1) {
    static uint8_t buf[32];
    while (uart_.available() > 32) {
      uart_.readBytes(buf, 32);
      uint16_t calculated_checksum = 0;
      for (int i = 0; i < 32; i++) {
        if (i < 30) {
          calculated_checksum += buf[i];
        }
      }
      uint16_t read_checksum = *(uint16_t*)(buf + 30);
      read_checksum = (read_checksum << 8) | (read_checksum >> 8);

      if (calculated_checksum != read_checksum) {
        Serial.printf("SM-UART-04L: Checksum mismatch, skipping.");
        Serial.printf(" expected checksum: %u, actual checksum: %u\n",
                      read_checksum, calculated_checksum);
        checksum_mismatches_++;
        continue;
      }
      memcpy(&reading_, buf + 4, 12);
      for (int i = 0; i < 6; i++) {
        uint16_t* ptr = &reading_.pm10_smoke + i;
        *ptr = (*ptr >> 8) | (*ptr << 8);
      }
      status_ = buf[30];
      Serial.printf("SM-UART-04L: %u, %u, %u, %u, %u, %u\n",
                    reading_.pm10_smoke, reading_.pm25_smoke,
                    reading_.pm100_smoke, reading_.pm10_env, reading_.pm25_env,
                    reading_.pm100_env);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void Smuart04lSensor::UpdateSensor(SensorUpdate* update) {
  xSemaphoreTake(reading_mutex_, portMAX_DELAY);
  update->smuart04l->pm10_smoke = reading_.pm10_smoke;
  update->smuart04l->has_pm10_smoke = true;
  update->smuart04l->pm25_smoke = reading_.pm25_smoke;
  update->smuart04l->has_pm25_smoke = true;
  update->smuart04l->pm100_smoke = reading_.pm100_smoke;
  update->smuart04l->has_pm100_smoke = true;
  update->smuart04l->pm10_env = reading_.pm10_env;
  update->smuart04l->has_pm10_env = true;
  update->smuart04l->pm25_env = reading_.pm25_env;
  update->smuart04l->has_pm25_env = true;
  update->smuart04l->pm100_env = reading_.pm100_env;
  update->smuart04l->has_pm100_env = true;
  xSemaphoreGive(reading_mutex_);
};

std::string Smuart04lSensor::GetSensorName() { return "smuart04l"; }
