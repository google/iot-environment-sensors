/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SENSOR_SGP30_H_
#define SENSOR_SGP30_H_
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "sensor.pb-c.h"
#include "sensor_types.h"
#include "sensor_wrapper.h"
#include "sparkfun/src/SparkFun_SGP30_Arduino_Library.h"

struct Sgp30Reading {
  uint16_t co2;
  uint16_t tvoc;
  uint16_t baseline_co2;
  uint16_t baseline_tvoc;
  uint16_t h2;
  uint16_t ethanol;
};

class Sgp30Sensor : public SensorWrapper {
 public:
  Sgp30Sensor(HardwareSerial* debug_serial, SemaphoreHandle_t* i2c_mutex,
              TemperatureSensor* temperature_sensor = nullptr,
              HumiditySensor* humidity_sensor = nullptr);
  void Init() override;
  void Loop() override;
  virtual std::string GetSensorName() override { return "sgp30"; }
  void UpdateSensor(SensorUpdate* update) override;

 private:
  uint16_t AbsHumidityForSensor(float relative_humidity, float temperature);
  Sgp30Reading reading_;
  uint64_t failed_readings_;
  SGP30 sgp30_;
  HardwareSerial& debug_serial_;
  SemaphoreHandle_t& i2c_mutex_;
  SemaphoreHandle_t reading_mutex_;
  TemperatureSensor* temperature_sensor_;
  HumiditySensor* humidity_sensor_;
  uint16_t abs_humidity_hardware_format_;
  float abs_humidity_;
};

#endif