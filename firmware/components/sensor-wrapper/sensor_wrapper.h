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

#ifndef SENSOR_WRAPPER_H_
#define SENSOR_WRAPPER_H_

#include <cinttypes>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "sensor.pb-c.h"

struct SensorMetric {
  std::string metric_name;
  std::string value;
  std::vector<std::string> attributes;
};

using SensorMetricList = std::vector<SensorMetric>;

class SensorWrapper {
 public:
  virtual void Init() = 0;
  virtual void Loop() = 0;
  virtual std::string GetSensorName() = 0;
  virtual void UpdateSensor(SensorUpdate* update) { return; };
};

#endif