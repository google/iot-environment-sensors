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