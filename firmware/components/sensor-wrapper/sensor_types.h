#ifndef SENSOR_TYPES_H_
#define SENSOR_TYPES_H_

class TemperatureSensor {
 public:
  virtual float GetTemperature() = 0;
  virtual void CalibrateTemperature(float actual_temp) = 0;
};

class HumiditySensor {
 public:
  virtual float GetHumidity() = 0;
};

class PressureSensor {
 public:
  virtual float GetPressure() = 0;
};

#endif