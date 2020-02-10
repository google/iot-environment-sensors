#ifndef SENSOR_UMUART04L_H_
#define SENSOR_UMUART04L_H_

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <HardwareSerial.h>

#include "sensor.pb-c.h"
#include "sensor_wrapper.h"

struct Smuart04lReading {
  uint16_t pm10_smoke;
  uint16_t pm25_smoke;
  uint16_t pm100_smoke;
  uint16_t pm10_env;
  uint16_t pm25_env;
  uint16_t pm100_env;
};

class Smuart04lSensor : public SensorWrapper {
 public:
  Smuart04lSensor(HardwareSerial* debug_serial, HardwareSerial* uart);
  void Init() override;
  void Loop() override;
  std::string GetSensorName() override;
  void UpdateSensor(SensorUpdate* update) override;

 private:
  Smuart04lReading reading_;
  uint64_t checksum_mismatches_;
  uint8_t status_;
  SemaphoreHandle_t reading_mutex_;
  HardwareSerial& debug_serial_;
  HardwareSerial& uart_;
};

#endif