#ifndef SENSOR_SCD30_H_
#define SENSOR_SCD30_H_

#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "sensor.pb-c.h"
#include "sensor_types.h"
#include "sensor_wrapper.h"
#include "sparkfun/src/SparkFun_SCD30_Arduino_Library.h"

struct Scd30Reading {
  float temperature;
  float humidity;
  float co2;
};

class Scd30Sensor : public SensorWrapper,
                    public TemperatureSensor,
                    public HumiditySensor {
 public:
  Scd30Sensor(HardwareSerial* debug_serial, SemaphoreHandle_t* i2c_mutex,
              PressureSensor* pressure_sensor = nullptr);
  void Init() override;
  void Loop() override;
  virtual std::string GetSensorName() override { return "scd30"; }

  float GetTemperature() override {
    xSemaphoreTake(reading_mutex_, portMAX_DELAY);
    const float temperature = reading_.temperature;
    xSemaphoreGive(reading_mutex_);
    return temperature;
  }

  float GetHumidity() override {
    xSemaphoreTake(reading_mutex_, portMAX_DELAY);
    const float humidity = reading_.humidity;
    xSemaphoreGive(reading_mutex_);
    return humidity;
  }

  void CalibrateTemperature(float actual_temp) override;

  void UpdateSensor(SensorUpdate* update) override {
    xSemaphoreTake(reading_mutex_, portMAX_DELAY);
    update->scd30->temperature = reading_.temperature;
    update->scd30->has_temperature = true;
    update->scd30->humidity = reading_.humidity;
    update->scd30->has_humidity = true;
    update->scd30->co2 = reading_.co2;
    update->scd30->has_co2 = true;
    xSemaphoreGive(reading_mutex_);
  };

 private:
  Scd30Reading reading_;
  uint64_t failed_readings_;
  HardwareSerial& debug_serial_;
  SemaphoreHandle_t reading_mutex_;
  SemaphoreHandle_t& i2c_mutex_;
  PressureSensor* pressure_sensor_;
  SCD30 scd30_;
};

#endif