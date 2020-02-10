#ifndef SENSOR_BMP280_H_
#define SENSOR_BMP280_H_
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "adafruit/Adafruit_BMP280.h"
#include "sensor.pb-c.h"
#include "sensor_types.h"
#include "sensor_wrapper.h"

struct Bmp280Reading {
  float temperature;
  float pressure;
};

class Bmp280Sensor : public SensorWrapper,
                     public PressureSensor,
                     public TemperatureSensor {
 public:
  Bmp280Sensor(HardwareSerial* debug_serial, SemaphoreHandle_t* i2c_mutex);
  void Init() override;
  void Loop() override;
  virtual std::string GetSensorName() override { return "bmp280"; }

  float GetPressure() override {
    xSemaphoreTake(reading_mutex_, portMAX_DELAY);
    const float pressure = reading_.pressure / 100.0f;
    xSemaphoreGive(reading_mutex_);
    return pressure;
  }

  float GetTemperature() override {
    xSemaphoreTake(reading_mutex_, portMAX_DELAY);
    const float temperature = reading_.temperature + temperature_offset_;
    xSemaphoreGive(reading_mutex_);
    return temperature;
  }

  void CalibrateTemperature(float actual_temp) override;

  void UpdateSensor(SensorUpdate* update) override {
    Serial.println("Updating BMP280 proto.");
    xSemaphoreTake(reading_mutex_, portMAX_DELAY);
    update->bmp280->temperature = reading_.temperature + temperature_offset_;
    update->bmp280->has_temperature = true;
    update->bmp280->pressure = reading_.pressure;
    update->bmp280->has_pressure = true;
    xSemaphoreGive(reading_mutex_);
  };

 private:
  Bmp280Reading reading_;
  HardwareSerial& debug_serial_;
  SemaphoreHandle_t reading_mutex_;
  SemaphoreHandle_t& i2c_mutex_;
  float temperature_offset_;
  Adafruit_BMP280 bmp280_;
};

#endif