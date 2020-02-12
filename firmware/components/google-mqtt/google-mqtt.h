#ifndef GOOGLE_MQTT_H_
#define GOOGLE_MQTT_H_

#include <vector>

#include "sensor_types.h"
#include "sensor_wrapper.h"

void SetupGoogleMQTT(std::vector<SensorWrapper *> sensors,
                     std::vector<TemperatureSensor *> temperature_sensors);
void LoopGoogleMQTT();
void GoogleMQTTWatchdog();

#endif