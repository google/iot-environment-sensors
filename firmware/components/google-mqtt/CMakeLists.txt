# Copyright 2020 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

file(GLOB_RECURSE GOOGLE_SRCS LIST_DIRECTORIES false "google-cloud-iot-arduino/src/*.cpp")
file(GLOB_RECURSE MQTT_SRCS LIST_DIRECTORIES false "mqtt/src/*.cpp" "mqtt/src/*.c")
set(COMPONENT_SRCS "google-mqtt.cc" "sensor.pb-c.c" ${GOOGLE_SRCS} ${MQTT_SRCS})
set(COMPONENT_ADD_INCLUDEDIRS "." "mqtt/src")
set(COMPONENT_REQUIRES "arduino" "protobuf-c" "sensor-wrapper" "esp_https_ota")
register_component()