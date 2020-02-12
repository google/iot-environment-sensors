
file(GLOB_RECURSE GOOGLE_SRCS LIST_DIRECTORIES false "google-cloud-iot-arduino/src/*.cpp")
file(GLOB_RECURSE MQTT_SRCS LIST_DIRECTORIES false "mqtt/src/*.cpp" "mqtt/src/*.c")
set(COMPONENT_SRCS "google-mqtt.cc" "sensor.pb-c.c" ${GOOGLE_SRCS} ${MQTT_SRCS})
set(COMPONENT_ADD_INCLUDEDIRS "." "mqtt/src")
set(COMPONENT_REQUIRES "arduino" "protobuf-c" "sensor-wrapper" "esp_https_ota")
register_component()