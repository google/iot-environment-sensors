set(COMPONENT_SRCS "main.cc")
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_REQUIRES "arduino" "sensor-wrapper" "sensor-bmp280" "sensor-scd30" "sensor-smuart04l" "sensor-sgp30" "google-mqtt")
register_component()
