set(COMPONENT_SRCS "sensor_bmp280.cc" "adafruit/Adafruit_BMP280.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_REQUIRES "arduino" "sensor-wrapper")
register_component()
