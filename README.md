This project demonstrates how an ESP32 can be combined with Google Cloud IoT, Google Cloud Functions, Google StackDriver and Google BigQuery to collect and analyze metrics from environmental sensors.

**This example is not an officially supported Google product, does not have a SLA/SLO, and should not be used in production.**

The project consists of two components:

- The Google Cloud Function - This is a simple Go-based piece of code. Simply configure the Cloud Function in the console to be triggered on a telemetry event from the IoT project.
- The firmware - This is an ESP-IDF project and must be built using the ESP-IDF toolset.

