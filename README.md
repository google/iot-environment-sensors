This project demonstrates how an ESP32 can be combined with Google Cloud IoT, Google Cloud Functions, Google StackDriver and Google BigQuery to collect and analyze metrics from environmental sensors.

**This example is not an officially supported Google product, does not have a SLA/SLO, and should not be used in production.**

The project consists of three components:

- The Google Cloud Function - This is a simple Go-based piece of code. Simply configure the Cloud Function in the console to be triggered on a telemetry event from the IoT project.
- The firmware - This is an ESP-IDF project and must be built using the ESP-IDF toolset.
- A CAD model for the PCB design in [this repository](https://github.com/bobobo1618/iot-environment-sensor).

# Instructions

There are three things that have to be set up:

- The hardware
- The firmware
- The Google Cloud Platform project which will collect metrics for analysis

## Hardware

This has been tested with the PCB design from [this repository](https://github.com/bobobo1618/iot-environment-sensor). The board is intended to be combined with:

- A Sensirion SCD30 CO2 sensor, with a pin-header soldered to it.
- An Amphenol SM-UART-04L particulate sensor.
- An ESP32-PICO-KIT.

The board can be manufactured manually or by a PCB fab.

This repository also contains a FreeCAD model for a case intended to fit the PCB design.

## Google Cloud Platform

The project is intended to be used with:

- IoT Core: To act as a bridge between the MQTT protocol used by the sensors and Pubsub.
- Pubsub: To act as a bridge between IoT Core and Cloud Functions.
- StackDriver: For real-time monitoring and alerting on sensor readings.
- BigQuery: For long-term historical data collection and analysis.
- Cloud Functions: To ingest incoming data and store it in BigQuery and/or StackDriver.

The recommended setup procedure is (using the [Cloud SDK](https://cloud.google.com/sdk) CLI):

- Create a GCP project ([docs](https://cloud.google.com/resource-manager/docs/creating-managing-projects)): `gcloud projects create <PROJECT_ID>`
- Create an IoT Core registry in the region of your choice ([docs](https://cloud.google.com/iot/docs/how-tos/devices)): `gcloud iot registries create <REGISTRY_ID> --project=<PROJECT_ID> --region=<REGION>`
- Create the partitioned BigQuery table where your historical data will be stored ([docs](https://cloud.google.com/bigquery/docs/creating-column-partitions)): `bq mk --table --schema bqschema.json --time_partitioning_field Timestamp <PROJECT_ID>:<DATASET>.<TABLE>`
- Set up the Cloud Functions ([docs](https://cloud.google.com/functions/docs/deploying/filesystem)):
    - BigQuery: 
      ```bash
      gcloud functions deploy <NAME> \
        --runtime go111 \
        --trigger-topic telemetry \
        --entry-point GasTranslator \
        --memory 128MB \
        --source cloud_function \
        --set-env-vars=PROJECT_ID=<PROJECT_ID>,BQ_DATASET=<BQ_DATASET>,BQ_TABLE=<BQ_TABLE>
      ```
    - StackDriver:
    ```bash
      gcloud functions deploy <NAME> \
        --runtime go111 \
        --trigger-topic telemetry \
        --entry-point GasTranslatorStackdriver \
        --memory 128MB \
        --source cloud_function \
        --set-env-vars=PROJECT_ID=<PROJECT_ID>,SD_NAMESPACE=<SD_NAMESPACE>,SD_LOCATION=<SD_LOCATION>,
      ```

## Firmware

The firmware is an [Espressif ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/) project and in particular uses [arduino-esp32](https://github.com/espressif/arduino-esp32). As such, it depends on version 3.3.1 of ESP-IDF which can be installed following [these instructions](https://docs.espressif.com/projects/esp-idf/en/v3.3.1/get-started/index.html). arduino-esp32 frequently updates its IDF version so check the git log for the most recent commit hash:

```bash
cd firmware/components/arduino
git log --oneline --grep 'IDF' firmware/components/arduino/
```

Once you've set up ESP-IDF, you'll need to configure the project by running `idf.py menuconfig` from the firmware directory. In particular, under `Component Config -> Google IoT Configuration` there are options for the IoT Core project and registry, as well as the WiFi AP and password.

The firmware is also built with support for OTA firmware updates and requires that you generate an OTA signing key. To do so:

- Generate a private key: `espsecure.py generate_signing_key secure_boot_signing_key.pem`
- Generate a public key: `espsecure.py extract_public_key --keyfile secure_boot_signing_key.pem signature_verification_key.bin`

The next step is to build the firmware with `idf.py build`.

Finally, you need to flash the device and provision if for IoT Core. The `provision.sh` script does both.

### OTAs

The firmware assumes that the firmware update will be stored using Google Cloud Storage. If you want to store it elsewhere, you'll need to replace `googleca.pem` with the public certificate of the HTTPS server you intend to update from.

In order to send an OTA update, you must:

- Build: `idf.py build`
- Sign: `espsecure.py sign_data --keyfile secure_boot_signing_key.pem build/hello-world.bin`
- Upload to your HTTPS server: `gsutil cp -a public-read build/hello-world.bin gs://<your bucket>/firmware.bin`
- Send the update command using IoT Core:
  ```
  gcloud iot devices commands send \
    --device=<device id> \
    --project=<project> \
    --region=<region> \
    --registry=<registry> \
    --command-data='update: https://storage.googleapis.com/<your bucket>/firmware.bin'
  ```
  