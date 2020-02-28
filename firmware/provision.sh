#!/bin/bash
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

rm -Rf spiffs/
mkdir spiffs
esptool.py -p /dev/ttyUSB0 read_mac | grep MAC: | tail -n1 | sed 's/MAC: //;s/://g' > spiffs/devid.dat
/usr/bin/python makemac.py
DEVID=$(cat spiffs/devid.dat)
KEYDIR="keys/${DEVID}"
PRIVKEY="${KEYDIR}/ec_private.pem"
PUBKEY="${KEYDIR}/ec_public.pem"

if ! test -e "${PRIVKEY}"; then
    mkdir -p "${KEYDIR}"
    openssl ecparam -genkey -name prime256v1 -noout -out "${PRIVKEY}"
    openssl ec -in "${PRIVKEY}" -pubout -out "${PUBKEY}"
fi

openssl ec -in "${PRIVKEY}" -text -noout | grep -A3 priv: | grep -v priv: | sed -E 's/\s+//' | tr -d '\n' > spiffs/ec_private.dat

if ! gcloud iot devices describe ${DEVID} --project="${PROJECT_ID}" --region="${DEVICE_REGION}" --registry="${DEVICE_REGISTRY}"; then
    gcloud iot devices create ${DEVID} --project="${PROJECT_ID}" --region="${DEVICE_REGION}" --registry="${DEVICE_REGISTRY}" --public-key path="${PUBKEY}",type=es256-pem
fi

spiffsgen.py 983040 spiffs/ spiffs.bin
esptool.py -p /dev/ttyUSB0 -b 460800 write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0xd000 build/ota_data_initial.bin 0x10000 build/hello-world.bin 0x310000 spiffs.bin
