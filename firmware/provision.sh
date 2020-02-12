#!/bin/bash
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
esptool.py --chip esp32 -p /dev/ttyUSB0 -b 115200 write_flash -z 0x310000 spiffs.bin
