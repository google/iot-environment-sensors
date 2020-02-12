import sys

with open('spiffs/devid.dat', 'r') as infile, open('spiffs/mac.dat', 'wb') as outfile:
    devid = infile.read()
    if len(devid) < 12:
        sys.exit(1)
    outfile.write(bytes(bytearray.fromhex(devid[:12])))

