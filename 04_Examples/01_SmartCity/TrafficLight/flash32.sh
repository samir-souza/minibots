PORT=/dev/cu.SLAB_USBtoUART
#PORT=/dev/cu.usbserial-A700abYF
#PORT=/dev/cu.wchusbserial1410
BAUD=115200
#BAUD=74880

esptool.py --chip esp32 --port $PORT --baud $BAUD --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 ESP32/bootloader.bin 0x10000 ESP32/NodeMCU.bin 0x8000 ESP32/partitions.bin

