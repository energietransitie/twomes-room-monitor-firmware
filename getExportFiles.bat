@echo off
echo copying needed files to /binaries

mkdir .\binaries

copy ".\.pio\build\esp32dev\bootloader.bin" ".\binaries"
copy ".\.pio\build\esp32dev\partitions.bin" ".\binaries"
copy ".\.pio\build\esp32dev\firmware.bin" ".\binaries"
copy ".\.pio\build\esp32dev\ota_data_initial.bin" ".\binaries"

timeout 5