@echo off
SETLOCAL EnableDelayedExpansion

echo       Select a COM port or press enter to auto-detect
echo		write the full name of the COM port!


reg query HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM | find "REG_SZ" > %temp%\comlist-temp.txt

echo.

echo. Active COM Ports:

for /f "tokens=2,4* delims=\= " %%a in (%temp%\comlist-temp.txt) do echo.                            %%b   (%%a)

set /p _selectedPort=Select a COM port or leave empty to use auto-detect:
echo "%_selectedPort%"

IF "%_selectedPort%"=="" GOTO :autoUpload
GOTO :selectedUpload

:autoUpload
echo USING AUTO DETECT
py -m esptool --chip esp32 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 ota_data_initial.bin 0x10000 firmware.bin  
goto :end

:selectedUpload
echo USING MANUAL COM
py -m esptool --chip esp32 --port "%_selectedPort%" --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 ota_data_initial.bin 0x10000 firmware.bin

:end
timeout 5