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
py -m esptool --chip esp32 erase_flash
goto :end

:selectedUpload
echo USING MANUAL COM
py -m esptool --chip esp32 --port "%_selectedPort%" erase_flash

:end
timeout 5