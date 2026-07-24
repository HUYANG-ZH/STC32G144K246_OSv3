@echo off
setlocal
cd /d "%~dp0"
if "%~1"=="" (
    echo Usage: flash_cli_example.bat "D:\path\to\SEEKFREE.hex" [COM7]
    echo This script only checks the HEX and does not write Flash.
    exit /b 2
)
python -m stcfastboot plan "%~1"
endlocal
pause
