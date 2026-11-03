@echo off
setlocal
cd /d "%~dp0"
if errorlevel 1 goto :error

call "start_receiver.bat" --transport udp %*
if errorlevel 1 goto :error
endlocal
exit /b 0

:error
echo [error] UDP receiver failed. Review the message above.
pause
endlocal
exit /b 1
