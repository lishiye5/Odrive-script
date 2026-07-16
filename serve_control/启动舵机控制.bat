@echo off
chcp 65001 >nul
cd /d "%~dp0"
start "" pythonw.exe "%~dp0mavlink_servo_control.py"
