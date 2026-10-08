@echo off
REM Windows entry point for the Docker-based build (see build.ps1 for details).
REM Usage: build.bat <build|run|test|deploy|shell|images|clean> [args...]
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*
exit /b %ERRORLEVEL%
