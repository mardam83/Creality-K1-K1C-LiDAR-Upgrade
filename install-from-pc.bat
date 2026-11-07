@echo off
rem install-from-pc.bat - install or update the K1 LiDAR toolkit on the printer from
rem Windows 10/11 (uses the ssh and tar built into Windows). Double-click it, or:
rem
rem   install-from-pc.bat PRINTER_IP [install options]
rem   first install:  install-from-pc.bat 192.168.1.xx --with-hook --lang it
rem   update:         install-from-pc.bat 192.168.1.xx
rem
rem Options: --with-hook (LiDAR features at every print start), --no-hook,
rem --lang en|it, --force. An update keeps hook, language, settings and calibrations.
rem The printer must be idle. The root password is asked once.
setlocal
cd /d "%~dp0"

set "HOST="
set "OPTS="
for /f "tokens=1,*" %%a in ("%*") do (
    set "HOST=%%a"
    set "OPTS=%%b"
)
if "%HOST%"=="" (
    echo K1 LiDAR toolkit - install / update
    echo.
    set /p "HOST=Printer IP address (e.g. 192.168.1.xx): "
)
if "%HOST%"=="" goto :usage
if "%HOST%"=="-h" goto :usage
if "%HOST%"=="--help" goto :usage
if "%~1"=="" (
    echo.
    echo Options: first install = --with-hook --lang it    update = just press Enter
    set /p "OPTS=Options: "
)

where ssh >nul 2>&1 || (echo ssh not found: Settings - Apps - Optional features - OpenSSH Client & goto :fail)
where tar >nul 2>&1 || (echo tar not found: Windows 10 1803 or newer is needed & goto :fail)
if not exist MANIFEST (echo MANIFEST missing: run me from the toolkit folder & goto :fail)

rem MANIFEST = list of the files that go to the printer
set "PKG=%TEMP%\k1lidar_pkg.tgz"
tar -czf "%PKG%" --exclude=__pycache__ --exclude=*.pyc -T MANIFEST
if errorlevel 1 (echo could not build the package & goto :fail)

set "STAGE=/usr/data/k1-lidar.new"
echo.
echo == K1 LiDAR toolkit -^> root@%HOST% ==
echo (the root password is asked once)
ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 root@%HOST% "rm -rf %STAGE% && mkdir -p %STAGE% && tar xzf - -C %STAGE% && sh %STAGE%/update.sh --apply %STAGE% %OPTS%" < "%PKG%"
set "RC=%errorlevel%"
del "%PKG%" >nul 2>&1
echo.
if not "%RC%"=="0" goto :fail
echo Done.
if "%~1"=="" pause
exit /b 0

:usage
echo usage: install-from-pc.bat PRINTER_IP [--with-hook] [--lang it]
if "%~1"=="" pause
exit /b 1

:fail
echo FAILED (see the messages above).
if "%~1"=="" pause
exit /b 1
