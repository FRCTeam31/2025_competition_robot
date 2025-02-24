@echo off

:: Default values
set "runAdvantageScope = false"
set "runElastic = false"

:: Parse arguments
:parseArgs
if "%~1" == "" goto final
if "%~1" == "-a" set "runAdvantageScope=true"
if "%~1" == "-e" set "runElastic=true"

shift
goto parseArgs

:: Run tools based on arguments
:final
if "%runAdvantageScope%" == "true" start "" "C:\Users\Public\wpilib\2025\advantagescope\AdvantageScope (WPILib).exe"
if "%runElastic%" == "true" start "" "C:\Users\Public\wpilib\2025\elastic\elastic_dashboard.exe"

./gradlew simulateJava