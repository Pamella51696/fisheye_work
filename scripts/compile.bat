@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0\.."

if not defined OPENCV_DIR (
  if exist "C:\Users\ps95973\Downloads\opencv\build\java\opencv-490.jar" (
    set "OPENCV_DIR=C:\Users\ps95973\Downloads\opencv\build"
  ) else (
    set "OPENCV_DIR=C:\opencv\build"
  )
)

set "OPENCV_JAR=%OPENCV_DIR%\java\opencv-490.jar"
if not exist "%OPENCV_JAR%" (
  echo ERROR: OpenCV Java jar not found:
  echo   %OPENCV_JAR%
  echo Set OPENCV_DIR to your OpenCV build folder, e.g.:
  echo   set OPENCV_DIR=C:\Users\ps95973\Downloads\opencv\build
  exit /b 1
)

if exist out rmdir /s /q out
mkdir out

echo Compiling all sources with OpenCV jar...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$root = (Get-Location).Path; " ^
  "$src = Get-ChildItem -Path $root -Recurse -Filter *.java | Where-Object { $_.FullName -notmatch '\\out\\' } | ForEach-Object { $_.FullName }; " ^
  "if ($src.Count -eq 0) { throw 'No .java files found' }; " ^
  "& javac -encoding UTF-8 -cp '%OPENCV_JAR%' -d out $src"
if errorlevel 1 exit /b 1

echo.
echo OK: classes in %CD%\out
echo Run: scripts\run.bat
exit /b 0
