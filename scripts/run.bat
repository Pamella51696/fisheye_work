@echo off
setlocal
cd /d "%~dp0\.."

if not defined OPENCV_DIR (
  if exist "C:\Users\ps95973\Downloads\opencv\build\java\opencv-490.jar" (
    set "OPENCV_DIR=C:\Users\ps95973\Downloads\opencv\build"
  ) else (
    set "OPENCV_DIR=C:\opencv\build"
  )
)

set "OPENCV_JAR=%OPENCV_DIR%\java\opencv-490.jar"
if not exist "out\VideoStreamingServer.class" (
  echo ERROR: out\ not built. Run scripts\compile.bat first.
  exit /b 1
)
if not exist "%OPENCV_JAR%" (
  echo ERROR: Missing %OPENCV_JAR% — set OPENCV_DIR.
  exit /b 1
)

set "PATH=%OPENCV_DIR%\bin;%PATH%"
set "JAVA_LIB=%OPENCV_DIR%\java\x64;%OPENCV_DIR%\bin"

echo Starting VideoStreamingServer (port 9090)...
java -cp "out;%OPENCV_JAR%" -Djava.library.path="%JAVA_LIB%" VideoStreamingServer %*
