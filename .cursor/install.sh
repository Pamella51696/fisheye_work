#!/usr/bin/env bash
set -euo pipefail

cd /workspace

if ! command -v javac >/dev/null; then
  sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq openjdk-21-jdk
fi

if ! command -v ffmpeg >/dev/null; then
  sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ffmpeg
fi

if ! dpkg -s libopencv-java >/dev/null 2>&1; then
  sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq libopencv-dev libopencv-java
fi

mkdir -p build
javac -d build -cp /usr/share/java/opencv.jar VideoStreamingServer.java
