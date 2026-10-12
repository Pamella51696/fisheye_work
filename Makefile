# fisheye_work — WP5 + surround integration
#
# Prefer Maven (downloads OpenCV Java jar):  make mvn-compile
# Or local OpenCV install:                 make compile  (needs OPENCV_JAR)

SHELL := /bin/bash
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

MAIN_WP5     := com.middleware.panorama.VideoStreamingServer
MAIN_KANNALA := KannalaSurroundServer
PORT         ?= 9090

# Override for your machine (Windows: use scripts\compile.bat or set in Git Bash)
OPENCV_JAR ?= $(OPENCV_DIR)/java/opencv-490.jar
OPENCV_LIB ?= $(OPENCV_DIR)/java/x64

OUT := $(ROOT)/out
CP  := $(OUT):$(OPENCV_JAR)

JAVA_RUN := java --enable-native-access=ALL-UNNAMED \
	-Djava.library.path="$(OPENCV_LIB)" \
	-cp "$(CP)"

.PHONY: all compile clean run run-wp5 run-kannala mvn-compile mvn-test mvn-run mvn-run-kannala help

help:
	@echo "Targets:"
	@echo "  mvn-compile     mvn -q -DskipTests compile   (recommended; uses pom.xml)"
	@echo "  mvn-test        mvn -q test"
	@echo "  mvn-run         run WP5 server (port $(PORT))"
	@echo "  mvn-run-kannala run KannalaSurroundServer"
	@echo "  compile         scripts/compile.sh (requires OPENCV_JAR)"
	@echo "  run / run-wp5   WP5 viewer entry ($(MAIN_WP5))"
	@echo "  run-kannala     $(MAIN_KANNALA)"
	@echo "  clean           remove out/ and mvn target/"

all: mvn-compile

mvn-compile:
	cd "$(ROOT)" && mvn -q -DskipTests compile

mvn-test:
	cd "$(ROOT)" && mvn -q test

mvn-run:
	cd "$(ROOT)" && mvn -q exec:java -Dexec.mainClass="$(MAIN_WP5)" -Dexec.args="$(PORT)"

mvn-run-kannala:
	cd "$(ROOT)" && mvn -q exec:java -Dexec.mainClass="$(MAIN_KANNALA)" -Dexec.args="$(PORT)"

compile:
	@test -n "$(OPENCV_JAR)" || (echo "Set OPENCV_JAR or OPENCV_DIR"; exit 1)
	cd "$(ROOT)" && OPENCV_JAR="$(OPENCV_JAR)" ./scripts/compile.sh

run run-wp5: compile
	$(JAVA_RUN) $(MAIN_WP5) $(PORT)

run-kannala: compile
	$(JAVA_RUN) $(MAIN_KANNALA) $(PORT)

clean:
	rm -rf "$(OUT)" "$(ROOT)/target"
	@find "$(ROOT)" -name '*.class' -not -path '*/target/*' -delete 2>/dev/null || true
