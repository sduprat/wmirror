.PHONY: build validate-cmocka validate-plot validate-all clean arduino-build

CMAKE_BIN := /Users/admin/Tools/cmake-3.30.0-macos-universal/CMake.app/Contents/bin
CMAKE := $(CMAKE_BIN)/cmake
CTEST := $(CMAKE_BIN)/ctest
PYTHON := python3
ROOT := $(CURDIR)
BUILD_DIR := $(ROOT)/build
LIB_OUT := $(BUILD_DIR)/wmirror_logic.so
TEST_DIR := $(ROOT)/tests/cmocka_wmirror
TEST_BUILD_DIR := $(TEST_DIR)/build
PLOT_SCRIPT := $(ROOT)/plot_norm.py

export PATH := $(CMAKE_BIN):$(PATH)

build: $(LIB_OUT)

$(LIB_OUT): $(ROOT)/wmirror_logic.c $(ROOT)/wmirror_logic.h
	mkdir -p $(BUILD_DIR)
	gcc -shared -fPIC -O2 -I$(ROOT) -o $(LIB_OUT) $(ROOT)/wmirror_logic.c -lm
	echo "Built native logic lib: $(LIB_OUT)"

validate-cmocka:
	$(CMAKE) -S $(TEST_DIR) -B $(TEST_BUILD_DIR)
	$(CMAKE) --build $(TEST_BUILD_DIR)
	$(CTEST) --test-dir $(TEST_BUILD_DIR) --output-on-failure
	echo "CMocka validation OK"

validate-plot:
	$(PYTHON) $(PLOT_SCRIPT)
	echo "Plot validation OK"

validate-all: validate-cmocka validate-plot

arduino-build:
	echo "Arduino firmware build must be run from the IDE (Arduino: Build / PlatformIO build) because the hardware toolchain is not available in this host shell."

clean:
	rm -rf $(BUILD_DIR) $(TEST_BUILD_DIR)
	echo "Cleaned generated build artifacts"
