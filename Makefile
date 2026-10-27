.PHONY: all build test validate-cmocka validate-plot validate-all clean arduino-build

CC ?= gcc
PYTHON ?= python3
ROOT := $(CURDIR)
BUILD_DIR := $(ROOT)/build
BIN_DIR := $(BUILD_DIR)/bin

LIB_OUT := $(BUILD_DIR)/wmirror_logic.so

CMOCKA_ROOT ?= /Users/admin/Tools/cmocka-2.0.2/build/install
IFLAGS := -I$(ROOT)
LDFLAGS := -lm -lcmocka

ifneq ($(wildcard $(CMOCKA_ROOT)/include),)
    IFLAGS += -I$(CMOCKA_ROOT)/include
endif
ifneq ($(wildcard $(CMOCKA_ROOT)/lib),)
    LDFLAGS := -L$(CMOCKA_ROOT)/lib $(LDFLAGS)
endif

CFLAGS ?= -O2 -Wall -Wextra

all: build test

build: $(LIB_OUT)

$(LIB_OUT): wmirror_logic.c wmirror_logic.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -shared -fPIC $(IFLAGS) -o $@ wmirror_logic.c -lm
	@echo "Built native logic lib: $(LIB_OUT)"

$(BIN_DIR)/test_basic: tests/test_basic.c
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) $(IFLAGS) -o $@ $< $(LDFLAGS)

$(BIN_DIR)/test_wmirror_logic: tests/test_wmirror_logic.c wmirror_logic.c wmirror_logic.h
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) $(IFLAGS) -o $@ tests/test_wmirror_logic.c wmirror_logic.c $(LDFLAGS)

test: validate-cmocka

validate-cmocka: $(BIN_DIR)/test_basic $(BIN_DIR)/test_wmirror_logic
	@echo "Running tests..."
	$(BIN_DIR)/test_basic
	$(BIN_DIR)/test_wmirror_logic
	@echo "All CMocka tests PASSED successfully."

validate-plot:
	$(PYTHON) plot_norm.py
	@echo "Plot validation OK"

validate-all: test validate-plot

arduino-build:
	@echo "Arduino firmware build must be run from Arduino IDE or PlatformIO."

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned generated build artifacts"
