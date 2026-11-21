LIBRARY := soft_i2c
BUILD_DIR ?= build
CC ?= cc
AR ?= ar
CFLAGS ?= -O2
WARNINGS := -std=c11 -Wall -Wextra -Wpedantic
INCLUDES := -Iinclude
SOURCE := src/$(LIBRARY).c
OBJECT := $(BUILD_DIR)/$(LIBRARY).o
ARCHIVE := $(BUILD_DIR)/lib$(LIBRARY).a
TEST := $(BUILD_DIR)/test_$(LIBRARY)
WIRINGPI_CFLAGS ?=
TEST_INPUTS := $(SOURCE) tests/test_soft_i2c.c tests/wiringPi.h include/soft_i2c.h

.PHONY: all test clean
all: $(ARCHIVE)

$(BUILD_DIR):
	mkdir -p "$@"

$(ARCHIVE): $(OBJECT)
	$(AR) rcs "$@" "$<"

$(OBJECT): $(SOURCE) include/soft_i2c.h | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(INCLUDES) $(WIRINGPI_CFLAGS) $(CFLAGS) $(WARNINGS) -c "$<" -o "$@"

$(TEST): $(TEST_INPUTS) | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(INCLUDES) -Itests $(CFLAGS) $(WARNINGS) $(LDFLAGS) $(filter %.c,$^) $(LDLIBS) -o "$@"

test: $(TEST)
	"$(TEST)"

clean:
	$(RM) "$(OBJECT)" "$(ARCHIVE)" "$(TEST)"
