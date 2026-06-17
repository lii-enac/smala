UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
  CXX ?= clang++
else
  $(error cppautogui currently supports macOS/Darwin only)
endif

BUILD_DIR ?= build
BIN_DIR ?= bin

CPPFLAGS += -Iinclude
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic -Wno-deprecated-declarations
LDFLAGS += -framework ApplicationServices -framework CoreFoundation -framework ImageIO

LIB_OBJ := $(BUILD_DIR)/src/cppautogui_macos.o

RECORD_OBJ := $(BUILD_DIR)/src/record.o
REPLAY_OBJ := $(BUILD_DIR)/src/replay.o
LAUNCH_OBJ := $(BUILD_DIR)/src/launch_tests.o

BINS := $(BIN_DIR)/record_cppautogui \
        $(BIN_DIR)/replay_cppautogui \
        $(BIN_DIR)/launch_tests_cppautogui

.PHONY: all clean dirs

all: $(BINS)

dirs:
	@mkdir -p $(BUILD_DIR)/src $(BIN_DIR) datas references results/diffs new_records

$(BUILD_DIR)/src/%.o: src/%.cpp include/cppautogui.hpp | dirs
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/src/%.o: src/%.mm include/cppautogui.hpp | dirs
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -x objective-c++ -c $< -o $@

$(BIN_DIR)/record_cppautogui: $(LIB_OBJ) $(RECORD_OBJ) | dirs
	$(CXX) $^ $(LDFLAGS) -o $@

$(BIN_DIR)/replay_cppautogui: $(LIB_OBJ) $(REPLAY_OBJ) | dirs
	$(CXX) $^ $(LDFLAGS) -o $@

$(BIN_DIR)/launch_tests_cppautogui: $(LIB_OBJ) $(LAUNCH_OBJ) | dirs
	$(CXX) $^ $(LDFLAGS) -o $@

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)
