CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wpedantic
INCLUDES := -I/usr/include/eigen3 -Iinclude
BUILD    := build

SRC := src/icp_scan_matcher.cpp src/main.cpp
OBJ := $(patsubst src/%.cpp,$(BUILD)/%.o,$(SRC))
BIN := $(BUILD)/slam_demo

.PHONY: all clean run grid

all: $(BIN)

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BIN): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $@

run: $(BIN)
	./$(BIN) data/rplidar_scan_sample.csv

grid:
	python scripts/occupancy_grid.py data/rplidar_scan_sample.csv --out build/map.pgm

clean:
	rm -rf $(BUILD)
