# surface-touch userspace processor — heatd (touchpad)
#
# heatd reads the pad's vendor heat-container stream from the intel_quickspi
# hidraw node and exposes a standard PTP touchpad over uhid.  See
# ../README.md for the product view.

CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude -Ibuild

# OpenCV (the segmentation-stage algorithms: cv::connectedComponents,
# cv::moments).  Only core+imgproc+geometry are linked — never another
# module.  Headers arrive via -isystem so OpenCV's own headers never fire
# our -Wall -Wextra -Wpedantic.
# Arch package: opencv (5.x, pkg-config name opencv5).  cv::moments lives in
# geometry in OpenCV 5; OpenCV 4 is not supported by this build.
OPENCV_PC := $(shell pkg-config --exists opencv5 2>/dev/null && echo opencv5)
ifeq ($(filter clean check,$(MAKECMDGOALS)),)
ifeq ($(OPENCV_PC),)
$(error missing dependency: OpenCV 5 — install the 'opencv' package (sudo pacman -S opencv) or your distribution's OpenCV 5 development package)
endif
endif
OPENCV_CFLAGS := $(if $(OPENCV_PC),$(shell pkg-config --cflags $(OPENCV_PC) 2>/dev/null | sed 's/-I/-isystem /g'),)
CPPFLAGS  += $(OPENCV_CFLAGS)
OPENCV_LIBS := -lopencv_core -lopencv_imgproc -lopencv_geometry

SRC_DIR  := src
BUILD    := build

DAEMON_SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ        := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD)/%.o,$(DAEMON_SRC))

GEN := $(BUILD)/ptp_collection.hpp
BIN2HPP := tools/bin2hpp.py

all: heatd

# Generated header: the uhid front descriptor is the committed fixture, not
# a hand copy (fixtures/ptp_collection_000D_0005.bin is a byte-exact slice
# of the device's own report descriptor).
$(GEN): fixtures/ptp_collection_000D_0005.bin $(BIN2HPP)
	@mkdir -p $(BUILD)
	python3 $(BIN2HPP) $< $@ ptp collection

$(BUILD)/%.o: $(SRC_DIR)/%.cpp $(GEN) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

heatd: $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV_LIBS)

clean:
	rm -rf $(BUILD) heatd digid

.PHONY: all clean
