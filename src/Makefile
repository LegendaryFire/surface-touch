# surface-touch userspace processors — heatd (touchpad) and digid (digitizer).
#
# heatd reads the pad's vendor heat-container stream from the intel_quickspi
# hidraw node and exposes a standard PTP touchpad over uhid.  digid arms
# the digitizer port and exposes the touchscreen (MT) and pen as uhid
# devices.  See ../README.md for the product view and ../docs/DEVELOP.md for
# the engineering background.
#
# `all` builds every shipped binary.

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

DIGI_SRC   := $(wildcard $(SRC_DIR)/digi/*.cpp)
DIGI_OBJ   := $(patsubst $(SRC_DIR)/digi/%.cpp,$(BUILD)/digi/%.o,$(DIGI_SRC))

GEN := $(BUILD)/ptp_collection.hpp
BIN2HPP := tools/bin2hpp.py

all: heatd digid

# Generated header: the uhid front descriptor is the committed fixture, not
# a hand copy (fixtures/ptp_collection_000D_0005.bin is a byte-exact slice
# of the device's own report descriptor).
$(GEN): fixtures/ptp_collection_000D_0005.bin $(BIN2HPP)
	@mkdir -p $(BUILD)
	python3 $(BIN2HPP) $< $@ ptp collection

$(BUILD)/%.o: $(SRC_DIR)/%.cpp $(GEN) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/digi/%.o: $(SRC_DIR)/digi/%.cpp | $(BUILD)
	@mkdir -p $(BUILD)/digi
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

heatd: $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV_LIBS)

# Every header src/digi/*.cpp includes is listed: an edited header must
# never leave digid on a pre-edit object.
DIGI_DEPS := include/digi_frame.hpp include/digi_pipeline.hpp \
             include/digi_uhid.hpp include/pen_frame.hpp include/pen_feed.hpp \
             include/pen_uhid.hpp include/pen_pressure.hpp \
             include/pen_position.hpp \
             include/touch_tracker.hpp
$(BUILD)/digi/digid.o: $(DIGI_DEPS)
$(BUILD)/digi/digi_pipeline.o: $(DIGI_DEPS)
$(BUILD)/digi/pen_position.o: $(DIGI_DEPS)

digid: $(DIGI_OBJ) $(BUILD)/touch_tracker.o | identity
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV_LIBS)

# heatd's engine sources are byte-pinned against the hardware-validated
# revision; the digitizer path shares touch_tracker with it.
identity:
	@sha256sum -c heatd-sources.sha256 --quiet && echo "identity: heatd sources byte-identical"

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD) heatd digid

.PHONY: all clean identity
