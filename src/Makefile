# SPDX-License-Identifier: MIT
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
CPPFLAGS += -Iinclude

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

BUILD    := build

# Each binary names its sources; touch_tracker is shared by both.
HEATD_SRC    := heatd.cpp heat_engine.cpp touch_tracker.cpp
DIGID_SRC    := digid.cpp digi_pipeline.cpp pen_position.cpp touch_tracker.cpp

HEATD_OBJ    := $(HEATD_SRC:%.cpp=$(BUILD)/%.o)
DIGIPROC_OBJ := $(DIGID_SRC:%.cpp=$(BUILD)/%.o)

all: heatd digid

# -MMD -MP writes a .d file beside each object listing the headers it
# included, so an edited header always rebuilds the objects that use it.
$(BUILD)/%.o: %.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

-include $(wildcard $(BUILD)/*.d)

# The pad front's descriptor is the committed fixture, embedded at compile
# time (include/ptp_collection.hpp).
$(BUILD)/heatd.o: include/ptp_collection.hpp fixtures/ptp_collection_000D_0005.bin

heatd: $(HEATD_OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV_LIBS)

digid: $(DIGIPROC_OBJ) | identity
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
