# SPDX-License-Identifier: MIT
# surface-heatview: the touchpad heat map, with heatd's engine. Needs SDL3.
# Not built or installed by the top-level make; run `make tools` there.

SRC := ../../src

CXXFLAGS ?= -O2
CXX_EXTRA := -std=c++20 -Wall -Wextra -Wpedantic -I$(SRC)/include \
             $(patsubst -I%,-isystem %,$(shell pkg-config --cflags opencv5 sdl3 2>/dev/null))
LDLIBS := -lopencv_core -lopencv_imgproc -lopencv_geometry $(shell pkg-config --libs sdl3 2>/dev/null)

surface-heatview: heatview.cpp $(SRC)/heat_engine.cpp $(SRC)/touch_tracker.cpp
	$(CXX) $(CXX_EXTRA) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -f surface-heatview

.PHONY: clean
