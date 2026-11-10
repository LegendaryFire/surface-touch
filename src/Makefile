# SPDX-License-Identifier: MIT
# heatd (touchpad) and digid (touchscreen + pen).

# ---- sources ---------------------------------------------------------------

HEATD_SRC    := heatd.cpp heat_engine.cpp touch_tracker.cpp
DIGID_SRC    := digid.cpp digi_pipeline.cpp pen_position.cpp touch_tracker.cpp

BUILD := build

# ---- flags -----------------------------------------------------------------

# CXXFLAGS and LDFLAGS are the user's (makepkg sets its own); what the code
# needs to build at all goes in the extra flags below, which are always applied.
CXXFLAGS ?= -O2
CXX_EXTRA := -std=c++20 -Wall -Wextra -Wpedantic -Iinclude

# OpenCV 5, for connected components and moments.  Its headers go in with
# -isystem so they don't trigger our warnings.
OPENCV := opencv5
ifneq ($(MAKECMDGOALS),clean)
  ifeq ($(shell pkg-config --exists $(OPENCV) && echo yes),)
    $(error OpenCV 5 not found: install 'opencv' (Arch) or your distribution's OpenCV 5 development package)
  endif
endif
CXX_EXTRA += $(patsubst -I%,-isystem %,$(shell pkg-config --cflags $(OPENCV) 2>/dev/null))
LDLIBS    := -lopencv_core -lopencv_imgproc -lopencv_geometry

# ---- rules -----------------------------------------------------------------

all: heatd digid

heatd:    $(HEATD_SRC:%.cpp=$(BUILD)/%.o)
digid:    $(DIGID_SRC:%.cpp=$(BUILD)/%.o)

heatd digid:
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

# -MMD -MP writes build/*.d: every header (and #embed file) each object
# used, so editing one rebuilds exactly what depends on it.
$(BUILD)/%.o: %.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXX_EXTRA) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(wildcard $(BUILD)/*.d)

clean:
	rm -rf $(BUILD) heatd digid

.PHONY: all clean
