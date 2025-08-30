# ============================================================
# Toolchain / PATH
# ============================================================
export PATH := /mingw64/bin:/usr/bin:/bin:$(PATH)

CXX := g++
AR  := ar

# ============================================================
# Project
# ============================================================
TARGET := Fx2D.exe
BUILD_DIR := build

# ============================================================
# pkg-config (MSYS2)
# ============================================================
RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib)
RAYLIB_LIBS   := $(shell pkg-config --libs   raylib)
YAML_CFLAGS   := $(shell pkg-config --cflags yaml-cpp)
YAML_LIBS     := $(shell pkg-config --libs --static yaml-cpp)

# ============================================================
# Flags
# ============================================================
CXXFLAGS := -std=c++20 -Wall -Wextra -O3\
            -DNO_FONT_AWESOME \
            -I./ -Iinclude \
            -Ilib/imgui -Ilib/rlImGui \
            -I/mingw64/include/eigen3 \
            -DYAML_CPP_STATIC_DEFINE \
            $(YAML_CFLAGS) \
            $(RAYLIB_CFLAGS)

LDFLAGS := $(RAYLIB_LIBS) $(YAML_LIBS)

# ============================================================
# Sources
# ============================================================
APP_SRCS := \
    src/main.cpp \
    src/Entity.cpp \
    src/Scene.cpp \
    src/Collisions.cpp \
    src/ConstraintSolver.cpp \
    src/Renderer.cpp \
    src/YamlUtils.cpp

IMGUI_CORE_SRCS := \
    lib/imgui/imgui.cpp \
    lib/imgui/imgui_draw.cpp \
    lib/imgui/imgui_tables.cpp \
    lib/imgui/imgui_widgets.cpp

RLIMGUI_SRCS := \
    lib/rlImGui/rlImGui.cpp \
    $(IMGUI_CORE_SRCS)

SRCS := $(APP_SRCS) $(RLIMGUI_SRCS)
OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS))

# ============================================================
# Phony targets
# ============================================================
.PHONY: all clean rebuild debug init

all: raylib

init:
	@mkdir -p $(BUILD_DIR)

# ============================================================
# Build rules
# ============================================================
raylib: init $(OBJS)
	@echo "Linking (raylib) -> $(TARGET)"
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS)
	@echo "Built: $(TARGET)"

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling (raylib) $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ============================================================
# Housekeeping
# ============================================================
clean:
	@echo "Cleaning..."
	@rm -rf $(BUILD_DIR) $(TARGET)

rebuild: clean all

debug: CXXFLAGS += -g -DDEBUG
debug: clean all
