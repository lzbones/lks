# LKS (Lane Keeping System) Makefile
# 车道保持系统仿真构建文件

CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic -Iinclude
LDFLAGS  := -lm

BUILDDIR := build
TARGET   := $(BUILDDIR)/lks

SRCS     := src/main.cpp \
            src/math/angle_normalize.cpp \
            src/math/clamp_f64.cpp \
            src/vehicle/state_init.cpp \
            src/vehicle/params_default.cpp \
            src/vehicle/delta_rate_limit.cpp \
            src/vehicle/step.cpp \
            src/lane/model_init.cpp \
            src/lane/model_free.cpp \
            src/lane/model_push.cpp \
            src/lane/gen_sine.cpp \
            src/lane/gen_arc.cpp \
            src/lane/find_lookahead_idx.cpp \
            src/lane/find_lookahead_pt.cpp \
            src/controller/pp_config_default.cpp \
            src/controller/pp_compute_delta.cpp \
            src/controller/cross_error.cpp \
            src/simulation/speed_pid.cpp \
            src/simulation/run.cpp

# 将 src/foo.cpp → build/foo.o
OBJS     := $(patsubst src/%.cpp,$(BUILDDIR)/%.o,$(SRCS))

.PHONY: all clean run test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# 编译每个源文件到 build/ 对应子目录
$(BUILDDIR)/main.o: src/main.cpp | create_build_dir
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILDDIR)/math/%.o: src/math/%.cpp | create_math_dir
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILDDIR)/vehicle/%.o: src/vehicle/%.cpp | create_vehicle_dir
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILDDIR)/lane/%.o: src/lane/%.cpp | create_lane_dir
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILDDIR)/controller/%.o: src/controller/%.cpp | create_controller_dir
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILDDIR)/simulation/%.o: src/simulation/%.cpp | create_simulation_dir
	$(CXX) $(CXXFLAGS) -c -o $@ $<

create_build_dir:
	mkdir -p $(BUILDDIR)

create_math_dir:
	mkdir -p $(BUILDDIR)/math

create_vehicle_dir:
	mkdir -p $(BUILDDIR)/vehicle

create_lane_dir:
	mkdir -p $(BUILDDIR)/lane

create_controller_dir:
	mkdir -p $(BUILDDIR)/controller

create_simulation_dir:
	mkdir -p $(BUILDDIR)/simulation

clean:
	rm -rf $(BUILDDIR)
	@rm -f test/*.csv test/*.png 2>/dev/null || true

run: $(TARGET)
	./$(TARGET)

test: run
	python3 test/result_plot.py