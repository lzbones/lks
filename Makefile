# LKS (Lane Keeping System) Makefile
# 车道保持系统仿真构建文件

CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic
LDFLAGS  := -lm

TARGET   := lks
SRCS     := main.cpp vehicle_model.cpp lane_model.cpp pure_pursuit.cpp simulation.cpp
OBJS     := $(SRCS:.cpp=.o)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET) *.csv *.png

run: $(TARGET)
	./$(TARGET)

# 依赖自动推导
main.o: main.cpp simulation.h
vehicle_model.o: vehicle_model.cpp vehicle_model.h config.h
lane_model.o: lane_model.cpp lane_model.h
pure_pursuit.o: pure_pursuit.cpp pure_pursuit.h vehicle_model.h lane_model.h config.h
simulation.o: simulation.cpp simulation.h vehicle_model.h lane_model.h pure_pursuit.h config.h