CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic
LDFLAGS  :=

TARGET   := raytracer

SRCS := \
    bvh.cpp \
    camera.cpp \
    cube.cpp \
    image_ppm.cpp \
    main.cpp \
    mat4.cpp \
    plane.cpp \
    shade.cpp \
    shape.cpp \
    sphere.cpp \
    texture.cpp

OBJS := $(SRCS:.cpp=.o)

# Default target: optimised build
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Debug build: raytracer_debug (no optimisation, with symbols)
debug: CXXFLAGS := -std=c++17 -O0 -g -Wall -Wextra -pedantic
debug: TARGET   := raytracer_debug
debug: $(TARGET)

# Clean up
clean:
	rm -f $(OBJS) $(TARGET) raytracer_debug

.PHONY: all debug clean


