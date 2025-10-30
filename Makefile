# Makefile for Raytracer core

CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
TARGET   = render

# Object files
OBJS = main.o camera.o image_ppm.o mat4.o shape.o cube.o sphere.o plane.o bvh.o

# Default target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

# Pattern rule for .cpp -> .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

# Optional convenience target
run: $(TARGET)
	./$(TARGET)

