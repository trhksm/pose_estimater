CXX := g++
CXXFLAGS := -std=c++17 -O2

OPENCV_FLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS  := $(shell pkg-config --libs opencv4)

SRC := camera_display.cpp \
       ../capture/src/frame_capture_base.cpp \
       ../capture/src/frame_camera_capture.cpp

TARGET := test1

all: $(TARGET)

$(TARGET):
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $(SRC) -o $@ $(OPENCV_LIBS)

clean:
	rm -f $(TARGET)
