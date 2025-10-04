CXX := g++
CXXFLAGS := -std=c++17 -O2 -I/usr/include/eigen3

OPENCV_FLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS  := $(shell pkg-config --libs opencv4)

COMMON_SRC := ../capture/src/frame_capture_base.cpp \
              ../capture/src/frame_camera_capture.cpp \
	      
CAL_SRC := ../cal/src/cal_base.cpp \
	      ../cal/src/cal_calibration.cpp \

TARGETS := capture_display detect_aruco calibration

all: $(TARGETS)

capture_display: capture_display.cpp $(COMMON_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

detect_aruco: detect_aruco.cpp $(COMMON_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

calibration: calibration.cpp $(COMMON_SRC) $(CAL_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

clean:
	rm -f $(TARGETS)

