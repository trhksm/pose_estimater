CXX := g++
CXXFLAGS := -std=c++17 -O2

OPENCV_FLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS  := $(shell pkg-config --libs opencv4)

COMMON_SRC := ../capture/src/frame_capture_base.cpp \
              ../capture/src/frame_camera_capture.cpp \
	      
CAL_CALIBRATION_SRC := ../cal/src/cal_base.cpp \
	      ../cal/src/cal_calibration.cpp \

CAL_LOCALIZATION_SRC := ../cal/src/cal_base.cpp \
			../cal/src/cal_localization.cpp \

MARKER_SRC := ../marker/src/ArUcodata.cpp

TARGETS := capture_display detect_aruco calibration localization

all: $(TARGETS)

capture_display: capture_display.cpp $(COMMON_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

detect_aruco: detect_aruco.cpp $(COMMON_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

calibration: calibration.cpp $(COMMON_SRC) $(CAL_CALIBRATION_SRC) $(MARKER_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

localization: localization.cpp $(COMMON_SRC) $(CAL_LOCALIZATION_SRC) $(MARKER_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)


clean:
	rm -f $(TARGETS)

