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

IMU_SRC := ../hw_imu_module/src/imu.cpp \
           ../hw_serial_module/src/serial.cpp \
		   ../utils_module/src/time_api/time_api_real.cpp \
		   ../utils_module/src/time_api/time_api_simulator.cpp \


TARGETS := capture_display detect_aruco calibration localization localization_w_imu agv_localization

all: $(TARGETS)

capture_display: capture_display.cpp $(COMMON_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

detect_aruco: detect_aruco.cpp $(COMMON_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

calibration: calibration.cpp $(COMMON_SRC) $(CAL_CALIBRATION_SRC) $(MARKER_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

localization: localization.cpp $(COMMON_SRC) $(CAL_LOCALIZATION_SRC) $(MARKER_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

calibration_w_imu: calibration_w_imu.cpp $(COMMON_SRC) $(CAL_CALIBRATION_SRC) $(MARKER_SRC) $(IMU_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

localization_w_imu: localization_w_imu.cpp $(COMMON_SRC) $(CAL_LOCALIZATION_SRC) $(MARKER_SRC) $(IMU_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

agv_localization: agv_localization.cpp $(COMMON_SRC) $(CAL_LOCALIZATION_SRC) $(MARKER_SRC) $(IMU_SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $^ -o $@ $(OPENCV_LIBS)

clean:
	rm -f $(TARGETS)

