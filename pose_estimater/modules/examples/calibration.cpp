#include "../capture/include/frame_camera_capture.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
int main() {
    const char* devname = "/dev/video0";
    const char* param_file = "../param/camera_calib/baffalo_BSW300M.xml";
    int width = 1280;
    int height = 720;
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    try {
        Modules::PoseEstimation::FrameCameraCapture cam(devname, width, height, fourcc, param_file);

	if (!cam.is_opened()) {
	    std::cerr << "Camera failed to open" << std::endl;
	    return 1;
	}

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	cv::Mat frame;
	cv::namedWindow("ArUco", cv::WINDOW_NORMAL);

	while(true) {
	    std::chrono::steady_clock::time_point tp;
	    if (!cam.read_until(frame, tp, 500)) {
                std::cerr << "Failed to read frame" << std::endl;
		continue;
	    }

	    std::vector<int> ids;
	    std::vector<std::vector<cv::Point2f>> corners;
	    cv::aruco::detectMarkers(frame, dictionary, corners, ids);

	    if(!ids.empty()) {
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
	    } 
	    cv::imshow("ArUco",frame);

	    if(cv::waitKey(1) == 27) break;
	}
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}



