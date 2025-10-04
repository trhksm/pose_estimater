#include "../capture/include/frame_camera_capture.hpp"
#include "../cal/include/cal_calibration.hpp"
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

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);

        cv::Mat frame;
        cv::namedWindow("ArUco", cv::WINDOW_NORMAL);

        //arg for test 
        Vec3 shelf_world_position = {0.0,0.0,-3.0};
        Vec3 shelf_rotate_axis = {1.0,0.0,0.0};
        double shelf_rotate_rad = 0.001;
        Vec3 camera_world_position = get_camera_world_position(shelf_world_position, shelf_rotate_axis, shelf_rotate_rad);
        std::vector<Vec3> ideal_fov_unit_vecs = get_ideal_fov_unit_vecs();

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

                std::vector<Vec3> ideal_aruco_unit_vecs = get_ideal_aruco_unit_vecs(camera_world_position, corners[0]);
                std::vector<Vec3> ideal_aruco_positions = get_ideal_aruco_positions(ideal_aruco_unit_vecs, camera_world_position);
				std::vector<Vec3> ideal_fov_positions = get_ideal_fov_positions(ideal_fov_unit_vecs, camera_world_position);
                std::vector<std::vector<double>> ideal_aruco_screen_positions = get_ideal_aruco_screen_positions(ideal_aruco_positions,ideal_fov_positions);
                std::vector<Vec3> camera_rotate_axis = get_camera_rotate_axis(ideal_aruco_screen_positions, corners[0]);
                double camera_rotate_degree = get_camera_rotate_degree(camera_rotate_axis, ideal_aruco_positions, camera_world_position, ideal_fov_unit_vecs, corners[0]);

                std::cout << "Camera Rotate Degree: " << camera_rotate_degree << std::endl;
            }

            cv::imshow("ArUco", frame);

            if(cv::waitKey(1) == 27) break;
        }
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}



