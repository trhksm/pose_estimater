#include "../capture/include/frame_camera_capture.hpp"
#include "../cal/include/cal_calibration.hpp"
#include "../marker/include/ArUcodata.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <chrono>

int main() {
    const char* devname = "/dev/video0";//要変更
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
        Vec3 camera_world_position = {0.0,0.0,-1 * CAMERA_HEIGHT};//get_camera_world_position(shelf_world_position, shelf_rotate_axis, shelf_rotate_rad);
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
            // フレームサイズ取得
            int cx = frame.cols / 2;
            int cy = frame.rows / 2;

            // 中心に赤い十字マーカーを描く
            cv::drawMarker(frame, cv::Point(cx, cy), cv::Scalar(0, 0, 255),  // 赤 (BGR)
                cv::MARKER_CROSS, 20, 2);  // サイズ20px、太さ2


            if(!ids.empty()) {
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
                std::vector<std::vector<Vec3>> corners_positions = get_aruco_corners_positions(ids);
                for (int i = 0; i < corners_positions.size(); i++ ) {

                    std::vector<Vec3> ideal_aruco_unit_vecs = get_ideal_aruco_unit_vecs(camera_world_position, corners_positions[i]);//不必要かも
                    std::vector<Vec3> ideal_aruco_positions = get_ideal_aruco_positions(ideal_aruco_unit_vecs, camera_world_position);//不必要かも
                    std::vector<Vec3> ideal_fov_positions = get_ideal_fov_positions(ideal_fov_unit_vecs, camera_world_position);
                    std::vector<std::vector<double>> ideal_aruco_screen_positions = get_ideal_aruco_screen_positions(ideal_aruco_positions,ideal_fov_positions);
                    std::vector<Vec3> camera_rotate_axis = get_camera_rotate_axis(ideal_aruco_screen_positions, corners[i]);
                    std::vector<double> camera_rotate_rad = get_camera_rotate_rad(camera_rotate_axis, ideal_aruco_positions, camera_world_position, ideal_fov_unit_vecs, corners[i]);

                    for (int j = 0; j < camera_rotate_axis.size(); j++) {
                        std::cout <<"id :" << ids[i] <<"camera_rotate" << j << " :(" << camera_rotate_axis[j][0] << ", " << camera_rotate_axis[j][1] << ", " << camera_rotate_axis[j][2] << ")  : " << camera_rotate_rad[j]  << std::endl;
                    }

                    save_vec3_vectors("../testdata/calibration/ideal_aruco_unit_vecs.txt", ideal_aruco_unit_vecs);
                    save_vec3_vectors("../testdata/calibration/ideal_aruco_positions.txt", ideal_aruco_positions);
                    save_vec3_vectors("../testdata/calibration/ideal_fov_positions.txt", ideal_fov_positions);
                    save_vec3_vectors("../testdata/calibration/camera_rotate_axis.txt", camera_rotate_axis);
                }
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