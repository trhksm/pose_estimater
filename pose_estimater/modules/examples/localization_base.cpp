#include "../capture/include/frame_camera_capture.hpp"
#include "../cal/include/cal_localization.hpp"
#include "../marker/include/ArUcodata.hpp"
#include "../hw_imu_module/include/imu.hpp"
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

        //arg for test id :8camera_rotate0 :(0.822185, 0.56922, -0)  : 0.010426
        double calibration_pitch = -0.0109;//get_calibration_pitch() //キャリブレーションは棚基準
        double calibration_roll = 0.0698;//get_calibration_roll() //キャリブレーションは棚基準
        Vec3 calibration_camera_pose = {0.0,0.0,1.0};//get_calibration_camera_pose() only cal by pitch and roll
        Vec3 calibration_camera_axis = {0.822185, 0.56922, 0};
        double calibration_camera_rad = 0.010426;
        rot(calibration_camera_pose,calibration_camera_axis,ORIGIN,calibration_camera_rad,calibration_camera_pose);
        std::vector<Vec3> ideal_fov_unit_vecs = get_ideal_fov_unit_vecs();
        Vec3 calibration_shelf_pose           = get_calibration_shelf_pose(calibration_pitch,calibration_roll);//棚基準 must be cahnge

        Modules::Hardware::IMU imu("/dev/pico_imu",calibration_pitch,calibration_roll);
        imu.init();
        double pitch = 0.0;
        double roll  = 0.0;
        
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
                imu.read(pitch,roll);//棚基準のpitch rollの変位をもらう
                double yaw                                          = get_yaw(corners);
                std::cout << "pitch" << pitch << "roll" << roll << std::endl;
                Vec3 shelf_pose                                     = get_shelf_pose(pitch,roll,calibration_pitch,calibration_roll);//棚基準 must be cahnged
                Vec3 camera_pose                                    = get_camera_pose(calibration_camera_pose, calibration_shelf_pose, shelf_pose,yaw);//ワールド座標基準 must be changed
                Vec3 camera_rotate_axis                             = get_camera_rotate_axis(camera_pose);
                double camera_rotate_rad                            = get_camera_rotate_rad(camera_pose);
                std::vector<Vec3> fov_vecs                          = get_fov_vecs(ideal_fov_unit_vecs, camera_rotate_axis, camera_rotate_rad,yaw);//ワールド座標基準 must be changed
                std::vector<std::vector<Vec3>> camera_to_aruco_vecs = get_camera_to_aruco_vecs(corners, fov_vecs);
                std::vector<std::vector<int>>    pairs_id_and_index = get_pairs_id_and_index(ids, corners);
                std::vector<std::pair<std::vector<Vec3>,int>> pairs_aruco_corners_positions_and_index = get_pairs_aruco_corners_positions_and_index(pairs_id_and_index);
                Vec3 camera_world_positions                         = get_camera_world_positions(camera_to_aruco_vecs, pairs_aruco_corners_positions_and_index);

                Vec3 agv_world_positions                            = get_shelf_agv_world_positions(camera_world_positions,shelf_pose);

                auto log_vec = [](const std::string& name, const Vec3& v) {
                    std::cout << name << " : (" << v[0] << ", " << v[1] << ", " << v[2] << ")\n";
                };
                log_vec("camera_pose", camera_pose);
                log_vec("camera_rotate_axis", camera_rotate_axis);
                std::cout << "camera_rotate_rad: " << camera_rotate_rad << std::endl;
                std::cout << "camera_world_positions :(" << camera_world_positions[0] << ", " << camera_world_positions[1] << ", " << camera_world_positions[2] << ")" << std::endl;
            } 
            cv::imshow("ArUco", frame);

            if(cv::waitKey(1) == 27) break;
        }
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}