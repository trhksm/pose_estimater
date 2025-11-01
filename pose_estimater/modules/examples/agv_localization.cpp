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

        //arg for test
        double calibration_pitch = -0.0109;//get_calibration_pitch()
        double calibration_roll = 0.0698;//get_calibration_roll()
        std::vector<Vec3> ideal_fov_unit_vecs = get_agv_ideal_fov_unit_vecs();
        Vec3 calibration_agv_pose           = get_calibration_agv_pose(calibration_pitch,calibration_roll);//must be changed

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
                std::cout << "pitch" << pitch << "roll" << roll << "yaw" << yaw << std::endl;
                Vec3 agv_pose                                       = get_agv_pose(pitch,roll,calibration_pitch,calibration_roll,yaw);//yaw check
                Vec3 agv_rotate_axis                                = get_camera_rotate_axis(agv_pose);
                double agv_rotate_rad                               = get_camera_rotate_rad(agv_pose);
                std::vector<Vec3> fov_vecs                          = get_agv_fov_vecs(ideal_fov_unit_vecs, agv_rotate_axis, agv_rotate_rad,yaw);//ワールド座標基準 must be changed
                std::vector<std::vector<Vec3>> camera_to_aruco_vecs = get_camera_to_aruco_vecs(corners, fov_vecs);
                std::vector<std::vector<int>>    pairs_id_and_index = get_pairs_id_and_index(ids, corners);
                std::vector<std::pair<std::vector<Vec3>,int>> pairs_aruco_corners_positions_and_index = get_pairs_aruco_corners_positions_and_index(pairs_id_and_index);
                Vec3 camera_world_positions                         = get_camera_world_positions(camera_to_aruco_vecs, pairs_aruco_corners_positions_and_index);
                Vec3 agv_world_positions                            = get_agv_world_positions(camera_world_positions,yaw);
                auto log_vec = [](const std::string& name, const Vec3& v) {
                    std::cout << name << " : (" << v[0] << ", " << v[1] << ", " << v[2] << ")\n";
                };

                log_vec("agv_rotate_axis", agv_rotate_axis);
                std::cout << "agv_rotate_rad: " << agv_rotate_rad << std::endl;
                
                /*for (const auto& pos : fov_vecs) {
                    std::cout << "fov_vecs: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
                }*/
                //for (const auto& pos : camera_to_aruco_vecs) {
                  //  std::cout << "camera_to_aruco_vecs :(" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
                //}
                std::cout << "agv_world_positions :(" << agv_world_positions[0] << ", " << agv_world_positions[1] << ", " << agv_world_positions[2] << ")" << std::endl;
                /*for (size_t i = 0; i < corners.size(); ++i) {
                    std::cout << "ID: " << ids[i] << std::endl;
                    for (size_t j = 0; j < corners[i].size(); ++j) {
                        const auto& pt = corners[i][j];
                        std::cout << "  corner[" << j << "]: (" << pt.x << ", " << pt.y << ")" << std::endl;
                    }    
                }
                */

                //save_vec3_vectors("../testdata/localization/ideal_fov_unit_vecs.txt", ideal_fov_unit_vecs);
                //save_vec3_vectors("../testdata/localization/fov_vecs.txt", fov_vecs);
                //save_vec3_vectors("../testdata/localization/camera_to_aruco_vecs.txt", camera_to_aruco_vecs);
                //save_vec3_vectors("../testdata/localization/camera_world_positions.txt", camera_world_positions);
            } 
            cv::imshow("ArUco", frame);

            if(cv::waitKey(1) == 27) break;
        }
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
