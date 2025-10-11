#include "../capture/include/frame_camera_capture.hpp"
#include "../cal/include/cal_calibration.hpp"
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

//要注意
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

//要注意
Vec3 getPositionFromCSVById(const std::string& filename, int target_id) {
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "CSVファイルを開けませんでした: " << filename << std::endl;
        return {0,0,0};
    }

    // ヘッダー読み飛ばし
    if (!std::getline(file, line)) {
        std::cerr << "CSVが空です。" << std::endl;
        return {0,0,0};
    }

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string id_str, x_str, y_str;
        if (!std::getline(ss, id_str, ',')) continue;
        if (!std::getline(ss, x_str, ',')) continue;
        if (!std::getline(ss, y_str, ',')) continue;

        int id = std::stoi(id_str);
        if (id == target_id) {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            return {x, y, 0.0};
        }
    }

    std::cerr << "ID " << target_id << " が見つかりませんでした。" << std::endl;
    return {0, 0, 0};
}
std::vector<double> readNumericColumnFromCSV(const std::string& filename, const std::string& columnName) {
    std::ifstream file(filename);
    std::string line;
    std::vector<double> columnData;
    std::unordered_map<std::string, int> columnIndexMap;

    if (!file.is_open()) {
        std::cerr << "CSVファイルを開けませんでした: " << filename << std::endl;
        return columnData;
    }

    // ヘッダー行を読み取り
    if (std::getline(file, line)) {
        auto headers = split(line, ',');
        for (size_t i = 0; i < headers.size(); ++i) {
            columnIndexMap[headers[i]] = i;
        }

        // 該当列があるか確認
        if (columnIndexMap.find(columnName) == columnIndexMap.end()) {
            std::cerr << "列名 " << columnName << " が見つかりません。" << std::endl;
            return columnData;
        }

        int colIndex = columnIndexMap[columnName];

        // データ行を読み取り
        while (std::getline(file, line)) {
            auto values = split(line, ',');
            if (colIndex < values.size()) {
                try {
                    double num = std::stod(values[colIndex]);
                    columnData.push_back(num);
                } catch (const std::exception& e) {
                    std::cerr << "変換エラー: " << e.what() << std::endl;
                }
            }
        }
    }

    return columnData;
}
// 方向ベクトルとして表示したい場合
void save_vec3_vectors(const std::string& filename, const std::vector<Vec3>& directions) {
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "ファイルを開けません: " << filename << std::endl;
        return;
    }

    for (const auto& d : directions) {
        // 始点(0,0,0)から方向ベクトルdへ
        ofs << 0.0 << " " << 0.0 << " " << 0.0 << " "
            << d[0] << " " << d[1] << " " << d[2] << "\n";
    }
}


std::vector<Vec3> get_aruco_corner_positions(std::vector<int>ids) {
    int target_id = ids[0];  // 例: 23
    std::string column_name = std::to_string(target_id);

    Vec3 corner_center_position = getPositionFromCSVById("data.csv", target_id);
    double cx = corner_center_position[0];
    double cy = corner_center_position[1];
    std::vector<Vec3> corner_positions = {{cx + 0.04, cy - 0.04, 0.0},{cx + 0.04,cy + 0.04, 0.0},{cx - 0.04,cy + 0.04, 0.0},{cx - 0.04, cy - 0.04, 0.0}};
    return corner_positions;
}


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
                std::cout << ids[0] << std::endl;
                std::vector<Vec3> corners_positions = get_aruco_corner_positions(ids);

                std::vector<Vec3> ideal_aruco_unit_vecs = get_ideal_aruco_unit_vecs(camera_world_position, corners_positions);
                std::vector<Vec3> ideal_aruco_positions = get_ideal_aruco_positions(ideal_aruco_unit_vecs, camera_world_position);
				std::vector<Vec3> ideal_fov_positions = get_ideal_fov_positions(ideal_fov_unit_vecs, camera_world_position);
                std::vector<std::vector<double>> ideal_aruco_screen_positions = get_ideal_aruco_screen_positions(ideal_aruco_positions,ideal_fov_positions);
                std::vector<Vec3> camera_rotate_axis = get_camera_rotate_axis(ideal_aruco_screen_positions, corners[0]);
                std::vector<double> camera_rotate_rad = get_camera_rotate_rad(camera_rotate_axis, ideal_aruco_positions, camera_world_position, ideal_fov_unit_vecs, corners[0]);

                std::cout << "Camera Rotate Rad 0: " << camera_rotate_rad[0] << std::endl;
                std::cout << "Camera Rotate Rad 1: " << camera_rotate_rad[1] << std::endl;
                std::cout << "Camera Rotate Rad 2: " << camera_rotate_rad[2] << std::endl;
                std::cout << "Camera Rotate Rad 3: " << camera_rotate_rad[3] << std::endl;

                for (const auto& pos : ideal_aruco_unit_vecs) {
                    std::cout << "ideal_aruco_unit_vecs: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
                }
                for (const auto& pos : ideal_aruco_positions) {
                    std::cout << "ideal_aruco_positions :(" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
                }
                for (const auto& pos : ideal_fov_positions) {
                    std::cout << "ideal_fov_positions :(" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
                }
                for (const auto& pos : ideal_aruco_screen_positions) {
                    std::cout << "ideal_aruco_screen_positions :(" << pos[0] << ", " << pos[1] << ")" << std::endl;
                }
                for (const auto& pos : camera_rotate_axis) {
                    std::cout << "camera_rotate_axis :(" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
                }
                for (size_t i = 0; i < corners.size(); ++i) {
                    std::cout << "ID: " << ids[i] << std::endl;
                    for (size_t j = 0; j < corners[i].size(); ++j) {
                        const auto& pt = corners[i][j];
                        std::cout << "  corner[" << j << "]: (" << pt.x << ", " << pt.y << ")" << std::endl;
                    }    
                }
                save_vec3_vectors("ideal_aruco_unit_vecs.txt", ideal_aruco_unit_vecs);
                save_vec3_vectors("ideal_aruco_positions.txt", ideal_aruco_positions);
                save_vec3_vectors("ideal_fov_positions.txt", ideal_fov_positions);
                save_vec3_vectors("camera_rotate_axis.txt", camera_rotate_axis);


            }

            cv::imshow("ArUco", frame);

            if(cv::waitKey(1) == 27) break;
            //std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

