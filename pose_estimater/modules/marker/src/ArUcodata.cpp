#include "../include/ArUcodata.hpp"

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

Vec3 getPositionFromCSVById(const std::string& filename, int target_id) {
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "CSVファイルを開けませんでした: " << filename << std::endl;
        return {0,0,0};
    }

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


void save_vec3_vectors(const std::string& filename, const std::vector<Vec3>& directions) {
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "ファイルを開けません: " << filename << std::endl;
        return;
    }

    for (const auto& d : directions) {
        // 始点(0,0,0)から方向ベクトルdへ
        ofs << 0.0 << " " << 0.0 << " " << -0.9 << " "
            << d[0] << " " << d[1] << " " << d[2] << "\n";
    }
}

std::vector<Vec3> get_aruco_corner_positions(std::vector<int>ids) {
    int target_id = ids[0];
    std::string column_name = std::to_string(target_id);

    Vec3 corner_center_position = getPositionFromCSVById("../marker/data/data.csv", target_id);
    double cx = corner_center_position[0];
    double cy = corner_center_position[1];
    std::vector<Vec3> corner_positions = {{cx + 0.04, cy - 0.04, 0.0},{cx + 0.04,cy + 0.04, 0.0},{cx - 0.04,cy + 0.04, 0.0},{cx - 0.04, cy - 0.04, 0.0}};
    return corner_positions;
}
