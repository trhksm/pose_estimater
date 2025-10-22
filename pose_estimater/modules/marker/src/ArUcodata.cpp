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

std::vector<std::vector<Vec3>> get_aruco_corners_positions(const std::vector<int>& ids) {
    std::vector<std::vector<Vec3>> aruco_corner_positions;

    for (int target_id : ids) {
        Vec3 center = getPositionFromCSVById("../marker/data/data.csv", target_id);
        double cx = center[0];
        double cy = center[1];

        std::vector<Vec3> corners = {
            {cx + 0.04, cy - 0.04, 0.0},
            {cx + 0.04, cy + 0.04, 0.0},
            {cx - 0.04, cy + 0.04, 0.0},
            {cx - 0.04, cy - 0.04, 0.0}
        };

        aruco_corner_positions.push_back(corners);
    }

    return aruco_corner_positions;
}

std::vector<std::vector<int>> get_pairs_id_and_index(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners){
    int detected_num = corners.size();
    int acceptable_pair_dist = 100;//check later
    std::vector<cv::Point2f> centers(detected_num);
    std::vector<cv::Point2f> right_dir(detected_num);
    std::vector<std::vector<int>> pairs_id_and_index;
    std::vector<bool> used(detected_num, false);

    for (int i = 0; i < detected_num; i++) {
        // マーカー中心
        centers[i] = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
        // マーカーにとって右方向（右上 - 左上）agvの方向を考えてないが大丈夫？
        right_dir[i] = corners[i][1] - corners[i][0];
    }

    for (int i = 0; i < detected_num; i++) {
        if (used[i]) continue;//check later

        int nearest_index = -1;
        float min_dist = std::numeric_limits<float>::max();
        //一番近いマーカ探し
        for (int j = 0; j < detected_num; j++) {
            if (i == j || used[j]) continue;

            float dx = centers[j].x - centers[i].x;
            float dy = centers[j].y - centers[i].y;
            float dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                nearest_index = j;
            }
        }
        if ( min_dist > acceptable_pair_dist) continue;

        //ペアに追加
        if (nearest_index != -1) {
            cv::Point2f diff = centers[nearest_index] - centers[i];
            float dot = diff.x * right_dir[i].x + diff.y * right_dir[i].y;            
            if (dot > 0) {
                // neighborが右側
                pairs_id_and_index.push_back({ids[i], ids[nearest_index],i,nearest_index});
            } else {
                // neighborが左側
                pairs_id_and_index.push_back({ids[nearest_index], ids[i],nearest_index,i});
            }

            used[i] = true;
            used[nearest_index] = true;
        }
    }

    return pairs_id_and_index;
}

std::vector<std::pair<std::vector<Vec3>,int>> get_pairs_aruco_corners_positions_and_index(
    const std::vector<std::vector<int>>& pairs_id_and_index)
{
    std::vector<std::pair<std::vector<Vec3>,int>> pairs_aruco_corners_positions_and_index;
    std::string filename = "../marker/data/data2.csv";
    std::ifstream ifs(filename);

    if (!ifs.is_open()) {
        std::cerr << "ファイルを開けません: " << filename << std::endl;
        return {};
    }


    std::string line;
    std::getline(ifs, line); // ヘッダー行スキップ
    std::vector<std::vector<std::string>> csv_data;

    double cx,cy;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        auto tokens = split(line, ',');
        if (tokens.size() < 6) continue;
        csv_data.push_back(tokens);
    }
    for (const auto& pair : pairs_id_and_index) {
        int id1 = pair[0];
        int id2 = pair[1];
        int idx1 = pair[2];
        int idx2 = pair[3];
        bool found = false;

        std::cout << "Processing pair ids: (" << id1 << ", " << id2 << "), indices: (" << idx1 << ", " << idx2 << ")" << std::endl;

        for (const auto& tokens : csv_data) {
            try {
                int file_id1 = std::stoi(tokens[0]);
                int file_id2 = std::stoi(tokens[1]);

                if (file_id1 == id1 && file_id2 == id2) {
                    cx = std::stof(tokens[3]) / 1000.0;
                    cy = std::stof(tokens[4]) / 1000.0;
                    found = true;
                    break;
                }
            } catch (...) {
                continue;
            }
        }

        if (!found) {
            std::cerr << "ペア (" << id1 << ", " << id2 << ") が見つかりません。" << std::endl;
        }
        double cy1 = cy - 0.05; double cy2 = cy + 0.05;
        std::vector<Vec3> corners1 = {
            {cx + 0.04, cy1 - 0.04, 0.0},
            {cx + 0.04, cy1 + 0.04, 0.0},
            {cx - 0.04, cy1 + 0.04, 0.0},
            {cx - 0.04, cy1 - 0.04, 0.0}
        };
        std::vector<Vec3> corners2 = {
            {cx + 0.04, cy2 - 0.04, 0.0},
            {cx + 0.04, cy2 + 0.04, 0.0},
            {cx - 0.04, cy2 + 0.04, 0.0},
            {cx - 0.04, cy2 - 0.04, 0.0}
        };

        pairs_aruco_corners_positions_and_index.push_back({corners1,idx1});
        pairs_aruco_corners_positions_and_index.push_back({corners2,idx2});
    }
    return pairs_aruco_corners_positions_and_index;
}
