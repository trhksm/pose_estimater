#include "../include/cal_calibration.hpp"
#include "../include/cal_base.hpp"

auto solve2x2 = [](double J00, double J01, double J10, double J11,
                   double F0, double F1, double& dx, double& dy) {
    double det = J00 * J11 - J01 * J10;
    if (std::fabs(det) < 1e-12) { // 特異行列チェック
        dx = dy = 0.0;
        return false;
    }
    dx = (-F0 * J11 + F1 * J01) / det;
    dy = (-F1 * J00 + F0 * J10) / det;
    return true;
};

Vec3 get_camera_world_position(const Vec3& shelf_world_position, const Vec3& shelf_rotate_axis ,const double shelf_rotate_rad) {
    Vec3 camera_world_position;
    const Vec3 ideal_camera_world_position = {shelf_world_position[0], shelf_world_position[1], -1 * CAMERA_HEIGHT};
    rot(ideal_camera_world_position ,shelf_rotate_axis ,shelf_world_position ,shelf_rotate_rad ,camera_world_position);
    return camera_world_position;
}

std::vector<Vec3> get_ideal_fov_unit_vecs() {
    std::vector<Vec3> ideal_fov_unit_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

    Vec3 left_fov_origin_vec;rot(AXIS_Z, AXIS_Y, ORIGIN, -1.0 * RAD_FOVH / 2.0, left_fov_origin_vec);
    Vec3 right_fov_origin_vec;rot(AXIS_Z, AXIS_Y, ORIGIN, RAD_FOVH / 2.0, right_fov_origin_vec);

    rot(left_fov_origin_vec , AXIS_X, ORIGIN, RAD_FOVV / 2.0,ideal_fov_unit_vecs[0]); //leftup
    rot(right_fov_origin_vec, AXIS_X, ORIGIN, RAD_FOVV / 2.0,ideal_fov_unit_vecs[1]); //rightup
    rot(right_fov_origin_vec, AXIS_X, ORIGIN, -1.0 * RAD_FOVV / 2.0,ideal_fov_unit_vecs[2]); //rightdown
    rot(left_fov_origin_vec , AXIS_X, ORIGIN, -1.0 * RAD_FOVV / 2.0,ideal_fov_unit_vecs[3]); //leftdown
    return ideal_fov_unit_vecs;
}

std::vector<Vec3> get_ideal_fov_positions(std::vector<Vec3> ideal_fov_unit_vecs, Vec3& camera_world_position){
    std::vector<Vec3> ideal_fov_positions = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_fov_unit_vecs[0], ideal_fov_positions[0]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_fov_unit_vecs[1], ideal_fov_positions[1]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_fov_unit_vecs[2], ideal_fov_positions[2]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_fov_unit_vecs[3], ideal_fov_positions[3]);
    return ideal_fov_positions;
}

std::vector<Vec3> get_ideal_aruco_unit_vecs(const Vec3& camera_world_position, const std::vector<Vec3>& corner_positions) {
    std::vector<Vec3> ideal_aruco_vecs(4);
    std::vector<Vec3> ideal_aruco_unit_vecs(4);
    for (int i = 0; i < 4; ++i) {
        v3sub(corner_positions[i], camera_world_position, ideal_aruco_vecs[i]);
        v3nrm(ideal_aruco_vecs[i], ideal_aruco_unit_vecs[i]);
    }
    return ideal_aruco_unit_vecs;
}


std::vector<Vec3> get_ideal_aruco_positions(const std::vector<Vec3> ideal_aruco_unit_vecs, const Vec3& camera_world_position) {
    std::vector<Vec3> ideal_aruco_positions = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_aruco_unit_vecs[0], ideal_aruco_positions[0]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_aruco_unit_vecs[1], ideal_aruco_positions[1]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_aruco_unit_vecs[2], ideal_aruco_positions[2]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, ideal_aruco_unit_vecs[3], ideal_aruco_positions[3]);
    return ideal_aruco_positions;
}

std::vector<std::vector<double>> get_ideal_aruco_screen_positions(
    const std::vector<Vec3>& ideal_aruco_positions,
    const std::vector<Vec3>& ideal_fov_positions)
{
    Vec3 a0; v3sub(ideal_fov_positions[0], ideal_fov_positions[1], a0);
    Vec3 a1; v3add(a0, ideal_fov_positions[2], a1);
    Vec3 a;  v3sub(a1, ideal_fov_positions[3], a);
    Vec3 b;  v3sub(ideal_fov_positions[1], ideal_fov_positions[0], b);
    Vec3 c;  v3sub(ideal_fov_positions[3], ideal_fov_positions[0], c);
    Vec3 d;

    std::vector<std::vector<double>> screen_positions(4, std::vector<double>(2, 0.0));

    for (int i = 0; i < 4; ++i) {
        v3sub(ideal_fov_positions[0], ideal_aruco_positions[i], d);

        // 初期値
        double n = 0.5, l = 0.5;
        const double tol = 1e-6;
        const int max_iter = 50;

        for (int iter = 0; iter < max_iter; ++iter) {
            double f1 = a[0] * n * l + b[0] * n + c[0] * l + d[0];
            double f2 = a[1] * n * l + b[1] * n + c[1] * l + d[1];

            double J00 = a[0] * l + b[0];
            double J01 = a[0] * n + c[0];
            double J10 = a[1] * l + b[1];
            double J11 = a[1] * n + c[1];

            double delta_n = 0.0, delta_l = 0.0;
            if (!solve2x2(J00, J01, J10, J11, f1, f2, delta_n, delta_l)) break;

            n += delta_n;
            l += delta_l;

            if (std::sqrt(delta_n * delta_n + delta_l * delta_l) < tol)
                break;
        }

        screen_positions[i][0] = n * WIDTH_PX;
        screen_positions[i][1] = l * HEIGHT_PX;
    }

    return screen_positions;
}

std::vector<Vec3> get_camera_rotate_axis(const std::vector<std::vector<double>> ideal_aruco_screen_positions, const std::vector<cv::Point2f>& corner) {
    std::vector<Vec3> difference_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    std::vector<Vec3> camera_rotate_axis = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    for (int i = 0; i <= 3; i++) {
        difference_vecs[i] = Vec3{corner[i].x - ideal_aruco_screen_positions[i][0], corner[i].y - ideal_aruco_screen_positions[i][1], 0.0};
        v3crs(difference_vecs[i],AXIS_Z,camera_rotate_axis[i]);
        v3nrm(camera_rotate_axis[i],camera_rotate_axis[i]); 
    }
    //棚の向き（y軸正方向に一旦固定）
    return camera_rotate_axis;
}

std::vector<std::vector<double>> get_aruco_screen_positions(
    const std::vector<Vec3> camera_rotate_axis,
    const double rad,
    const Vec3& camera_world_position,
    const std::vector<Vec3> ideal_fov_unit_vecs,
    const std::vector<Vec3> ideal_aruco_positions)
{
    std::vector<Vec3> fov_unit_vecs(4), fov_positions(4);
    std::vector<std::vector<double>> aruco_screen_positions(4, std::vector<double>(2, 0.0));

    auto solve2x2 = [](double J00, double J01, double J10, double J11,
                       double F0, double F1, double& dx, double& dy) {
        double det = J00 * J11 - J01 * J10;
        if (std::fabs(det) < 1e-12) { dx = dy = 0.0; return false; }
        dx = (-F0 * J11 + F1 * J01) / det;
        dy = (-F1 * J00 + F0 * J10) / det;
        return true;
    };

    for (int j = 0; j < 4; ++j) {

        for (int i = 0; i < 4; ++i)
            rot(ideal_fov_unit_vecs[i], camera_rotate_axis[j], CEILING_ORIGIN, rad / 2, fov_unit_vecs[i]);
        for (int i = 0; i < 4; ++i)
            line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, fov_unit_vecs[i], fov_positions[i]);

        Vec3 a0; v3sub(fov_positions[0], fov_positions[1], a0);
        Vec3 a1; v3add(a0, fov_positions[2], a1);
        Vec3 a;  v3sub(a1, fov_positions[3], a);
        Vec3 b;  v3sub(fov_positions[1], fov_positions[0], b);
        Vec3 c;  v3sub(fov_positions[3], fov_positions[0], c);
        Vec3 d;  v3sub(fov_positions[0], ideal_aruco_positions[j], d);

        double n = 0.5, l = 0.5;
        const double tol = 1e-6;
        const int max_iter = 50;

        for (int iter = 0; iter < max_iter; ++iter) {
            double f1 = a[0] * n * l + b[0] * n + c[0] * l + d[0];
            double f2 = a[1] * n * l + b[1] * n + c[1] * l + d[1];

            double J00 = a[0] * l + b[0];
            double J01 = a[0] * n + c[0];
            double J10 = a[1] * l + b[1];
            double J11 = a[1] * n + c[1];

            double delta_n = 0.0, delta_l = 0.0;
            if (!solve2x2(J00, J01, J10, J11, f1, f2, delta_n, delta_l)) break;

            n += delta_n;
            l += delta_l;

            if (std::sqrt(delta_n * delta_n + delta_l * delta_l) < tol) break;
        }

        aruco_screen_positions[j][0] = n * WIDTH_PX;
        aruco_screen_positions[j][1] = l * HEIGHT_PX;
    }

    return aruco_screen_positions;
}

std::vector<double> get_camera_rotate_rad(
    const std::vector<Vec3>& camera_rotate_axis,
    const std::vector<Vec3>& ideal_aruco_positions,
    const Vec3& camera_world_position,
    const std::vector<Vec3>& ideal_fov_unit_vecs,
    const std::vector<cv::Point2f>& corner)
{
    std::vector<double> best_rad(4, 0.0);
    std::vector<double> min_difference(4, 1e9);
    std::vector<std::vector<double>> aruco_screen_positions(4, std::vector<double>(2, 0.0));

    std::vector<double> step_levels = {1e-2, 1e-3, 1e-4, 1e-5, 1e-6};

    double search_range = 1.57;

    for (size_t level = 0; level < step_levels.size(); ++level) {
        double step = step_levels[level];
        for (int i = 0; i < 4; ++i) {
            double start = std::max(0.0, best_rad[i] - search_range);
            double end   = std::min(1.57, best_rad[i] + search_range);

            for (double rad = start; rad <= end; rad += step) {
                auto screen_positions = get_aruco_screen_positions(
                    camera_rotate_axis, rad, camera_world_position,
                    ideal_fov_unit_vecs, ideal_aruco_positions);

                Vec3 difference_vec = {screen_positions[i][0] - corner[i].x,
                             screen_positions[i][1] - corner[i].y, 0.0};
                double difference = v3len(difference_vec);

                if (difference < min_difference[i]) {
                    min_difference[i] = difference;
                    best_rad[i] = rad;
                }
            }
        }
        search_range = step * 10.0;
    }

    return best_rad;
}