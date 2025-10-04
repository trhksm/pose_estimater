#include "../include/cal_calibration.hpp"
#include <Eigen/Dense>

Vec3 get_camera_world_position(const Vec3& shelf_world_position, const Vec3& shelf_rotate_axis ,const double shelf_rotate_rad) {
    Vec3 camera_world_position;
    const Vec3 ideal_camera_world_position = {shelf_world_position[0], shelf_world_position[1], CAMERA_HEIGHT};
    rot(ideal_camera_world_position ,shelf_rotate_axis ,shelf_world_position ,shelf_rotate_rad ,camera_world_position);
    return camera_world_position;
}

std::vector<Vec3> get_ideal_fov_unit_vecs() {
    std::vector<Vec3> ideal_fov_unit_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

    Vec3 left_fov_origin_vec;rot(AXIS_Z, AXIS_Y, ORIGIN, -1.0 * RAD_FOVH / 2.0, left_fov_origin_vec);
    Vec3 right_fov_origin_vec;rot(AXIS_Z, AXIS_Y, ORIGIN, RAD_FOVH / 2.0, right_fov_origin_vec);

    rot(left_fov_origin_vec , AXIS_X, ORIGIN, -1.0 * RAD_FOVV / 2.0,ideal_fov_unit_vecs[0]); //leftup
    rot(right_fov_origin_vec, AXIS_X, ORIGIN, -1.0 * RAD_FOVV / 2.0,ideal_fov_unit_vecs[1]); //rightup
    rot(right_fov_origin_vec, AXIS_X, ORIGIN, RAD_FOVV / 2.0,ideal_fov_unit_vecs[2]); //rightdown
    rot(left_fov_origin_vec , AXIS_X, ORIGIN, RAD_FOVV / 2.0,ideal_fov_unit_vecs[3]); //leftdown
    return ideal_fov_unit_vecs;
}

std::vector<Vec3> get_ideal_fov_positions(std::vector<Vec3> ideal_fov_unit_vecs, Vec3& camera_world_position){
    std::vector<Vec3> ideal_fov_positions = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position,ideal_fov_unit_vecs[0], ideal_fov_positions[0]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position,ideal_fov_unit_vecs[1], ideal_fov_positions[1]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position,ideal_fov_unit_vecs[2], ideal_fov_positions[2]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position,ideal_fov_unit_vecs[3], ideal_fov_positions[3]);
    return ideal_fov_positions;
}

std::vector<Vec3> get_ideal_aruco_unit_vecs(const Vec3& camera_world_position, const std::vector<cv::Point2f>& corner) {
    std::vector<Vec3> ideal_aruco_vecs(4);
    std::vector<Vec3> ideal_aruco_unit_vecs(4);

    for (int i = 0; i < 4; ++i) {
        Vec3 ci = {static_cast<double>(corner[i].x), static_cast<double>(corner[i].y), 0.0};
        v3sub(ci, camera_world_position, ideal_aruco_vecs[i]);
        v3nrm(ideal_aruco_vecs[i], ideal_aruco_unit_vecs[i]);
    }

    return ideal_aruco_unit_vecs;
}


std::vector<Vec3> get_ideal_aruco_positions(const std::vector<Vec3> ideal_aruco_unit_vecs, const Vec3& camera_world_position) {
    std::vector<Vec3> ideal_aruco_positions = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, ideal_aruco_unit_vecs[0], ideal_aruco_positions[0]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, ideal_aruco_unit_vecs[1], ideal_aruco_positions[1]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, ideal_aruco_unit_vecs[2], ideal_aruco_positions[2]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, ideal_aruco_unit_vecs[3], ideal_aruco_positions[3]);
    return ideal_aruco_positions;
}

/*std::vector<std::vector<double>> get_ideal_aruco_screen_positions(const std::vector<Vec3> ideal_aruco_positions, const std::vector<Vec3> ideal_fov_positions) {
    double tol = 1e-4;
    Vec3 a; a = ideal_fov_positions[2];
    Vec3 b;v3sub(ideal_fov_positions[1], ideal_fov_positions[3],b);
    Vec3 c;v3sub(ideal_fov_positions[3], ideal_fov_positions[1],c);
    Vec3 d;
    std::vector<std::vector<double>> ideal_aruco_screen_positions = {{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0}};
    for (int i = 0; i <= 3; i++) {
        v3sub(ideal_fov_positions[0], ideal_aruco_positions[i] ,d);
        bool found = false;
        for (double n = 0; n <= 1; n += 0.001) {
            for (double l = 0; l <= 1; l += 0.001) {
                double eq1 = a[0]*n*l + b[0]*n + c[0]*l + d[0];
                double eq2 = a[1]*n*l + b[1]*n + c[1]*l + d[1];
                if (std::abs(eq1) < tol && std::abs(eq2) < tol) {
                    found = true;
                    ideal_aruco_screen_positions[i][0] = n * WIDTH_PX;
                    ideal_aruco_screen_positions[i][1] = l * HEIGHT_PX;
                    break;
                }
            }
            if (found) break;
        }
    }
    return ideal_aruco_screen_positions;
}
*/
std::vector<std::vector<double>> get_ideal_aruco_screen_positions(
    const std::vector<Vec3>& ideal_aruco_positions,
    const std::vector<Vec3>& ideal_fov_positions) {

    Vec3 a = ideal_fov_positions[2];
    Vec3 b, c, d;
    v3sub(ideal_fov_positions[1], ideal_fov_positions[3], b);
    v3sub(ideal_fov_positions[3], ideal_fov_positions[1], c);

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

            Eigen::Matrix2d J;
            J(0, 0) = a[0] * l + b[0];  // ∂f1/∂n
            J(0, 1) = a[0] * n + c[0];  // ∂f1/∂l
            J(1, 0) = a[1] * l + b[1];  // ∂f2/∂n
            J(1, 1) = a[1] * n + c[1];  // ∂f2/∂l

            Eigen::Vector2d F(f1, f2);
            Eigen::Vector2d delta = J.fullPivLu().solve(-F);

            n += delta[0];
            l += delta[1];

            if (delta.norm() < tol) break;
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
    return camera_rotate_axis;
}

/*std::vector<std::vector<double>> get_aruco_screen_positions(const std::vector<Vec3> camera_rotate_axis, const double degree ,const Vec3& camera_world_position, const std::vector<Vec3> ideal_fov_unit_vecs ,const std::vector<Vec3> ideal_aruco_positions) {
    std::vector<Vec3> fov_unit_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    std::vector<Vec3> fov_positions = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    rot(ideal_fov_unit_vecs[0],camera_rotate_axis[0],CEILING_ORIGIN, degree,fov_unit_vecs[0]);
    rot(ideal_fov_unit_vecs[1],camera_rotate_axis[0],CEILING_ORIGIN, degree,fov_unit_vecs[1]);
    rot(ideal_fov_unit_vecs[2],camera_rotate_axis[0],CEILING_ORIGIN, degree,fov_unit_vecs[2]);
    rot(ideal_fov_unit_vecs[3],camera_rotate_axis[0],CEILING_ORIGIN, degree,fov_unit_vecs[3]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, fov_unit_vecs[0], fov_positions[0]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, fov_unit_vecs[1], fov_positions[1]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, fov_unit_vecs[2], fov_positions[2]);
    line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL , camera_world_position, fov_unit_vecs[3], fov_positions[3]);
    double tol = 1e-4;
    Vec3 a; a = fov_positions[2];
    Vec3 b;v3sub(fov_positions[1], fov_positions[3],b);
    Vec3 c;v3sub(fov_positions[3], fov_positions[1],c);
    Vec3 d;
    std::vector<std::vector<double>> aruco_screen_positions = {{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0}};
    for (int i = 0; i <= 3; i++) {
        v3sub(fov_positions[0], ideal_aruco_positions[i] ,d);
        bool found = false;
        for (double n = 0; n <= 1; n += 0.001) {
            for (double l = 0; l <= 1; l += 0.001) {
                double eq1 = a[0]*n*l + b[0]*n + c[0]*l + d[0];
                double eq2 = a[1]*n*l + b[1]*n + c[1]*l + d[1];
                if (std::abs(eq1) < tol && std::abs(eq2) < tol) {
                    found = true;
                    aruco_screen_positions[i][0] = n * WIDTH_PX;
                    aruco_screen_positions[i][1] = l * HEIGHT_PX;
                    break;
                }
            }
            if (found) break;
        }
    }
    return aruco_screen_positions;
}
*/

std::vector<std::vector<double>> get_aruco_screen_positions(
    const std::vector<Vec3> camera_rotate_axis,
    const double degree,
    const Vec3& camera_world_position,
    const std::vector<Vec3> ideal_fov_unit_vecs,
    const std::vector<Vec3> ideal_aruco_positions) {

    std::vector<Vec3> fov_unit_vecs(4), fov_positions(4);
    for (int i = 0; i < 4; ++i)
        rot(ideal_fov_unit_vecs[i], camera_rotate_axis[0], CEILING_ORIGIN, degree, fov_unit_vecs[i]);
    for (int i = 0; i < 4; ++i)
        line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_world_position, fov_unit_vecs[i], fov_positions[i]);

    Vec3 a = fov_positions[2];
    Vec3 b, c, d;
    v3sub(fov_positions[1], fov_positions[3], b);
    v3sub(fov_positions[3], fov_positions[1], c);

    std::vector<std::vector<double>> aruco_screen_positions(4, std::vector<double>(2, 0.0));

    for (int i = 0; i < 4; ++i) {
        v3sub(fov_positions[0], ideal_aruco_positions[i], d);

        double n = 0.5, l = 0.5;  // 初期値
        const double tol = 1e-6;
        const int max_iter = 50;

        for (int iter = 0; iter < max_iter; ++iter) {
            double f1 = a[0] * n * l + b[0] * n + c[0] * l + d[0];
            double f2 = a[1] * n * l + b[1] * n + c[1] * l + d[1];

            Eigen::Matrix2d J;
            J(0, 0) = a[0] * l + b[0];  // ∂f1/∂n
            J(0, 1) = a[0] * n + c[0];  // ∂f1/∂l
            J(1, 0) = a[1] * l + b[1];  // ∂f2/∂n
            J(1, 1) = a[1] * n + c[1];  // ∂f2/∂l

            Eigen::Vector2d F(f1, f2);
            Eigen::Vector2d delta = J.fullPivLu().solve(-F);

            n += delta[0];
            l += delta[1];

            if (delta.norm() < tol) break;
        }

        aruco_screen_positions[i][0] = n * WIDTH_PX;
        aruco_screen_positions[i][1] = l * HEIGHT_PX;
    }

    return aruco_screen_positions;
}

double get_camera_rotate_degree(const std::vector<Vec3> camera_rotate_axis, const std::vector<Vec3>ideal_aruco_positions, const Vec3& camera_world_position, const std::vector<Vec3> ideal_fov_unit_vecs ,const std::vector<cv::Point2f>& corner) {
    double camera_rotate_degree = 0.0;
    double difference = 100000000.0;
    std::vector<std::vector<double>> aruco_screen_positions = {{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0}};
    std::vector<Vec3> differences = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    for (double d = 0.0; d <= 90.0; d += 0.001) {
        aruco_screen_positions = get_aruco_screen_positions(camera_rotate_axis, d , camera_world_position, ideal_fov_unit_vecs , ideal_aruco_positions);
        for (int i = 0; i <= 3; i++) {
            differences[i][0] = aruco_screen_positions[i][0] - corner[i].x;
            differences[i][1] = aruco_screen_positions[i][1] - corner[i].y;
        }
        double next_difference =  v3len(differences[0]) + v3len(differences[1]) + v3len(differences[2]) + v3len(differences[3]);
        if (difference < next_difference) {
            camera_rotate_degree = d;
            break;
        } else {
            difference = next_difference;
        } 
    }
    return camera_rotate_degree;
}

