#ifndef CAL_CALIBRATION_HPP
#define CAL_CALIBRATION_HPP
#include "cal_base.hpp"
#include "opencv2/opencv.hpp"

const Vec3 AXIS_X = {1.0, 0.0, 0.0};
const Vec3 AXIS_Y = {0.0, 1.0, 0.0};
const Vec3 AXIS_Z = {0.0, 0.0, 1.0};
const Vec3 ORIGIN = {0.0, 0.0, 0.0};

const double CAMERA_HEIGHT = 0.9;
const double RAD_FOVH = 81.0 * M_PI / 180.0;
const double RAD_FOVV = 48.60 * M_PI / 180.0;

const Vec3 CEILING_ORIGIN = {0.0, 0.0, 0.0};
const Vec3 CEILING_NORMAL = {0.0, 0.0, -1.0};

const int WIDTH_PX = 1280;
const int HEIGHT_PX = 720;

Vec3 get_camera_world_position(const Vec3& shelf_world_position, const Vec3& shelf_rotate_axis ,const double shelf_rotate_rad);
std::vector<Vec3> get_ideal_fov_unit_vecs();
std::vector<Vec3> get_ideal_fov_positions(std::vector<Vec3> ideal_fov_unit_vecs, Vec3& camera_world_position);
std::vector<Vec3> get_ideal_aruco_unit_vecs(const Vec3& camera_world_position,const std::vector<Vec3>& corner_positions);
std::vector<Vec3> get_ideal_aruco_positions(const std::vector<Vec3> ideal_aruco_unit_vecs, const Vec3& camera_world_position);
std::vector<std::vector<double>> get_ideal_aruco_screen_positions(const std::vector<Vec3>& ideal_aruco_positions, const std::vector<Vec3>& ideal_fov_positions);
std::vector<Vec3> get_camera_rotate_axis(const std::vector<std::vector<double>> ideal_aruco_screen_positions, const std::vector<cv::Point2f>& corner);
std::vector<std::vector<double>> get_aruco_screen_positions(const std::vector<Vec3> camera_rotate_axis, const double degree ,const Vec3& camera_world_position, const std::vector<Vec3> ideal_fov_unit_vecs ,const std::vector<Vec3> ideal_aruco_positions);
double get_camera_rotate_rad(const std::vector<Vec3>& camera_rotate_axis, const std::vector<Vec3>& ideal_aruco_positions, const Vec3& camera_world_position, const std::vector<Vec3>& ideal_fov_unit_vecs ,const std::vector<cv::Point2f>& corner);
#endif