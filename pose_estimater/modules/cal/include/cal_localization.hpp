#ifndef CAL_LOCALIZATION_HPP
#define CAL_LOCALIZATION_HPP
#include <vector>
#include <cmath>
#include <numbers>
#include <opencv2/core.hpp>
#include "cal_base.hpp"

const Vec3 AXIS_X = {1.0, 0.0, 0.0};
const Vec3 AXIS_Y = {0.0, 1.0, 0.0};
const Vec3 AXIS_Z = {0.0, 0.0, 1.0};
const Vec3 ORIGIN = {0.0, 0.0, 0.0};

const double PI = 3.141592653589793;
const double CAMERA_HEIGHT = 0.9;
const double SHELF_HEIGHT = 2.3;
const double AGV_HEIGHT = 1.9;
const double RAD_FOVH = 80.22 * M_PI / 180.0;
const double RAD_FOVV = 57.80 * M_PI / 180.0;
const double AGV_RAD_FOVH = 39.1 * M_PI / 180.0;
const double AGV_RAD_FOVV = 22.8 * M_PI / 180.0;
const Vec3 CEILING_ORIGIN = {0.0, 0.0, 0.0};
const Vec3 CEILING_NORMAL = {0.0, 0.0, -1.0};

const int WIDTH_PX = 1280;
const int HEIGHT_PX = 720;

Vec3 get_camera_pose(const Vec3& calibration_camera_pose, const Vec3& calibration_shelf_pose, const Vec3& shelf_pose,const double& yaw);
Vec3 get_camera_rotate_axis(const Vec3& camera_pose);
double get_camera_rotate_rad(const Vec3& camera_pose);
std::vector<Vec3> get_ideal_fov_unit_vecs();
std::vector<Vec3> get_agv_ideal_fov_unit_vecs();
std::vector<Vec3> get_fov_vecs(const std::vector<Vec3>& ideal_fov_unit_vecs, const  Vec3& camera_rotate_axis, const double& camera_rotate_rad,const double& yaw);
std::vector<Vec3> get_agv_fov_vecs(const std::vector<Vec3>& ideal_fov_unit_vecs, const  Vec3& agv_rotate_axis, const double& agv_rotate_rad,const double& yaw);
std::vector<std::vector<Vec3>> get_camera_to_aruco_vecs(const std::vector<std::vector<cv::Point2f>>& corners, const std::vector<Vec3>& fov_vecs);
Vec3 get_camera_world_positions(const std::vector<std::vector<Vec3>>& camera_to_aruco_vecs,  const std::vector<std::pair<std::vector<Vec3>,int>>& pairs_aruco_corners_positions_and_index);

Vec3 get_calibration_shelf_pose(const double calibration_pitch, const double calibration_roll);
Vec3 get_calibration_agv_pose(const double calibration_pitch,const double calibration_roll);
Vec3 get_shelf_pose(const double pitch,const double roll,const double calibration_pitch, const double calibration_roll);
Vec3 get_agv_pose(const double pitch,const double roll,const double calibration_pitch,const double calibration_roll, const double yaw);
double get_yaw(const std::vector<std::vector<cv::Point2f>>& corners);

Vec3 get_shelf_agv_world_positions(const Vec3& camera_world_positions, const Vec3& shelf_pose);
Vec3 get_agv_world_positions(const Vec3& camera_world_positions,const double& yaw);
#endif