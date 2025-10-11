#ifndef CAL_LOCALIZATION_HPP
#define CAL_LOCALIZATION_HPP
#include <vector>
#include <cmath>
#include <opencv2/core.hpp>

Vec3 get_camera_pose(const Vec3& calibration_camera_pose, const Vec3& calibration_shelf_pose, const Vec3& shelf_pose);
Vec3 get_camera_rotate_axis(const Vec3& camera_pose);
double get_camera_rotate_rad(const Vec3& camera_pose);
std::vector<Vec3> get_ideal_fov_unit_vecs();
std::vector<Vec3> get_fov_vecs(const std::vector<Vec3>& ideal_fov_unit_vecs, const  Vec3& camera_rotate_axis, const double& camera_rotate_rad);
std::vector<Vec3> get_camera_to_aruco_vecs(const std::vector<cv::Point2f>& corner, const std::vector<Vec3>& fov_vecs);
std::vector<Vec3> get_camera_world_positions(const std::vector<Vec3>& camera_to_aruco_vecs,  const std::vector<Vec3>& aruco_world_positions);
#endif