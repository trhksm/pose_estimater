#include "../include/cal_calibration.hpp"

void get_camera_world_position(const Vec3& shelf_world_position,const Vec3& shelf_rotate_axis,const double shelf_rotate_rad,Vec3& camera_world_position) {
    const Vec3 ideal_camera_world_position = [shelf_world_position.x, shelf_world_position.y, CAMERA_HEIGHT];
    rot(ideal_camera_world_position,shelf_rotate_axis,shelf_world_position,degree,camera_world_position);
}
void get_ideal_fov_unit_vec() {}
void get_ideal_aruco_unit_vec() {}
void get_ideal_aruco_screen_position() {}
void get_camera_rotate_axis() {}
void get_camera_rotate_degree() {}
void get_camera_calibration_info() {}
