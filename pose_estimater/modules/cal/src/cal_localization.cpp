#include "../include/cal_localization.hpp"
#include "../include/cal_base.hpp"

Vec3 get_camera_pose(const Vec3& calibration_camera_pose, const Vec3& calibration_shelf_pose, const Vec3& shelf_pose){
    Vec3 camera_pose;
    Vec3 calibration_shelf_related_position;  v3mul(SHELF_HEIGHT - CAMERA_HEIGHT, calibration_shelf_pose, calibration_shelf_related_position);
    Vec3 calibration_camera_related_position; v3add(calibration_shelf_related_position, calibration_camera_pose, calibration_camera_related_position);
    Vec3 shelf_related_position;              v3mul(SHELF_HEIGHT - CAMERA_HEIGHT, shelf_pose, shelf_related_position);
    
    Vec3 calibration_shelf_axis;
    v3crs(AXIS_Z, calibration_shelf_pose, calibration_shelf_axis);
    v3nrm(calibration_shelf_axis,calibration_shelf_axis);
    double calibration_shelf_rad = std::acos(v3dot(AXIS_Z, calibration_shelf_pose));

    Vec3 shelf_axis;
    v3crs(AXIS_Z, shelf_pose, shelf_axis);
    v3nrm(shelf_axis,shelf_axis);
    double shelf_rad = std::acos(v3dot(AXIS_Z, shelf_pose));
    
    rot(calibration_camera_related_position, calibration_shelf_axis, ORIGIN, calibration_shelf_rad, camera_pose);
    rot(camera_pose, shelf_axis, ORIGIN, shelf_rad, camera_pose);
    v3sub(camera_pose, shelf_related_position, camera_pose);
    return camera_pose;
}

Vec3 get_camera_rotate_axis(const Vec3& camera_pose){
    Vec3 camera_rotate_axis;
    v3crs(AXIS_Z, camera_pose, camera_rotate_axis);
    v3nrm(camera_rotate_axis,camera_rotate_axis);
    return camera_rotate_axis;
}

double get_camera_rotate_rad(const Vec3& camera_pose){
    double camera_rotate_rad = std::acos(v3dot(AXIS_Z, camera_pose));
    return camera_rotate_rad;
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

std::vector<Vec3> get_fov_vecs(const std::vector<Vec3>& ideal_fov_unit_vecs, const  Vec3& camera_rotate_axis, const double& camera_rotate_rad){
    std::vector<Vec3> fov_unit_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    std::vector<Vec3> fov_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    Vec3 camera_related_position = {0.0,0.0,-1 * CAMERA_HEIGHT};
    
    for(int i = 0;i < 4;++i){
        rot(ideal_fov_unit_vecs[i], camera_rotate_axis, ORIGIN, camera_rotate_rad, fov_unit_vecs[i]);
        line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_related_position, fov_unit_vecs[i], fov_vecs[i]);
    }
    return fov_vecs;
}

std::vector<Vec3> get_camera_to_aruco_vecs(const std::vector<cv::Point2f>& corners, const std::vector<Vec3>& fov_vecs){
    double n = 0.0;
    double l = 0.0;
    std::vector<Vec3> camera_to_aruco_vecs  = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

    for(int i = 0;i < 4;i++){
        n = corners[i].x / WIDTH_PX;
        l = corners[i].y / HEIGHT_PX;
        camera_to_aruco_vecs[i][0] = (1-n)*(1-l)*fov_vecs[0][0] + n * (1-l) * fov_vecs[1][0]+ n * l * fov_vecs[2][0] + (1-n) * l * fov_vecs[3][0]; //まだ回転は考慮しない
        camera_to_aruco_vecs[i][1] = (1-n)*(1-l)*fov_vecs[0][1] + n * (1-l) * fov_vecs[1][1]+ n * l * fov_vecs[2][1] + (1-n) * l * fov_vecs[3][1]; //まだ回転は考慮しない
        camera_to_aruco_vecs[i][2] = (1-n)*(1-l)*fov_vecs[0][2] + n * (1-l) * fov_vecs[1][2]+ n * l * fov_vecs[2][2] + (1-n) * l * fov_vecs[3][2]; //まだ回転は考慮しない・おそらく不必要
    }
    return camera_to_aruco_vecs;
}

std::vector<Vec3> get_camera_world_positions(const std::vector<Vec3>& camera_to_aruco_vecs,  const std::vector<Vec3>& aruco_world_positions){
    std::vector<Vec3> camera_world_positions  = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    for(int i = 0;i < 4;i++){
        v3sub(aruco_world_positions[i], camera_to_aruco_vecs[i], camera_world_positions[i]);
    }
    return camera_world_positions;
}