#include "../include/cal_localization.hpp"
#include "../include/cal_base.hpp"

/*Vec3 get_camera_pose(const Vec3& calibration_camera_pose, const Vec3& calibration_shelf_pose, const Vec3& shelf_pose){
    //それぞれ絶対角であることに留意
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
    
    rot(calibration_camera_related_position, calibration_shelf_axis, ORIGIN, -1 * calibration_shelf_rad, camera_pose);//キャリブレーションのときかたむいていなかったら
    rot(camera_pose, shelf_axis, ORIGIN, shelf_rad, camera_pose);//今の傾き分傾ける
    v3sub(camera_pose, shelf_related_position, camera_pose);//カメラの傾きを出す
    return camera_pose;
}*/

Vec3 get_camera_pose(const Vec3& calibration_camera_pose, const Vec3& calibration_shelf_pose, const Vec3& shelf_pose, const double& yaw){
    Vec3 camera_pose;
    
    Vec3 calibration_shelf_related_position;  v3mul(SHELF_HEIGHT - CAMERA_HEIGHT, calibration_shelf_pose, calibration_shelf_related_position);
    Vec3 calibration_camera_related_position; v3add(calibration_shelf_related_position, calibration_camera_pose, calibration_camera_related_position);
    Vec3 shelf_related_position;              v3mul(SHELF_HEIGHT - CAMERA_HEIGHT, shelf_pose, shelf_related_position);
    
    Vec3 calibration_shelf_axis;
    v3crs(AXIS_Z, calibration_shelf_pose, calibration_shelf_axis);
    double len = std::sqrt(calibration_shelf_axis[0]*calibration_shelf_axis[0] +
                           calibration_shelf_axis[1]*calibration_shelf_axis[1] +
                           calibration_shelf_axis[2]*calibration_shelf_axis[2]);
    if(len < 1e-8) {
        calibration_shelf_axis = {1.0, 0.0, 0.0};
        std::cout << "miss for axis";
    } else {
        v3nrm(calibration_shelf_axis, calibration_shelf_axis);
    }
    double dot_val = v3dot(AXIS_Z, calibration_shelf_pose);
    dot_val = std::max(-1.0, std::min(1.0, dot_val));
    double calibration_shelf_rad = std::acos(dot_val);
    
    Vec3 shelf_axis;
    v3crs(AXIS_Z, shelf_pose, shelf_axis);
    len = std::sqrt(shelf_axis[0]*shelf_axis[0] + shelf_axis[1]*shelf_axis[1] + shelf_axis[2]*shelf_axis[2]);
    if(len < 1e-8) {
        shelf_axis = {1.0, 0.0, 0.0};
    } else {
        v3nrm(shelf_axis, shelf_axis);
    }
    dot_val = v3dot(AXIS_Z, shelf_pose);
    dot_val = std::max(-1.0, std::min(1.0, dot_val));
    double shelf_rad = std::acos(dot_val);
    
    rot(calibration_camera_related_position, calibration_shelf_axis, ORIGIN, -1.0 * calibration_shelf_rad, camera_pose);
    rot(camera_pose, shelf_axis, ORIGIN, shelf_rad, camera_pose);
    v3sub(camera_pose, shelf_related_position, camera_pose);
    //shelf base -> world base
    rot(camera_pose,AXIS_Z,ORIGIN,yaw,camera_pose);
    v3nrm(camera_pose,camera_pose);
    return camera_pose;
}

Vec3 get_camera_rotate_axis(const Vec3& camera_pose){
    Vec3 camera_rotate_axis = {1.0,0.0,0.0};
    if(camera_pose[2] != 1.0)v3crs(AXIS_Z, camera_pose, camera_rotate_axis);
    v3nrm(camera_rotate_axis,camera_rotate_axis);
    return camera_rotate_axis;
}

double get_camera_rotate_rad(const Vec3& camera_pose){
    double dot = v3dot(AXIS_Z, camera_pose);
    dot = std::clamp(dot, -1.0, 1.0);
    return std::acos(dot);
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

std::vector<Vec3> get_agv_ideal_fov_unit_vecs() {
    std::vector<Vec3> ideal_fov_unit_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

    Vec3 left_fov_origin_vec;rot(AXIS_Z, AXIS_Y, ORIGIN, -1.0 * AGV_RAD_FOVH / 2.0, left_fov_origin_vec);
    Vec3 right_fov_origin_vec;rot(AXIS_Z, AXIS_Y, ORIGIN, AGV_RAD_FOVH / 2.0, right_fov_origin_vec);

    rot(left_fov_origin_vec , AXIS_X, ORIGIN, AGV_RAD_FOVV / 2.0,ideal_fov_unit_vecs[0]); //leftup
    rot(right_fov_origin_vec, AXIS_X, ORIGIN, AGV_RAD_FOVV / 2.0,ideal_fov_unit_vecs[1]); //rightup
    rot(right_fov_origin_vec, AXIS_X, ORIGIN, -1.0 * AGV_RAD_FOVV / 2.0,ideal_fov_unit_vecs[2]); //rightdown
    rot(left_fov_origin_vec , AXIS_X, ORIGIN, -1.0 * AGV_RAD_FOVV / 2.0,ideal_fov_unit_vecs[3]); //leftdown
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

std::vector<Vec3> get_fov_vecs(const std::vector<Vec3>& ideal_fov_unit_vecs, const  Vec3& camera_rotate_axis, const double& camera_rotate_rad,const double& yaw){
    std::vector<Vec3> fov_unit_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    std::vector<Vec3> fov_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    Vec3 camera_related_position = {0.0,0.0,-1 * CAMERA_HEIGHT};
    
    for(int i = 0;i < 4;++i){
        rot(ideal_fov_unit_vecs[i], AXIS_Z            , ORIGIN, yaw + PI / 2.0, fov_unit_vecs[i]);
        rot(fov_unit_vecs[i]      , camera_rotate_axis, ORIGIN, camera_rotate_rad           , fov_unit_vecs[i]);// order check
        line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, camera_related_position, fov_unit_vecs[i], fov_vecs[i]);
    }
    return fov_vecs;
}

std::vector<Vec3> get_agv_fov_vecs(const std::vector<Vec3>& ideal_fov_unit_vecs, const  Vec3& agv_rotate_axis, const double& agv_rotate_rad,const double& yaw){
    std::vector<Vec3> agv_fov_unit_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    std::vector<Vec3> agv_fov_vecs = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    Vec3 agv_related_position = {0.0,0.0,-1 * AGV_HEIGHT};
    
    for(int i = 0;i < 4;++i){
        rot(ideal_fov_unit_vecs[i], AXIS_Z            , ORIGIN, yaw + PI / 2.0, agv_fov_unit_vecs[i]);
        rot(agv_fov_unit_vecs[i]      , agv_rotate_axis, ORIGIN, agv_rotate_rad           , agv_fov_unit_vecs[i]);// order check
        line_plane_intersection(CEILING_ORIGIN, CEILING_NORMAL, agv_related_position, agv_fov_unit_vecs[i], agv_fov_vecs[i]);
    }
    return agv_fov_vecs;
}

std::vector<std::vector<Vec3>> get_camera_to_aruco_vecs(const std::vector<std::vector<cv::Point2f>>& corners, const std::vector<Vec3>& fov_vecs){
    double n = 0.0;
    double l = 0.0;
    std::vector<std::vector<Vec3>> camera_to_aruco_vecs(corners.size(), std::vector<Vec3>(4, Vec3{0.0, 0.0, 0.0}));
    for(int i = 0;i < corners.size();i++){
        for(int j = 0;j < 4;j++){
            n = corners[i][j].x / WIDTH_PX;
            l = corners[i][j].y / HEIGHT_PX;
            camera_to_aruco_vecs[i][j][0] = (1-n)*(1-l)*fov_vecs[0][0] + n * (1-l) * fov_vecs[1][0]+ n * l * fov_vecs[2][0] + (1-n) * l * fov_vecs[3][0]; //まだ回転は考慮しない
            camera_to_aruco_vecs[i][j][1] = (1-n)*(1-l)*fov_vecs[0][1] + n * (1-l) * fov_vecs[1][1]+ n * l * fov_vecs[2][1] + (1-n) * l * fov_vecs[3][1]; //まだ回転は考慮しない
            camera_to_aruco_vecs[i][j][2] = (1-n)*(1-l)*fov_vecs[0][2] + n * (1-l) * fov_vecs[1][2]+ n * l * fov_vecs[2][2] + (1-n) * l * fov_vecs[3][2]; //まだ回転は考慮しない・おそらく不必要
        }
    }
    return camera_to_aruco_vecs;
}

Vec3 get_camera_world_positions(
    const std::vector<std::vector<Vec3>>& camera_to_aruco_vecs,
    const std::vector<std::pair<std::vector<Vec3>,int>>& pairs_aruco_corners_positions_and_index)
{
    Vec3 sum_all{0.0, 0.0, 0.0};
    size_t total_count = pairs_aruco_corners_positions_and_index.size();

    for (size_t i = 0; i < total_count; i++) {
        int index = pairs_aruco_corners_positions_and_index[i].second;
        Vec3 sum{0.0, 0.0, 0.0};
        for (int j = 0; j < 4; j++) {
            Vec3 diff;
            v3sub(pairs_aruco_corners_positions_and_index[i].first[j],
                  camera_to_aruco_vecs[index][j],
                  diff);
            sum[0] += diff[0];
            sum[1] += diff[1];
            sum[2] += diff[2];
        }
        // 1つのマーカーあたり4コーナーの重み付き平均に変更 check
        Vec3 avg = { sum[0]/4, sum[1]/4, sum[2]/4 };
        sum_all[0] += avg[0];
        sum_all[1] += avg[1];
        sum_all[2] += avg[2];
    }

    // 全マーカー分の平均位置を返す 上記
    if (total_count > 0) {
        sum_all[0] /= total_count;
        sum_all[1] /= total_count;
        sum_all[2] /= total_count;
    }
    return sum_all;
}


Vec3 get_shelf_pose(const double pitch,const double roll,const double calibration_pitch, const double calibration_roll){
    Vec3 shelf_pose;
    rot(AXIS_Z    , AXIS_X, ORIGIN, roll + calibration_roll  , shelf_pose);
    rot(shelf_pose, AXIS_Y, ORIGIN, pitch + calibration_pitch, shelf_pose);
    v3nrm(shelf_pose,shelf_pose);//
    return shelf_pose;
}
Vec3 get_agv_pose(const double pitch,const double roll,const double calibration_pitch,const double calibration_roll,const double yaw){
    Vec3 agv_pose;
    rot(AXIS_Z   , AXIS_X, ORIGIN, roll + calibration_roll  , agv_pose);
    rot(agv_pose , AXIS_Y, ORIGIN, pitch + calibration_pitch, agv_pose);
    rot(agv_pose , AXIS_Z, ORIGIN, yaw                      , agv_pose);
    v3nrm(agv_pose,agv_pose);
    return agv_pose;
}

Vec3 get_calibration_shelf_pose(const double calibration_pitch, const double calibration_roll){
    Vec3 calibration_shelf_pose;
    rot(AXIS_Z                , AXIS_X, ORIGIN, calibration_roll     , calibration_shelf_pose);
    rot(calibration_shelf_pose, AXIS_Y, ORIGIN, calibration_pitch    , calibration_shelf_pose);
    v3nrm(calibration_shelf_pose,calibration_shelf_pose);
    return calibration_shelf_pose;
}

Vec3 get_calibration_agv_pose(const double calibration_pitch,const double calibration_roll){
    Vec3 calibration_agv_pose;
    rot(AXIS_Z                 , AXIS_X, ORIGIN, calibration_roll   , calibration_agv_pose);
    rot(calibration_agv_pose   , AXIS_Y, ORIGIN, calibration_pitch  , calibration_agv_pose);
    v3nrm(calibration_agv_pose,calibration_agv_pose);
    return calibration_agv_pose;
}


double get_yaw(const std::vector<std::vector<cv::Point2f>> &corners) {
    std::vector<Vec3> x_vecs;
    Vec3 x_vec{0.0, 0.0, 0.0};
    Vec3 x_axis{0.0, -1.0, 0.0};
    double yaw_rad = 0.0;

    for (const auto &corner_set : corners) {
        if (corner_set.size() < 4) continue;
        Vec3 v1{
            corner_set[0].x - corner_set[3].x,
            corner_set[0].y - corner_set[3].y,
            0.0
        };
        Vec3 v2{
            corner_set[1].x - corner_set[2].x,
            corner_set[1].y - corner_set[2].y,
            0.0
        };

        x_vecs.push_back(v1);
        x_vecs.push_back(v2);
    }

    for (const auto &v : x_vecs) {
        v3add(x_vec,v,x_vec);
    }
    v3nrm(x_vec,x_vec);
    yaw_rad = std::acos(v3dot(x_vec,x_axis));
    if (x_vec[0] > 0) yaw_rad *= -1.0;

    return yaw_rad;
}

Vec3 get_shelf_agv_world_position(const Vec3& camera_world_positions, const Vec3& shelf_pose){
    Vec3 agv_world_position;
    Vec3 shelf_related_position;  v3mul(SHELF_HEIGHT - CAMERA_HEIGHT, shelf_pose, shelf_related_position);
    v3sub(camera_world_positions,shelf_related_position,agv_world_position);
    return agv_world_position;
}

Vec3 get_agv_world_positions(const Vec3& camera_world_positions,const double& yaw){
    Vec3 agv_world_positions;
    Vec3 camera_to_agv = {0.0,0.03,0.0};
    rot(camera_to_agv,AXIS_Z,ORIGIN,yaw,camera_to_agv);
    v3add(camera_world_positions,camera_to_agv,agv_world_positions);
    return agv_world_positions;
}