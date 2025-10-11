# 計算用モジュール

## cal_base
quaternionと三次元ベクトルの定義
### rot rot0
quaternionを用いた三次元座標での回転
### distance_surface
三次元ベクトルを用いた面と線の交点導出
### foot_perpendicular
三次元ベクトルを用いた垂線の足導出
### line_plane_intersection
三次元ベクトルを用いた面と線の交点導出 

## cal_calibration
キャリブレーション用の計算
### get_camera_world_position
棚の傾きから、<br>カメラのワールド座標導出
### get_ideal_fov_unit_vecs
視野角から、<br>カメラが傾いていない場合のカメラの視野の四隅方向の単位ベクトル導出
### get_ideal_fov_positions
カメラが傾いていない場合のカメラの視野の四隅方向の単位ベクトルから、<br>カメラが傾いていない場合のカメラが映す天井の範囲の四隅の座標導出
### get_ideal_aruco_unit_vecs
ArUcoマーカーとカメラのワールド座標から、<br>カメラが傾いていない場合のカメラからArUcoマーカー四隅への単位ベクトル導出
### get_ideal_aruco_positions
カメラが傾いていない場合のカメラからArUcoマーカー四隅への単位ベクトルとカメラのワールド座標から、<br>カメラが傾いていない場合のArUcoマーカーのワールド座標導出（不必要かも）
### get_ideal_aruco_screen_positions
カメラが傾いていない場合のArUcoマーカーのワールド座標とカメラが映す天井の範囲の四隅の座標から、<br>カメラが傾いていない場合のArUcoマーカーのスクリーン座標導出
### get_camera_rotate_axis
 カメラが傾いていない場合のArUcoマーカーのスクリーン座標と実際のスクリーン座標から、<br>四隅それぞれのズレ方向の回転軸を導出
### get_aruco_screen_positions
回転軸と角度とカメラのワールド座標と傾いていない場合のカメラの視野の四隅方向の単位ベクトルとArUcoマーカーのワールド座標から、<br>ある角度傾いたときのArUcoマーカーのスクリーン座標導出
### get_camera_rotate_rad
get_aruco_screen_positionsをつかって導出したArUcoマーカーのスクリーン座標と実際のスクリーン座標が近くなる角度を四隅それぞれ導出

## cal_localization
自己位置推定用の計算
### get_camera_pose
キャリブレーション時のカメラと棚の方向ベクトルと現在の棚の方向ベクトルから、<br>現在のカメラの方向ベクトル導出
### get_camera_rotate_aixs
現在のカメラの方向ベクトルから、<br>現在のカメラの回転軸導出
### get_camera_rotate_rad
現在のカメラの方向ベクトルから、<br>現在のカメラの傾き導出
### get_ideal_fov_unit_vecs
視野角から、<br>カメラが傾いていない場合のカメラの視野の四隅方向の単位ベクトル導出
### get_fov_vecs
カメラの回転軸と傾きから、<br>カメラの視野の四隅方向でカメラから天井を指すように長さを調節したベクトル導出
### get_camera_to_aruco_vecs
ArUcoマーカーの四隅のスクリーン座標とカメラの視野の四隅方向でカメラから天井を指すように長さを調節したベクトルから、<br>カメラからArUcoマーカーの四隅を指すベクトル導出
### get_camera_world_positions
カメラからArUcoマーカーの四隅を指すベクトルとArUcoマーカーのワールド座標から、<br>カメラのワールド座標導出