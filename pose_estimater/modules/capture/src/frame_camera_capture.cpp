#include "../include/frame_camera_capture.hpp"

namespace Modules::PoseEstimation {

FrameCameraCapture::FrameCameraCapture(const char *devname, int img_w, int img_h,  //コンストラクタ作成
    int fourcc_code, const char *param_filename,
    int fps, int buf_size)
    : FrameCaptureBase(devname, img_w, img_h, fourcc_code, param_filename, fps, buf_size),  //親クラス初期化
      have_read_(false), end_flag_(false) {  //メンバ変数初期化
  // read xml of camera geometry parameters
  cv::Size img_size(img_w_, img_h_); //画像の立てかけ横
  cv::FileStorage fs(param_filename, cv::FileStorage::READ); //データの読み込み
  if (!fs.isOpened()) {  //FileStrageの読み込みができない
    _log_error("Failed to open param file: %s", param_filename); //エラーログ
    throw std::runtime_error(std::string("Fail to Open : ") + param_filename);  //警告
  }
  fs["intrinsic"] >> geometry_param_.K; //intrinsic読み込み
  fs["distortion"] >> geometry_param_.D; //distorion読み込み
  fs.release(); //メモリ解放
  _log_debug("Loaded camera parameters from: %s", param_filename); //読み込みファイル確認ログ

  // setup video capture
  cap_ = cv::VideoCapture(devname, cv::CAP_V4L2); //カメラのオブジェクト(Linux)
  if (!cap_.isOpened()) { //開けない場合
    _log_error("Failed to open camera device: %s", devname);
    throw std::runtime_error(
        std::string("VideoCapture Setup Error (Fail to Open) : ") + devname);
  }
  cap_.set(cv::CAP_PROP_FOURCC, fourcc_code); //動画コーデック(圧縮方式)
  cap_.set(cv::CAP_PROP_FPS, fps); //フレームレート
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, img_w_); //画像の横ピクセル数
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, img_h_); //画像の縦ピクセル数
  _log_info("Camera capture initialized: %s (%dx%d @ %d fps)", devname, img_w_, img_h_, fps);
  cap_.set(cv::CAP_PROP_BUFFERSIZE, buf_size); //内部バッファのサイズ(遅延性)

  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); //自動露出
  cap_.set(cv::CAP_PROP_EXPOSURE, 100); //カメラの露出時間

  capture_th_ = std::thread(&FrameCameraCapture::process, this); //別スレッドでprocess実行
}

FrameCameraCapture::~FrameCameraCapture() {
  end_flag_ = true;
  capture_th_.join(); //別スレッド終了まで待機
}

bool FrameCameraCapture::is_opened() const {
  return cap_.isOpened();
}

void FrameCameraCapture::process() {
  while (!end_flag_) {
    cap_.grab(); //フレームをキャプチャ
    {
      std::lock_guard<std::mutex> lk(cap_buf_mutex_); lkがスコープを抜けるまで重複しないようにロック
      cap_.retrieve(cap_buf_); //内部バッファをコピー・参照
      have_read_ = true;
      capture_time_point_ = std::chrono::steady_clock::now(); //時間記録
    }
    capture_condvar_.notify_all(); //待っている全スレッドに条件変更を通知
  }
}

bool FrameCameraCapture::is_readable() const {
  return have_read_;
}

bool FrameCameraCapture::read(cv::Mat &img, std::chrono::steady_clock::time_point &tp) {
  if (!have_read_) //一度読み込み
    return false;
  std::unique_lock<std::mutex> lk(cap_buf_mutex_); //手動で変更可能ロック
  cv::Mat dist_img = cap_buf_;
  undistort(dist_img, img); //distortを直す
  have_read_ = false;
  tp = capture_time_point_; //時間保存
  return true;
}

void FrameCameraCapture::undistort(const cv::Mat &dist_img, cv::Mat &undist_img) {
  cv::undistort(dist_img, undist_img, geometry_param_.K, geometry_param_.D);//カメラ行列と歪み係数で直す K,D要確認
}

bool FrameCameraCapture::read_until(cv::Mat &img, std::chrono::steady_clock::time_point &tp,
    int timeout_ms) { 
  std::unique_lock<std::mutex> lk(cap_buf_mutex_);
  if (!capture_condvar_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
          [this] { return have_read_.load(); })) //アンロックして待機 タイムアウトした場合
    return false;
  cv::Mat dist_img = cap_buf_;
  undistort(dist_img, img);
  have_read_ = false;
  tp = capture_time_point_;
  return true;
}

int FrameCameraCapture::width() const {
  return img_w_;
}

int FrameCameraCapture::height() const {
  return img_h_;
}

FisheyeFrameCameraCapture::FisheyeFrameCameraCapture(const char *devname, int img_w,
    int img_h, int fourcc_code,
    const char *param_filename,
    double cropping_rate, int fps,
    int buf_size) //魚眼カメラ
    : FrameCameraCapture(devname, img_w, img_h, fourcc_code, param_filename, fps, buf_size) {
  // fisheye(n�cn_�k~Uf������	�Y�
  cv::Size img_size(width(), height());
  cv::Mat pre_K = geometry_param_.K.clone();
  cv::Mat mapR = cv::Mat::eye(3, 3, CV_64F);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      pre_K, geometry_param_.D, img_size, mapR, geometry_param_.K,
      cropping_rate); //元カメラのK,Dから理想的なKを計算
  geometry_param_.K.at<double>(0, 2) = img_size.width / 2.0;
  geometry_param_.K.at<double>(1, 2) = img_size.height / 2.0;
  cv::fisheye::initUndistortRectifyMap(
      pre_K, geometry_param_.D, mapR, geometry_param_.K, img_size, CV_32FC1,
      geometry_param_.mapX, geometry_param_.mapY);
  geometry_param_.D = 0.0;
}

void FisheyeFrameCameraCapture::undistort(const cv::Mat &dist_img, cv::Mat &undist_img) {
  cv::remap(dist_img, undist_img, geometry_param_.mapX, geometry_param_.mapY,
      cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

}; // namespace Modules::PoseEstimation
