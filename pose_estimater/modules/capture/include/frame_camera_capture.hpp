#ifndef POSE_ESTIMATION_MODULE__FRAME_CAMERA_CAPTURE_HPP__
#define POSE_ESTIMATION_MODULE__FRAME_CAMERA_CAPTURE_HPP__

#include "frame_capture_base.hpp"
#include <atomic>
#include <condition_variable>
#include <mutex>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <thread>

namespace Modules::PoseEstimation {

class FrameCameraCapture : public FrameCaptureBase {
public:
  FrameCameraCapture(const char *devname, int img_w, int img_h, int fourcc_code,
      const char *param_filename, int fps = 30, int buf_size = 2);

  virtual ~FrameCameraCapture();

  bool is_opened() const override;

  void process();

  bool is_readable() const override;

  bool read(cv::Mat &img, std::chrono::steady_clock::time_point &tp) override;

  bool read_until(cv::Mat &img, std::chrono::steady_clock::time_point &tp,
      int timeout_ms = 500) override;

  int width() const override;

  int height() const override;

  const cv::Mat &get_camera_matrix() const override { return geometry_param_.K; }

  const cv::Mat &get_distort_param() const override { return geometry_param_.D; }

protected:
  void undistort(const cv::Mat &dist_img, cv::Mat &undist_img) override;

private:
  cv::VideoCapture cap_;
  std::atomic_bool have_read_;
  std::chrono::steady_clock::time_point capture_time_point_;
  cv::Mat cap_buf_;
  std::thread capture_th_;
  std::mutex cap_buf_mutex_;
  std::condition_variable capture_condvar_;
  std::atomic_bool end_flag_;
};

class FisheyeFrameCameraCapture : public FrameCameraCapture {
public:
  FisheyeFrameCameraCapture(const char *devname, int img_w, int img_h,
      int fourcc_code, const char *param_filename,
      double cropping_rate = 0.5, int fps = 30,
      int buf_size = 2);

private:
  void undistort(const cv::Mat &dist_img, cv::Mat &undist_img) override;
};

}; // namespace Modules::PoseEstimation

#endif
