#ifndef POSE_ESTIMATION_MODULE__FRAME_CAPTURE_BASE_HPP__
#define POSE_ESTIMATION_MODULE__FRAME_CAPTURE_BASE_HPP__

#include <atomic>
#include <chrono>
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

#include "configuration/types.hpp"

namespace Modules::PoseEstimation {

struct CameraGeometryParam {
  cv::Mat mapX;
  cv::Mat mapY;
  cv::Mat K;
  cv::Mat D;
};

class FrameCaptureBase {
public:
  FrameCaptureBase(const char *devname, int img_w, int img_h, int fourcc_code, const char *param_filename, int fps = 30, int buf_size = 2);

  virtual ~FrameCaptureBase() {}

  virtual bool is_opened() const = 0;

  virtual bool is_readable() const = 0;

  virtual bool read(cv::Mat &img, std::chrono::steady_clock::time_point &tp) = 0;

  virtual bool read_until(cv::Mat &img, std::chrono::steady_clock::time_point &tp, int timeout_ms = 500) = 0;

  virtual int width() const = 0;

  virtual int height() const = 0;

  virtual const cv::Mat &get_camera_matrix() const = 0;

  virtual const cv::Mat &get_distort_param() const = 0;

protected:
  virtual void undistort(const cv::Mat &dist_img, cv::Mat &undist_img) = 0;

  CameraGeometryParam geometry_param_;
  const int img_w_;
  const int img_h_;
};

}; // namespace Modules::PoseEstimation

#endif
