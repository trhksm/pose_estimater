#include "../include/frame_capture_base.hpp"

namespace Modules::PoseEstimation {

FrameCaptureBase::FrameCaptureBase(const char *devname, int img_w, int img_h, int fourcc_code, const char *param_filename, int fps, int buf_size)
    : img_w_(img_w), img_h_(img_h) {
  // Base constructor - implementation details handled by derived classes
}

}; // namespace Modules::PoseEstimation
