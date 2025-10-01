#include "../capture/include/frame_camera_capture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {
    using namespace Modules::PoseEstimation;

    const char* devname = "/dev/video0";           // カメラデバイス
    const char* param_file = "../param/camera_calib/baffalo_BSW300M.xml";  // カメラパラメータ XML
    int width = 1280;
    int height = 720;
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    int fps = 30;

    try {
        // 通常カメラ
        FrameCameraCapture cam(devname, width, height, fourcc, param_file, fps);

        // 魚眼カメラの場合
        // FisheyeFrameCameraCapture cam(devname, width, height, fourcc, param_file);

        cv::Mat frame;
        std::chrono::steady_clock::time_point tp;

        while (true) {
            // 最大 1000ms 待ってフレームを取得
            if (cam.read_until(frame, tp, 1000)) {
                cv::imshow("Camera Frame", frame);
            } else {
                std::cerr << "Failed to read frame!" << std::endl;
            }

            // ESC キーで終了
            int key = cv::waitKey(1);
            if (key == 27) break;
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}

