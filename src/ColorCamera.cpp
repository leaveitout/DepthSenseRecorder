#include "ColorCamera.hpp"

namespace rgbd {

    ColorCamera::ColorCamera() {

    }

    void ColorCamera::captureColor(cv::Mat &buffer) {
        throw new UnsupportedException("ColorCamera.captureColor");
    }

    ColorCamera::~ColorCamera() {

    }

    cv::Size ColorCamera::colorSize() const {
        throw new UnsupportedException("ColorCamera.colorSize");
    }

    void ColorCamera::start() {

    }
}