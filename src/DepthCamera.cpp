#include "DepthCamera.hpp"

namespace rgbd {

    DepthCamera::DepthCamera() {
    }

    rgbd::DepthCamera::DepthCamera(const std::shared_ptr<ColorCamera> camera) :
            _camera(camera) {
    }

    DepthCamera::~DepthCamera() {
    }

    cv::Size DepthCamera::colorSize() const {
        if (_camera)
            return _camera->colorSize();
        else
            throw new UnsupportedException("colorSize");
    }

    void DepthCamera::start() {
        if (_camera)
            _camera->start();
    }

    void DepthCamera::captureColor(cv::Mat& buffer) {
        if (_camera)
            _camera->captureColor(buffer);
    }

    cv::Size DepthCamera::depthSize() const {
        throw new UnsupportedException("depthSize");
    }

    void DepthCamera::captureDepth(cv::Mat& buffer) {
        throw new UnsupportedException("captureDepth");
    }

    void DepthCamera::captureAmplitude(cv::Mat& buffer) {
        throw new UnsupportedException("captureAmplitude");
    }

    void DepthCamera::capturePointCloud(PointCloud::Ptr buffer) {
        throw new UnsupportedException("captureVertex");
    }

    void DepthCamera::captureColoredPointCloud(ColoredPointCloud::Ptr buffer) {
        throw new UnsupportedException("captureColoredVertex");
    }

}
