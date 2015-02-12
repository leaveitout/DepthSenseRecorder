#ifndef COLOR_CAMERA_HPP
#define COLOR_CAMERA_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Error.hpp"

namespace rgbd {

class ColorCamera {
public:
    ColorCamera();

    virtual ~ColorCamera();

    /**
    * Return the size of color image.
    *
    * @return Size of color image
    */
    virtual cv::Size colorSize() const;

    /**
    * Start the updating of the device.
    */
    virtual void start();

    /**
    * Copy the latest color data to the buffer.
    * Note that the buffer must be allocated in advance.
    *
    * @param buffer Returned matrix of CV_8UC3
    */
    virtual void captureColor(cv::Mat& buffer);

};

}
#endif