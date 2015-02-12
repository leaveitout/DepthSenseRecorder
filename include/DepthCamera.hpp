#ifndef DEPTH_CAMERA_HPP
#define DEPTH_CAMERA_HPP


#include <vector>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/StdVector>
#include "Error.hpp"
#include "ColorCamera.hpp"

namespace rgbd {

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    typedef pcl::PointCloud<pcl::PointXYZRGB> ColoredPointCloud;

    class DepthCamera: public ColorCamera {
    public:
        DepthCamera();

        DepthCamera(const std::shared_ptr<ColorCamera> camera);

        virtual ~DepthCamera();

        virtual cv::Size colorSize() const;

        virtual void start();

        virtual void captureColor(cv::Mat& buffer);

        /**
        * Return the size of depth image.
        *
        * @return Size of depth image
        */
        virtual cv::Size depthSize() const;

        /**
        * Copy the latest depth data to the buffer.
        * Note that the buffer must be allocated in advance.
        *
        * @param buffer Returned cv::Mat of CV_32F
        */
        virtual void captureDepth(cv::Mat& buffer);

        /**
        * Copy the latest amplitude data to the buffer.
        * Note that the buffer must be allocated in advance.
        *
        * @param buffer Returned cv::Mat of CV_32F
        */
        virtual void captureAmplitude(cv::Mat& buffer);

        /**
        * Copy the latest 3D point cloud data to the buffer.
        * Note that the buffer must be allocated in advance.
        *
        * @param buffer Returned pcl::PointCloud<pcl::PointXYZ>::Ptr
        */
        virtual void capturePointCloud(PointCloud::Ptr buffer);

        /**
        * Copy the latest colored 3D point cloud data to the buffer.
        * Note that the buffer must be allocated in advance.
        *
        * @param buffer Returned pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        */
        virtual void captureColoredPointCloud(ColoredPointCloud::Ptr buffer);

    private:
        std::shared_ptr<ColorCamera> _camera;
    };

}


#endif