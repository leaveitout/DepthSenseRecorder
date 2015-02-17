#include <pcl/visualization/cloud_viewer.h>

#include "DS325.hpp"

using namespace rgbd;

int main(int argc, char *argv[]) {

    boost::shared_ptr<DepthCamera> camera(new DS325(0, FRAME_FORMAT_WXGA_H));
    camera->start();

    cv::Mat depth = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat amplitude = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);
    boost::shared_ptr<pcl::visualization::CloudViewer> viewer(new pcl::visualization::CloudViewer("Vertex"));

    ColoredPointCloud::Ptr cloud(new ColoredPointCloud(
            camera->depthSize().width, camera->depthSize().height));

    while(!viewer->wasStopped(30)) {

        camera->captureDepth(depth);
        camera->captureAmplitude(amplitude);
        camera->captureColor(color);
        camera->captureColoredPointCloud(cloud);

        viewer->showCloud(cloud);
    }

    return 0;
}
