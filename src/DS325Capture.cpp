#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "DS325.hpp"

using namespace rgbd;

//DEFINE_int32(id, 0, "camera id");

int main(int argc, char *argv[]) {
//    google::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<DepthCamera> camera(new DS325(0, FRAME_FORMAT_WXGA_H));
    camera->start();
//
    cv::Mat depth = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat amplitude = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);
    std::shared_ptr<pcl::visualization::CloudViewer> viewer(new pcl::visualization::CloudViewer("Vertex"));

//    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer",true));

    //std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer(new pcl::visualization::PCLVisualizer("Visualizer",true));
    //pcl::visualization::CloudViewer* viewer = new pcl::visualization::CloudViewer()
    ColoredPointCloud::Ptr cloud(new ColoredPointCloud(
            camera->depthSize().width, camera->depthSize().height));


//
//    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
//    cv::namedWindow("Amplitude", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
//    cv::namedWindow("Color", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
//
    //while (cv::waitKey(30) != 0x1b) {
    while(!viewer->wasStopped(30)) {


        camera->captureDepth(depth);
        camera->captureAmplitude(amplitude);
        camera->captureColor(color);
        camera->captureColoredPointCloud(cloud);

//        cv::Mat d, a;
//        depth.convertTo(d, CV_8U, 255.0 / 1000.0);
//        amplitude.convertTo(a, CV_8U, 255.0 / 1000.0);

//        cv::imshow("Depth", d);
//        cv::imshow("Amplitude", a);
//        cv::imshow("Color", color);
        //visualizer->addCl
        viewer->showCloud(cloud);
    }

    return 0;
}
