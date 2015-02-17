#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "DS325.hpp"

using namespace rgbd;

int main(int argc, char* argv[]) {

    boost::shared_ptr<DepthCamera> camera(new DS325(0, FRAME_FORMAT_WXGA_H));
    camera->start();

    cv::Mat depth = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat amplitude = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);

    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
    cv::namedWindow("Amplitude", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
    cv::namedWindow("Color", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);

    ColoredPointCloud::Ptr cloud(new ColoredPointCloud(
            camera->depthSize().width, camera->depthSize().height));

    while(cv::waitKey(30) != 0x1b) {

        camera->captureDepth(depth);
        camera->captureAmplitude(amplitude);
        camera->captureColor(color);
        camera->captureColoredPointCloud(cloud);

        cv::Mat d, a;
        depth.convertTo(d, CV_8U, 255.0 / 1000.0);
        amplitude.convertTo(a, CV_8U, 255.0 / 1000.0);

        cv::imshow("Depth", d);
        cv::imshow("Amplitude", a);
        cv::imshow("Color", color);
    }
}