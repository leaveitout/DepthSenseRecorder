#include <memory>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "DS325.hpp"

using namespace rgbd;

int main(int argc, char *argv[]) {

    boost::shared_ptr<DepthCamera> camera(new DS325(0, FRAME_FORMAT_WXGA_H));
    camera->start();

    boost::shared_ptr<pcl::visualization::CloudViewer> viewer(new pcl::visualization::CloudViewer("Vertex"));

//    ColoredPointCloud::Ptr colorCloud(new ColoredPointCloud(
//            camera->depthSize().width, camera->depthSize().height));

    PointCloud::Ptr cloud(new PointCloud(
            camera->depthSize().width, camera->depthSize().height));

    while(!viewer->wasStopped(30)) {


//        camera->captureDepth(depth);
//        camera->captureAmplitude(amplitude);
//        camera->captureColor(color);
//        camera->captureColoredPointCloud(cloud);
        camera->capturePointCloud(cloud);

//        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//        // Create the segmentation object
//        pcl::SACSegmentation<pcl::PointXYZ> seg;
//        // Optional
//        seg.setOptimizeCoefficients (true);
//        // Mandatory
//        seg.setModelType (pcl::SACMODEL_PLANE);
//        seg.setMethodType (pcl::SAC_RANSAC);
//        seg.setDistanceThreshold (0.01);
//
//        seg.setInputCloud (cloud);
//        seg.segment (*inliers, *coefficients);
//
//        if (inliers->indices.size () == 0)
//        {
//            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//            //return (-1);
//        }

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