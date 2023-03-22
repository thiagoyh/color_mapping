#pragma once

#include <vector>
#include <string>
#include <iostream>

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace {
    std::vector<double> R_02_01 = {9.999788e-01, -5.008404e-03, -4.151018e-03,
                                    4.990516e-03, 9.999783e-01, -4.308488e-03,
                                     4.172506e-03, 4.287682e-03, 9.999821e-01};
    std::vector<double> t_02_01 = {5.954406e-02, -7.675338e-04, 3.582565e-03};

}

class Color {
public:
    Color();
    ~Color();
    void coloring();
private:
    void loadPCD();
    void loadImage(cv::Mat& image);
    void writeOnce();
    void getPoseAndImage();
    void project2Pixel(cv::Mat& image, Eigen::Matrix<float, 3, 4>& gt_pose);
    Eigen::Vector2f brownconrady(const Eigen::Vector3f& xcam);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_points_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_color_;
    std::vector<float> colored_flag_;
    Eigen::Matrix4f cam_to_cam_;
    Eigen::Matrix<float, 3, 4> P_rect_02;
    Eigen::Matrix3f K;
    Eigen::Matrix4f R_rect_02;
    float k1, k2, p1, p2, p3, p4 = 0.0;

    // path:
    std::string ground_truth_path_ = "/home/lyh/dataset/groundtruth_kitti/dataset/poses/001.txt";
    std::string map_path_ = "/home/lyh/experiment/kitti/kitti_001.pcd";
    std::string image_path_ = "/home/lyh/dataset/kitti_00/image_02_un/";
    std::string out_path_ = "/home/lyh/experiment/kitti/kitti_001color.pcd";
    int width_;
    int height_;
};
