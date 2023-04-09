#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

class ColorMapping {
public:
    ColorMapping();
    ~ColorMapping();
    void MapBuild();

private:
    void TransformPointCloud(const Eigen::Quaterniond& q_w_i, const Eigen::Vector3d& t_w_i);
    std::vector<float> Read_lidar_data(const std::string lidar_data_path);
    std::string ground_truth_path_;
    std::string lidar_path_;
    std::string image_path_;
    std::string result_path_;
    Eigen::Matrix<float, 4, 4> T_ve_cam_;
    Eigen::Matrix<float, 3, 4> P2;
    Eigen::Matrix<float, 4, 4> R0_rect;
    Eigen::Matrix<float, 3, 4> trans;
    pcl::VoxelGrid<pcl::PointXYZRGB> filter_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_points_;
};
