#include <iomanip>
#include <iterator>
#include <chrono>
#include <thread>
#include <pcl/io/pcd_io.h>
#include "mapping.h"

Mapping::Mapping() :
    ground_truth_path_("/home/lyh/dataset/groundtruth_kitti/dataset/poses/kitti00/002.txt"),
    lidar_path_("/home/lyh/dataset/kitti_00/velodyne_points/data/"),
    result_path_("/home/lyh/experiment/kitti/kitti_002full.pcd"),
    map_points_(new pcl::PointCloud<pcl::PointXYZI>()) {
    T_ve_cam_ << 7.967514e-03, -9.999679e-01, -8.462264e-04, -1.377769e-02,
     -2.771053e-03, 8.241710e-04, -9.999958e-01, -5.542117e-02,
      9.999644e-01, 7.969825e-03, -2.764397e-03, -2.918589e-01,
      0, 0, 0, 1;
    //   T_ve_cam_ = T_ve_cam_.inverse();
    filter_.setLeafSize(0.2f, 0.2f, 0.2f);
}

Mapping::~Mapping() {}

void Mapping::MapBuild() {
    std::ifstream ground_truth_file(ground_truth_path_, std::ifstream::in);
    std::size_t line_num = 1000;
    std::string line;

    while (std::getline(ground_truth_file, line)) {
        // std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<float, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }
        Eigen::Quaternionf q_w_i(gt_pose.topLeftCorner<3, 3>());
        q_w_i.normalize();
        Eigen::Vector3f t_w_i = gt_pose.topRightCorner<3, 1>();
        std::cout << "vec: " << t_w_i << std::endl;
        std::stringstream lidar_data_path;
        lidar_data_path << lidar_path_
                        << std::setfill('0') << std::setw(10) << line_num << ".bin";
        std::cout << "lidar_data_path: " << lidar_data_path.str() << std::endl;
        std::vector<float> lidar_data = Read_lidar_data(lidar_data_path.str());
        std::cout << "after read\n";
        for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
            Eigen::Vector4f point_velodyne;
            point_velodyne[0] = lidar_data[i];
            point_velodyne[1] = lidar_data[i + 1];
            point_velodyne[2] = lidar_data[i + 2];
            point_velodyne[3] = 1.0f;

            Eigen::Vector4f point_camera = T_ve_cam_ * point_velodyne;
            Eigen::Vector3f point_word = q_w_i * point_camera.block<3, 1>(0, 0) + t_w_i;
            pcl::PointXYZI point;
            point.x = point_word[0];
            point.y = point_word[1];
            point.z = point_word[2];
            point.intensity = lidar_data[i + 3];
            map_points_->push_back(point);
        }


        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
        line_num++;
        // if (line_num == 400) break;
    }
    filter_.setInputCloud(map_points_);
    filter_.filter(*map_points_);
    std::cout << "line_num: " << line_num << std::endl;
    pcl::PCDWriter writer;
    writer.write(result_path_, *map_points_);
}


void Mapping::TransformPointCloud(const Eigen::Quaterniond& q_w_i, const Eigen::Vector3d& t_w_i) {

}
std::vector<float> Mapping::Read_lidar_data(const std::string lidar_data_path) {
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}
