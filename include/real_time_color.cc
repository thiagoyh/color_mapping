#include <iomanip>
#include <iterator>
#include <chrono>
#include <thread>
#include "real_time_color.h"

RealTimeColor::RealTimeColor() :
    lidar_path_("/home/lyh/dataset/kitti_color/2011_09_26_drive_0028_sync/velodyne_points/data/"),
    image_path_("/home/lyh/dataset/kitti_color/2011_09_26_drive_0028_sync/image_02/data/"),
    result_path_("/home/lyh/experiment/kitti/kitti_color/real_time/"),
    map_points_(new pcl::PointCloud<pcl::PointXYZRGB>()) {
    T_ve_cam_ << 7.533745000000e-03,-9.999714000000e-01,-6.166020000000e-04,-4.069766000000e-03,
    1.480249000000e-02,7.280733000000e-04,-9.998902000000e-01,-7.631618000000e-02,
    9.998621000000e-01,7.523790000000e-03,1.480755000000e-02,-2.717806000000e-01,
     0,0,0,1;

    P2 << 7.215377000000e+02,0.000000000000e+00,6.095593000000e+02,4.485728000000e+01,
    0.000000000000e+00,7.215377000000e+02,1.728540000000e+02,2.163791000000e-01,
    0.000000000000e+00,0.000000000000e+00,1.000000000000e+00,2.745884000000e-03;
    R0_rect << 9.999239000000e-01,9.837760000000e-03,-7.445048000000e-03,0,
    -9.869795000000e-03,9.999421000000e-01,-4.278459000000e-03,0,
    7.402527000000e-03,4.351614000000e-03,9.999631000000e-01,
    0, 0,0,0,1;
    //   T_ve_cam_ = T_ve_cam_.inverse();
    trans = P2 * R0_rect * T_ve_cam_;
    filter_.setLeafSize(0.2f, 0.2f, 0.2f);
}

RealTimeColor::~RealTimeColor() {}

void RealTimeColor::MapBuild() {
    std::size_t line_num = 0;
    std::string line;

    while (line_num < 400) {
        map_points_->clear();
        // std::getline(ground_truth_file, line);
        std::stringstream lidar_data_path;
        lidar_data_path << lidar_path_
                        << std::setfill('0') << std::setw(10) << line_num << ".bin";
        std::stringstream image_data_path;
        image_data_path << image_path_
                        << std::setfill('0') << std::setw(10) << line_num << ".png";
        cv::Mat image = cv::imread(image_data_path.str());
        if (image.channels() != 3) {
            std::cout << "need rgb picture!\n";
        }
        int rows = image.rows;
        int cols = image.cols;
        std::cout << "lidar_data_path: " << lidar_data_path.str() << std::endl;
        std::vector<float> lidar_data = Read_lidar_data(lidar_data_path.str());
        std::cout << "after read\n";
        for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
            Eigen::Vector4f point_velodyne;
            point_velodyne[0] = lidar_data[i];
            point_velodyne[1] = lidar_data[i + 1];
            point_velodyne[2] = lidar_data[i + 2];
            point_velodyne[3] = 1.0f;

            Eigen::Vector3f p_result = trans * point_velodyne;
            int p_u = std::floor(p_result.x() / p_result.z());
            int p_v = std::floor(p_result.y() / p_result.z());
            if ((p_u < 0) || (p_u > cols) || (p_v < 0) || (p_v > rows) || p_result.z() < 0 || p_result.z() > 30.0) {
                continue;
            }
            pcl::PointXYZRGB point_rgb;
            point_rgb.x = point_velodyne.x();
            point_rgb.y = point_velodyne.y();
            point_rgb.z = point_velodyne.z();
            point_rgb.r = image.at<cv::Vec3b>(p_v, p_u)[2];
            point_rgb.g = image.at<cv::Vec3b>(p_v, p_u)[1];
            point_rgb.b = image.at<cv::Vec3b>(p_v, p_u)[0];
            map_points_->push_back(point_rgb);
            // map_points_->push_back(point);
        }


        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
        line_num++;
        // if (line_num == 200) break;
        std::cout << "line_num: " << line_num << std::endl;
        writer.write(result_path_ + std::to_string(line_num) + ".pcd", *map_points_);
    }
    // filter_.setInputCloud(map_points_);
    // filter_.filter(*map_points_);

}


void RealTimeColor::TransformPointCloud(const Eigen::Quaterniond& q_w_i, const Eigen::Vector3d& t_w_i) {

}
std::vector<float> RealTimeColor::Read_lidar_data(const std::string lidar_data_path) {
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}
