#include <iomanip>
#include <iterator>
#include <chrono>
#include <thread>

#include "color_mapping.h"
#include "yaml-cpp/yaml.h"

ColorMapping::ColorMapping() :
    ground_truth_path_("/home/lyh/dataset/groundtruth_kitti/dataset/poses/kitti00/002.txt"),
    lidar_path_("/home/lyh/dataset/kitti_00/velodyne_points/data/"),
    image_path_("/home/lyh/dataset/kitti_00/image_02_un/"),
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

ColorMapping::ColorMapping(const std::string& config_file) : map_points_(new pcl::PointCloud<pcl::PointXYZRGB>()) {
    std::cout << config_file << std::endl;
    auto yaml = YAML::LoadFile(config_file);
    ground_truth_path_ = yaml["ground_truth_path"].as<std::string>();
    lidar_path_ = yaml["lidar_path"].as<std::string>();
    image_path_ = yaml["image_path"].as<std::string>();
    result_path_ = yaml["result_path"].as<std::string>();
    std::vector<float> R_velo_cam_vec = yaml["R_velo_cam"].as<std::vector<float>>();
    std::vector<float> t_velo_cam_vec = yaml["t_velo_cam"].as<std::vector<float>>();
    std::vector<float> R_rect_00_vec = yaml["R_rect_00"].as<std::vector<float>>();
    std::vector<float> P2_vec = yaml["P2"].as<std::vector<float>>();
    line_begin_ = yaml["line_begin"].as<int>();
    Eigen::Matrix3f R_velo_cam;
    Eigen::Vector3f t_velo_cam;
    R_velo_cam << R_velo_cam_vec[0], R_velo_cam_vec[1], R_velo_cam_vec[2],
               R_velo_cam_vec[3], R_velo_cam_vec[4], R_velo_cam_vec[5],
               R_velo_cam_vec[6], R_velo_cam_vec[7], R_velo_cam_vec[8];
    t_velo_cam << t_velo_cam_vec[0], t_velo_cam_vec[1], t_velo_cam_vec[2];
    T_ve_cam_.block<3, 3>(0, 0) = R_velo_cam;
    T_ve_cam_.block<3, 1>(0, 3) = t_velo_cam;
    T_ve_cam_(3, 0) = 0.0f;
    T_ve_cam_(3, 1) = 0.0f;
    T_ve_cam_(3, 2) = 0.0f;
    T_ve_cam_(3, 3) = 1.0f;
    P2 << P2_vec[0], P2_vec[1], P2_vec[2], P2_vec[3],
          P2_vec[4], P2_vec[5], P2_vec[6], P2_vec[7],
          P2_vec[8], P2_vec[9], P2_vec[10], P2_vec[11];
    R0_rect << R_rect_00_vec[0], R_rect_00_vec[1], R_rect_00_vec[2], 0,
               R_rect_00_vec[3], R_rect_00_vec[4], R_rect_00_vec[5], 0,
               R_rect_00_vec[6], R_rect_00_vec[7], R_rect_00_vec[8], 0,
               0, 0, 0, 1;
    std::cout << R0_rect << std::endl;
    trans = P2 * R0_rect * T_ve_cam_;
}


ColorMapping::~ColorMapping() {}

void ColorMapping::MapBuild() {
    std::ifstream ground_truth_file(ground_truth_path_, std::ifstream::in);
    std::size_t line_num = line_begin_;
    std::string line;

    while (std::getline(ground_truth_file, line)) {
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

            Eigen::Vector4f point_camera = T_ve_cam_ * point_velodyne;
            Eigen::Vector3f point_word = q_w_i * point_camera.block<3, 1>(0, 0) + t_w_i;
            Eigen::Vector3f p_result = trans * point_velodyne;
            int p_u = std::floor(p_result.x() / p_result.z());
            int p_v = std::floor(p_result.y() / p_result.z());
            if ((p_u < 20) || (p_u > cols - 20) || (p_v < 30) || (p_v > rows - 30) || p_result.z() < 0 || p_result.z() > 30.0) {
                continue;
            }
            pcl::PointXYZRGB point_rgb;
            point_rgb.x = point_word.x();
            point_rgb.y = point_word.y();
            point_rgb.z = point_word.z();
            point_rgb.r = image.at<cv::Vec3b>(p_v, p_u)[2];
            point_rgb.g = image.at<cv::Vec3b>(p_v, p_u)[1];
            point_rgb.b = image.at<cv::Vec3b>(p_v, p_u)[0];
            map_points_->push_back(point_rgb);
        }
        if (line_num % 50 == 0) {
            writer.write(result_path_ + std::to_string(line_num) + ".pcd", *map_points_);
            map_points_->clear();
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
        line_num++;
        // if (line_num == 400) break;
    }
    // filter_.setInputCloud(map_points_);
    // filter_.filter(*map_points_);
    // writer.write(result_path_, *map_points_);
    std::cout << "line_num: " << line_num << std::endl;
}


void ColorMapping::TransformPointCloud(const Eigen::Quaterniond& q_w_i, const Eigen::Vector3d& t_w_i) {

}
std::vector<float> ColorMapping::Read_lidar_data(const std::string lidar_data_path) {
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}
