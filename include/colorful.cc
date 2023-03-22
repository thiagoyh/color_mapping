#include "colorful.h"

Color::Color() : cloud_with_color_(new pcl::PointCloud<pcl::PointXYZRGB>()),
                 map_points_(new pcl::PointCloud<pcl::PointXYZI>()) {
    P_rect_02 << 7.188560e+02, 0.000000e+00, 6.071928e+02, 4.538225e+01,
                0.000000e+00, 7.188560e+02, 1.852157e+02, -1.130887e-01,
                0.000000e+00, 0.000000e+00, 1.000000e+00, 3.779761e-03;
    K << 9.601149e+02, 0.000000e+00, 6.947923e+02,
         0.000000e+00, 9.548911e+02, 2.403547e+02,
         0.000000e+00, 0.000000e+00, 1.000000e+00;
    R_rect_02 << 9.999191e-01, 1.228161e-02, -3.316013e-03, 0.0
             -1.228209e-02, 9.999246e-01, -1.245511e-04, 0.0,
              3.314233e-03, 1.652686e-04, 9.999945e-01, 0.0,
              0.0, 0.0, 0.0, 1.0;
    k1 = -3.685917e-01;
    k2 = 1.928022e-01;
    p1 = 4.069233e-04;
    p2 = 7.247536e-04;
    p3 = -6.276909e-02;
}

Color::~Color() {}

void Color::loadPCD() {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path_, *map_points_) == -1) {
        PCL_ERROR("couldn't read file!\n");
        return;
    }
}

void Color::writeOnce() {
    pcl::io::savePCDFileASCII(out_path_, *cloud_with_color_);
}

void Color::coloring() {
    std::ifstream ground_truth_file(ground_truth_path_, std::ifstream::in);
    std::size_t line_num = 0;
    std::string line;
    loadPCD();
    std::cout << "map size: " << map_points_->size() << std::endl;
    colored_flag_.resize(map_points_->size(), 50.0f);
    cloud_with_color_->reserve(map_points_->size());
    for (auto it = map_points_->begin(); it != map_points_->end(); ++it) {
        pcl::PointXYZRGB point_with_color;
        point_with_color.x = it->x;
        point_with_color.y = it->y;
        point_with_color.z = it->z;
        cloud_with_color_->push_back(point_with_color);
    }

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
        std::stringstream image_data_path;
        image_data_path << image_path_
                        << std::setfill('0') << std::setw(10) << line_num << ".png";
        std::cout << "image_data_path: " << image_data_path.str() << std::endl;
        cv::Mat image02 = cv::imread(image_data_path.str(), -1);
        width_ = image02.cols;
        height_ = image02.rows;
        project2Pixel(image02, gt_pose);
        line_num++;
    }
    writeOnce();
}

void Color::project2Pixel(cv::Mat& image, Eigen::Matrix<float, 3, 4>& gt_pose) {
    Eigen::Quaternionf q_w_i(gt_pose.topLeftCorner<3, 3>());
    q_w_i.normalize();
    Eigen::Vector3f t_w_i = gt_pose.topRightCorner<3, 1>();
    Eigen::Quaternionf q_i_w = q_w_i.conjugate();
    // Eigen::Vector3f t_i_w = -(q_i_w * t_w_i);
    for (int i = 0; i < map_points_->size(); ++i) {
        Eigen::Vector3f point_temp(map_points_->points[i].x,
                                   map_points_->points[i].y,
                                   map_points_->points[i].z);
        // if ((point_temp - t_w_i).norm() > 50) {
        //     continue;
        // }
        Eigen::Vector3f point_cam0 = q_i_w * (point_temp - t_w_i);
        point_cam0.z() += 0.05954406f;
        if (point_cam0.z()  < 0) {
            continue;
        }

        Eigen::Vector4f point_4f;
        point_4f.block<3, 1>(0, 0) = point_cam0;
        point_4f(4) = 1.0f;
        Eigen::Vector3f pixel_u_v = P_rect_02 * R_rect_02 * point_4f;

        // Eigen::Vector2f pixel_u_v = brownconrady(point_cam0);
        if (pixel_u_v.x() < 0 || pixel_u_v.x() >= width_ ||
            pixel_u_v.y() < 0 || pixel_u_v.y() >= height_) {
            continue;
        }

        // std::cout << "cjdsihfsfu\n";
        int u = std::floor(pixel_u_v.x());
        int v = std::floor(pixel_u_v.y());
        int r = image.at<cv::Vec3b>(v, u)[0];
        int g = image.at<cv::Vec3b>(v, u)[1];
        int b = image.at<cv::Vec3b>(v, u)[2];

        cloud_with_color_->points[i].x = point_temp.x();
        cloud_with_color_->points[i].y = point_temp.y();
        cloud_with_color_->points[i].z = point_temp.z();
        cloud_with_color_->points[i].r = r;
        cloud_with_color_->points[i].g = g;
        cloud_with_color_->points[i].b = b;

        colored_flag_[i] = point_cam0.z();
    }
}

Eigen::Vector2f Color::brownconrady(const Eigen::Vector3f& xcam) {
    if (xcam(2) < 0) {
        return Eigen::Vector2f(-1, -1);
    }
    Eigen::Vector2f Xp1(xcam(0) / xcam(2), xcam(1) / xcam(2));
    float r = Xp1.norm();

    float xdist = Xp1(0) + Xp1(0) * (k1 * std::pow(r, 2) + k2 * std::pow(r, 4)) +
                 (p1 * (std::pow(r, 2) + 2 * std::pow(Xp1(0), 2)) +
                  2 * p2 * Xp1(0) * Xp1(1)) * (1 + p3 * std::pow(r, 2) + p4 * std::pow(r, 4));
    float ydist = Xp1(1) + Xp1(1) * (k1 * std::pow(r, 2) + k2 * std::pow(r, 4)) +
                 (2 * p1 * Xp1(0) * Xp1(1) + p2 * (std::pow(r, 2) + 2 * std::pow(Xp1(1), 2))) *
                 (1 + p3 * std::pow(r, 2) + p4 * std::pow(r, 4));
    Eigen::Vector2f Xp1d(xdist, ydist);

       Eigen::Vector3f Xp1dh(Xp1d(0), Xp1d(1), 1);
   Eigen::Vector3f Xpix = K * Xp1dh;
   return Eigen::Vector2f(Xpix(0) / Xpix(2), Xpix(1) / Xpix(2));

    // Eigen::Vector2f Xp1(xcam(0) / xcam(2), xcam(1) / xcam(2));
    // Eigen::Vector3f Xp1dh(Xp1(0), Xp1(1), 1.0);
    // Eigen::Vector3f Xpix = K * Xp1dh;
    // return Eigen::Vector2f(Xpix(0) / Xpix(2), Xpix(1) / Xpix(2));
}
