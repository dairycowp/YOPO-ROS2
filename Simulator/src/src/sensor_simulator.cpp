#include "sensor_simulator.h"
#include <pcl_conversions/pcl_conversions.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

void SensorSimulator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion quat_tf;
    tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
    quat = Eigen::Quaternionf(quat_tf.w(), quat_tf.x(), quat_tf.y(), quat_tf.z());

    pos.x() = msg->pose.pose.position.x;
    pos.y() = msg->pose.pose.position.y;
    pos.z() = msg->pose.pose.position.z;

    odom_init = true;
}

cv::Mat SensorSimulator::renderDepthImage() {
    cv::Mat depth_image = cv::Mat::zeros(image_height, image_width, CV_32FC1);
    Eigen::Matrix3f R = quat.toRotationMatrix();
    Eigen::Vector3f T = pos;

    float fx = this->fx;
    float fy = this->fy;
    float cx = this->cx;
    float cy = this->cy;

#pragma omp parallel for
    for (int u = 0; u < image_width; ++u) {
        for (int v = 0; v < image_height; ++v) {
            // Normalized coordinates
            float x = (u - cx) / fx;
            float y = (v - cy) / fy;
            float z = 1.0;

            Eigen::Vector3f ray_direction(x, y, z);
            ray_direction.normalize();

            // Rotate ray direction
            Eigen::Vector3f rotated_ray = R * ray_direction;

            // Raycast
            float depth = max_depth_dist;
            Eigen::Vector3f current_pos = T;
            float step_size = 0.1; // Step size for ray marching

            for (float t = 0.0; t < max_depth_dist; t += step_size) {
                current_pos = T + t * rotated_ray;

                // Search for points in octree
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                // Convert Eigen::Vector3f to pcl::PointXYZ
                pcl::PointXYZ searchPoint;
                searchPoint.x = current_pos.x();
                searchPoint.y = current_pos.y();
                searchPoint.z = current_pos.z();

                if (octree->radiusSearch(searchPoint, 0.1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    depth = t;
                    break;
                }
            }

            if (normalize_depth)
                depth_image.at<float>(v, u) = depth / max_depth_dist;
            else
                depth_image.at<float>(v, u) = depth;
        }
    }

    return depth_image;
}

pcl::PointCloud<pcl::PointXYZ> SensorSimulator::renderLidarPointcloud() {
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    Eigen::Matrix3f R = quat.toRotationMatrix();
    Eigen::Vector3f T = pos;

    // Vertical angles
    std::vector<float> vertical_angles;
    float vertical_step = (vertical_angle_end - vertical_angle_start) / (vertical_lines - 1);
    for (int i = 0; i < vertical_lines; ++i) {
        vertical_angles.push_back((vertical_angle_start + i * vertical_step) * M_PI / 180.0);
    }

    // Horizontal angles
    std::vector<float> horizontal_angles;
    for (int i = 0; i < horizontal_num; ++i) {
        horizontal_angles.push_back(i * horizontal_resolution * M_PI / 180.0);
    }

    for (int v = 0; v < vertical_lines; ++v) {
        float theta = vertical_angles[v];

        for (int h = 0; h < horizontal_num; ++h) {
            float phi = horizontal_angles[h];

            // Spherical to Cartesian coordinates
            Eigen::Vector3f ray_direction;
            ray_direction.x() = cos(theta) * cos(phi);
            ray_direction.y() = cos(theta) * sin(phi);
            ray_direction.z() = sin(theta);

            // Rotate ray direction
            Eigen::Vector3f rotated_ray = R * ray_direction;

            // Raycast
            float depth = max_lidar_dist;
            Eigen::Vector3f current_pos = T;
            float step_size = 0.1; // Step size for ray marching

            for (float t = 0.0; t < max_lidar_dist; t += step_size) {
                current_pos = T + t * rotated_ray;

                // Search for points in octree
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                // Convert Eigen::Vector3f to pcl::PointXYZ
                pcl::PointXYZ searchPoint;
                searchPoint.x = current_pos.x();
                searchPoint.y = current_pos.y();
                searchPoint.z = current_pos.z();

                if (octree->radiusSearch(searchPoint, 0.1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    depth = t;
                    break;
                }
            }

            if (depth < max_lidar_dist) {
                Eigen::Vector3f point_pos = T + depth * rotated_ray;
                pointcloud.push_back(pcl::PointXYZ(point_pos.x(), point_pos.y(), point_pos.z()));
            }
        }
    }

    return pointcloud;
}

void SensorSimulator::timerDepthCallback() {
    if (!odom_init || !render_depth)
        return;

    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat depth_image = renderDepthImage();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    // std::cout << "生成图像耗时: " << elapsed.count() << " 秒" << std::endl;

    sensor_msgs::msg::Image ros_image;
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = nh_->now();
    cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_image.image = depth_image;
    cv_image.toImageMsg(ros_image);
    image_pub_->publish(ros_image);
}

void SensorSimulator::timerLidarCallback() {
    if (!odom_init || !render_lidar)
        return;

    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ> lidar_points = renderLidarPointcloud();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    // std::cout << "生成雷达耗时: " << elapsed.count() << " 秒" << std::endl;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(lidar_points, output);
    output.header.stamp = nh_->now();
    output.header.frame_id = "odom";
    point_cloud_pub_->publish(output);
}

void SensorSimulator::expand_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr expanded_cloud, int direction) {
    // Find min and max values
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*expanded_cloud, min_pt, max_pt);

    float expand_distance;
    if (direction == 0) // x direction
        expand_distance = max_pt.x - min_pt.x;
    else // y direction
        expand_distance = max_pt.y - min_pt.y;

    int original_size = expanded_cloud->points.size();
    for (int i = 0; i < original_size; ++i) {
        pcl::PointXYZ new_point = expanded_cloud->points[i];
        if (direction == 0)
            new_point.x += expand_distance;
        else
            new_point.y += expand_distance;
        expanded_cloud->points.push_back(new_point);
    }
}