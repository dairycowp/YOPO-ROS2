#ifndef SENSOR_SIMULATOR_H
#define SENSOR_SIMULATOR_H
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/octree/octree_search.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// 移除pcl_ros的point_cloud包含，直接使用sensor_msgs的point_cloud2
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <omp.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SensorSimulator {
public:
    SensorSimulator(std::shared_ptr<rclcpp::Node> nh) : nh_(nh) {
        YAML::Node config = YAML::LoadFile(CONFIG_FILE_PATH);
        // 读取camera参数
        fx = config["camera"]["fx"].as<float>();
        fy = config["camera"]["fy"].as<float>();
        cx = config["camera"]["cx"].as<float>();
        cy = config["camera"]["cy"].as<float>();
        image_width = config["camera"]["image_width"].as<int>();
        image_height = config["camera"]["image_height"].as<int>();
        max_depth_dist = config["camera"]["max_depth_dist"].as<float>();
        normalize_depth = config["camera"]["normalize_depth"].as<bool>();

        // 读取lidar参数
        vertical_lines = config["lidar"]["vertical_lines"].as<int>();
        vertical_angle_start = config["lidar"]["vertical_angle_start"].as<float>();
        vertical_angle_end = config["lidar"]["vertical_angle_end"].as<float>();
        horizontal_num = config["lidar"]["horizontal_num"].as<int>();
        horizontal_resolution = config["lidar"]["horizontal_resolution"].as<float>();
        max_lidar_dist = config["lidar"]["max_lidar_dist"].as<float>();

        render_lidar = config["render_lidar"].as<bool>();
        render_depth = config["render_depth"].as<bool>();
        float depth_fps = config["depth_fps"].as<float>();
        float lidar_fps = config["lidar_fps"].as<float>();

        std::string ply_file = config["ply_file"].as<std::string>();
        std::string odom_topic = config["odom_topic"].as<std::string>();
        std::string depth_topic = config["depth_topic"].as<std::string>();
        std::string lidar_topic = config["lidar_topic"].as<std::string>();

        float resolution = config["resolution"].as<float>();
        int expand_y_times = config["expand_y_times"].as<int>();
        int expand_x_times = config["expand_x_times"].as<int>();

        pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        printf("1.Reading Point Cloud... \n");

        // if (pcl::io::loadPCDFile("/home/lu/用完删除/test/map.pcd", *cloud) == -1) {
        //     PCL_ERROR("Couldn't read PLY file \n");
        //     return;
        // }

        if (pcl::io::loadPLYFile(ply_file, *orig_cloud) == -1) {
            PCL_ERROR("Couldn't read PLY file \n");
            return;
        }
        
        printf("2.Processing... \n");

        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        *cloud = *orig_cloud;
        for (int i = 0; i < expand_x_times; ++i) {
            expand_cloud(cloud, 0);
        }
        for (int i = 0; i < expand_y_times; ++i) {
            expand_cloud(cloud, 1);
        }

        octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr(
            new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution));
        octree->setInputCloud(cloud);
        octree->addPointsFromInputCloud();

        image_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(depth_topic, 1);
        point_cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 1);
        odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 1, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odomCallback(msg);
            });
        timer_depth_ = nh_->create_wall_timer(
            std::chrono::duration<float>(1.0f / depth_fps),
            [this]() {
                this->timerDepthCallback();
            });
        timer_lidar_ = nh_->create_wall_timer(
            std::chrono::duration<float>(1.0f / lidar_fps),
            [this]() {
                this->timerLidarCallback();
            });
        printf("3.Simulation Ready! \n");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    cv::Mat renderDepthImage();

    pcl::PointCloud<pcl::PointXYZ> renderLidarPointcloud();

    void timerDepthCallback();

    void timerLidarCallback();

    void expand_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr expanded_cloud, int direction = 0);

private:
    bool render_depth{false};
    bool render_lidar{false};
    bool odom_init{false};
    Eigen::Quaternionf quat;
    Eigen::Vector3f pos;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree;

    // camera param
    float fx = 80.0f; // focal length x
    float fy = 80.0f; // focal length y
    float cx = 80.0f; // principal point x (image center)
    float cy = 45.0f; // principal point y (image center)
    int image_width = 160;
    int image_height = 90;
    float max_depth_dist{20};
    bool normalize_depth{false};

    // lidar param
    int vertical_lines = 16;            // 纵向16线
    float vertical_angle_start = -15.0; // 起始垂直角度
    float vertical_angle_end = 15.0;    // 结束垂直角度
    int horizontal_num = 360;           // 水平360点
    float horizontal_resolution = 1.0;  // 水平分辨率为1度
    float max_lidar_dist{50};


    std::shared_ptr<rclcpp::Node> nh_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_depth_, timer_lidar_;
};


#endif //SENSOR_SIMULATOR_H