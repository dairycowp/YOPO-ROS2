#ifndef NETWORK_CONTROL_H_
#define NETWORK_CONTROL_H_

#include <Eigen/Eigen>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <so3_control/SO3Control.h>
#include <so3_control/HGDO.h>
#include <so3_control/mavros_interface.h>
#include <quadrotor_msgs/srv/set_takeoff_land.hpp>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <regex>
#include <fstream>
#include <thread>
#include <mutex>
#include <algorithm> 
#include <chrono>
#include <ctime>

#define ONE_G 9.81

class NetworkControl
{
public:
    NetworkControl(std::shared_ptr<rclcpp::Node> node) : mavros_interface_(node) {
        node_ = node;

        so3_controller_.setMass(mass_);
        disturbance_observer_ = HGDO(control_dt_);

        node_->declare_parameter("is_simulation", false);
        node_->declare_parameter("use_disturbance_observer", false);
        node_->declare_parameter("hover_thrust", 0.4);
        node_->declare_parameter("kx_xy", 5.7);
        node_->declare_parameter("kx_z", 6.2);
        node_->declare_parameter("kv_xy", 3.4);
        node_->declare_parameter("kv_z", 4.0);
        node_->declare_parameter("record_log", false);
        node_->declare_parameter("logger_file_name", std::string("/home/lu/"));
        
        node_->get_parameter("is_simulation", is_simulation_);
        node_->get_parameter("use_disturbance_observer", use_disturbance_observer_);
        node_->get_parameter("hover_thrust", hover_thrust_);
        node_->get_parameter("kx_xy", kx_xy);
        node_->get_parameter("kx_z", kx_z);
        node_->get_parameter("kv_xy", kv_xy);
        node_->get_parameter("kv_z", kv_z);
        node_->get_parameter("record_log", record_log_);
        node_->get_parameter("logger_file_name", logger_file_name);
        
        printf("kx: (%f, %f, %f), kv: (%f, %f, %f) \n", kx_xy, kx_xy, kx_z, kv_xy, kv_xy, kv_z);

        so3_command_pub_ = node_->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);
        position_cmd_sub_ = node_->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "position_cmd", 1, 
            [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
                this->network_cmd_callback(msg);
            });
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 1,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odom_callback(msg);
            });
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 1,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->imu_callback(*msg);
            });

        takeoff_land_control_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds((int)(control_dt_ * 1000)),
            [this]() {
                this->timerCallback();
            });

        takeoff_land_srv_ = node_->create_service<quadrotor_msgs::srv::SetTakeoffLand>(
            "takeoff_land",
            [this](const std::shared_ptr<quadrotor_msgs::srv::SetTakeoffLand::Request> req,
                   std::shared_ptr<quadrotor_msgs::srv::SetTakeoffLand::Response> res) {
                return this->takeoff_land_srv_handle(req, res);
            });

        if (is_simulation_) {
            // Sleep for 2 seconds
            rclcpp::sleep_for(std::chrono::seconds(2));
            // 恢复自动调用起飞功能
            std::thread(&NetworkControl::simulateTakeoff, this).detach();
        }
        
    };

    ~NetworkControl(){};

    // 恢复simulateTakeoff函数，使用异步方式避免executor冲突
    void simulateTakeoff() {
        // Create a client for the takeoff/land service
        auto client = node_->create_client<quadrotor_msgs::srv::SetTakeoffLand>("takeoff_land");
        auto request = std::make_shared<quadrotor_msgs::srv::SetTakeoffLand::Request>();
        request->takeoff = true;
        request->takeoff_altitude = 2.0;
        
        // Wait for service to be available in a separate thread to avoid blocking
        std::thread service_call_thread([this, client, request]() {
            // Wait for service to be available
            while (!client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
            }
        
            // Send the request asynchronously
            auto result_future = client->async_send_request(request);
            
            // We don't wait for the result here to avoid executor conflicts
            RCLCPP_INFO(node_->get_logger(), "Takeoff request sent asynchronously");
        });
        
        service_call_thread.detach();
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Service<quadrotor_msgs::srv::SetTakeoffLand>::SharedPtr takeoff_land_srv_;
    rclcpp::TimerBase::SharedPtr takeoff_land_control_timer_;
    std::mutex mutex_;

    double mass_ = 0.98;
    double control_dt_ = 0.02;
    double hover_thrust_ = 0.4;
    double kx_xy, kx_z, kv_xy, kv_z;
    
    double cur_yaw_ = 0;
    Eigen::Vector3d cur_pos_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d cur_vel_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d cur_acc_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond cur_att_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d dis_acc_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d last_des_acc_ = Eigen::Vector3d(0, 0, 0);
    double last_thrust_ = 0;

    Eigen::Vector3d des_pos_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d des_vel_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d des_acc_ = Eigen::Vector3d(0, 0, 0);
    double des_yaw_ = 0;
    double des_yaw_dot_ = 0;

    bool is_simulation_ = false;
    bool state_init_ = false;
    bool ref_valid_ = false;
    bool ctrl_valid_ = false;
    bool position_cmd_init_ = false;
    bool takeoff_cmd_init_ = false;
    bool use_disturbance_observer_ = false;
    bool record_log_ = false;
    
    SO3Control so3_controller_;
    HGDO disturbance_observer_;
    Mavros_Interface mavros_interface_;
    
    std::ofstream logger;
    std::string logger_file_name;

    void initLogRecorder();

    void recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw);

    Eigen::Vector3d publishHoverSO3Command(Eigen::Vector3d des_pos_, Eigen::Vector3d des_vel_, Eigen::Vector3d des_acc_, double des_yaw_, double des_yaw_dot_);

    void get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw, Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des);

    void pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw);

    void limite_acc(Eigen::Vector3d &acc);

    void network_cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void imu_callback(const sensor_msgs::msg::Imu &imu);

    void timerCallback();

    // mavros interface
    bool takeoff_land_srv_handle(const std::shared_ptr<quadrotor_msgs::srv::SetTakeoffLand::Request> req,
                                 std::shared_ptr<quadrotor_msgs::srv::SetTakeoffLand::Response> res){
        std::thread t(&NetworkControl::takeoff_land_thread, this, std::ref(*req));
        t.detach();
        res->res = true;
        return true;
    }

    bool arm_disarm_vehicle(bool arm);

    void takeoff_land_thread(quadrotor_msgs::srv::SetTakeoffLand::Request &req);

    // 移除simulateTakeoff函数，避免节点重复添加到executor的问题
    /*
    void simulateTakeoff() {
        // Create a client for the takeoff/land service
        auto client = node_->create_client<quadrotor_msgs::srv::SetTakeoffLand>("takeoff_land");
        auto request = std::make_shared<quadrotor_msgs::srv::SetTakeoffLand::Request>();
        request->takeoff = true;
        request->takeoff_altitude = 2.0;
        
        // Wait for service to be available
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }
    
        // Send the request
        auto result = client->async_send_request(request);
        
        // Wait for the result
        if (rclcpp::spin_until_future_complete(node_, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node_->get_logger(), "Takeoff called successfully");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call takeoff service");
        }
    }
    */
};

#endif