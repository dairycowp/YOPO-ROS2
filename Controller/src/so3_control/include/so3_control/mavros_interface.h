#ifndef MAVROS_INTERFACE_H_
#define MAVROS_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Mavros_Interface
{
public:
    Mavros_Interface(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        state_.reset();
        std::string base_name = "/mavros";
        // char id_str[10];
        // sprintf(id_str, "%d", id);
        // base_name += id_str;

        std::string att_target_pub_name;
        att_target_pub_name = base_name + "/setpoint_raw/attitude";
        att_target_pub_ = node_->create_publisher<mavros_msgs::msg::AttitudeTarget>(att_target_pub_name, 10);

        std::string state_sub_name;
        state_sub_name = base_name + "/state";
        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
            state_sub_name, 10, 
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                this->state_cb(*msg);
            });

        std::string set_mode_s_name;
        set_mode_s_name = base_name + "/set_mode";
        set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(set_mode_s_name);

        std::string arm_disarm_s_name;
        arm_disarm_s_name = base_name + "/cmd/arming";
        arm_disarm_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(arm_disarm_s_name);
    }

    ~Mavros_Interface() {}

    typedef struct mavros_state_t
    {
        rclcpp::Time header;
        bool has_armed;
        bool offboard_enabled;
        void reset()
        {
            has_armed = false;
            offboard_enabled = false;
        }
        mavros_state_t()
        {
            reset();
        }
    };

    void state_cb(const mavros_msgs::msg::State &state_data)
    {
        mavros_msgs::msg::State temp_data = state_data;
        state_.header = state_data.header.stamp;
        state_.has_armed = state_data.armed;
        if (state_data.mode == "OFFBOARD")
        {
            state_.offboard_enabled = true;
        }
        else
        {
            state_.offboard_enabled = false;
        }
    }

    void get_status(bool &arm_state, bool &offboard_enabled)
    {
        arm_state = state_.has_armed;
        offboard_enabled = state_.offboard_enabled;
    }

    bool set_arm_and_offboard()
    {
        rclcpp::Rate ofb_check_rate(1);
        int try_arm_ofb_times = 0;
        while (!state_.offboard_enabled || !state_.has_armed)
        {
            if (state_.offboard_enabled)
            {
                rclcpp::Rate arm_check_rate(1);
                while (!state_.has_armed)
                {
                    auto arm_srv = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    arm_srv->value = true;
                    auto result = arm_disarm_client_->async_send_request(
                        std::make_shared<mavros_msgs::srv::CommandBool::Request>(*arm_srv));
                    // Wait for the result.
                    if (rclcpp::spin_until_future_complete(node_, result) ==
                        rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_INFO(node_->get_logger(), "vehicle ARMED");
                    }
                    try_arm_ofb_times = try_arm_ofb_times + 1;
                    if (try_arm_ofb_times >= 3)
                    {
                        RCLCPP_ERROR(node_->get_logger(), "try 3 times, cannot armed uav, give up!");
                        return false;
                    }
                    arm_check_rate.sleep();
                }
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "not in OFFBOARD mode");
                auto set_mode_srv = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                set_mode_srv->base_mode = 0;
                set_mode_srv->custom_mode = "OFFBOARD";
                auto result = set_mode_client_->async_send_request(
                    std::make_shared<mavros_msgs::srv::SetMode::Request>(*set_mode_srv));
                if (rclcpp::spin_until_future_complete(node_, result) !=
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    return false;
                }
                RCLCPP_INFO(node_->get_logger(), "switch to OFFBOARD mode");
                ofb_check_rate.sleep();
            }
        }
        return true;
    }

    /* void set_arm(const rclcpp::TimerEvent& event) {
        if(state_.offboard_enabled) {
            auto arm_srv = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_srv->value = true;
            auto result = arm_disarm_client_->async_send_request(
                std::make_shared<mavros_msgs::srv::CommandBool::Request>(*arm_srv));
            if (rclcpp::spin_until_future_complete(node_, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(node_->get_logger(), "vehicle ARMED");
            }
        } else {
            RCLCPP_INFO(node_->get_logger(), "not in OFFBOARD mode");
        }
    } */

    bool set_disarm()
    {
        rclcpp::Rate arm_check_rate(1);
        while (state_.has_armed)
        {
            auto arm_srv = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_srv->value = false;
            auto result = arm_disarm_client_->async_send_request(
                std::make_shared<mavros_msgs::srv::CommandBool::Request>(*arm_srv));
            if (rclcpp::spin_until_future_complete(node_, result) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "vehicle DISARMED");
            arm_check_rate.sleep();
        }
        return true;
    }

    void pub_att_thrust_cmd(const Eigen::Quaterniond &q_d, const double &thrust_d)
    {
        /*
            目前mavros用的是北东地的坐标系，为了和pid通用所以没有改mavros而是在这里改为北东地
        */
        auto at_cmd = mavros_msgs::msg::AttitudeTarget();
        at_cmd.header.stamp = node_->now();
        at_cmd.type_mask = at_cmd.IGNORE_ROLL_RATE | at_cmd.IGNORE_PITCH_RATE | at_cmd.IGNORE_YAW_RATE;
        at_cmd.thrust = (float)thrust_d;
        at_cmd.orientation.w = q_d.w();
        at_cmd.orientation.x = q_d.x();
        at_cmd.orientation.y = -q_d.y();
        at_cmd.orientation.z = -q_d.z();
        att_target_pub_->publish(at_cmd);
    }

    // for simulation
    void set_arm_and_offboard_manually()
    {
        state_.has_armed = true;
        state_.offboard_enabled = true;
    }

    void set_disarm_manually()
    {
        state_.has_armed = false;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_target_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_disarm_client_;
    mavros_state_t state_;
};

#endif