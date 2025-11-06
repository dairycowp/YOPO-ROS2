#include "so3_control/NetworkControl.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("network_ctrl_node");
    NetworkControl controller(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}