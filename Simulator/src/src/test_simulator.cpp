#include "sensor_simulator.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sensor_simulator_node");
    
    SensorSimulator sensor_simulator(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}