#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_control/SO3Control.h>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <tf2/utils.h>

class SO3ControlNode : public rclcpp::Node
{
public:
  SO3ControlNode() : rclcpp::Node("so3_control_node")
  {
    initialize();
  }

  void initialize()
  {
    std::string quadrotor_name;
    this->declare_parameter("quadrotor_name", "quadrotor");
    this->get_parameter("quadrotor_name", quadrotor_name);
    frame_id_ = "/" + quadrotor_name;

    double mass;
    this->declare_parameter("mass", 0.5);
    this->declare_parameter("record_log", false);
    this->declare_parameter("PID_logger_file_name", std::string("/home/lu/"));
    this->declare_parameter("use_external_yaw", true);
    
    this->declare_parameter("gains.rot.x", 1.5);
    this->declare_parameter("gains.rot.y", 1.5);
    this->declare_parameter("gains.rot.z", 1.0);
    this->declare_parameter("gains.ang.x", 0.13);
    this->declare_parameter("gains.ang.y", 0.13);
    this->declare_parameter("gains.ang.z", 0.1);
    this->declare_parameter("gains.kx.x", 5.7);
    this->declare_parameter("gains.kx.y", 5.7);
    this->declare_parameter("gains.kx.z", 6.2);
    this->declare_parameter("gains.kv.x", 3.4);
    this->declare_parameter("gains.kv.y", 3.4);
    this->declare_parameter("gains.kv.z", 4.0);
    
    this->declare_parameter("corrections.z", 0.0);
    this->declare_parameter("corrections.r", 0.0);
    this->declare_parameter("corrections.p", 0.0);
    
    this->declare_parameter("so3_control.init_state_x", 0.0);
    this->declare_parameter("so3_control.init_state_y", 0.0);
    this->declare_parameter("so3_control.init_state_z", -10000.0);

    this->get_parameter("mass", mass);
    controller_.setMass(mass);
    
    this->get_parameter("record_log", record_log_);
    this->get_parameter("PID_logger_file_name", logger_file_name);
    this->get_parameter("use_external_yaw", use_external_yaw_);

    this->get_parameter("gains.rot.x", kR_[0]);
    this->get_parameter("gains.rot.y", kR_[1]);
    this->get_parameter("gains.rot.z", kR_[2]);
    this->get_parameter("gains.ang.x", kOm_[0]);
    this->get_parameter("gains.ang.y", kOm_[1]);
    this->get_parameter("gains.ang.z", kOm_[2]);
    this->get_parameter("gains.kx.x", kx_[0]);
    this->get_parameter("gains.kx.y", kx_[1]);
    this->get_parameter("gains.kx.z", kx_[2]);
    this->get_parameter("gains.kv.x", kv_[0]);
    this->get_parameter("gains.kv.y", kv_[1]);
    this->get_parameter("gains.kv.z", kv_[2]);

    this->get_parameter("corrections.z", corrections_[0]);
    this->get_parameter("corrections.r", corrections_[1]);
    this->get_parameter("corrections.p", corrections_[2]);

    this->get_parameter("so3_control.init_state_x", init_x_);
    this->get_parameter("so3_control.init_state_y", init_y_);
    this->get_parameter("so3_control.init_state_z", init_z_);

    so3_command_pub_ = this->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_callback(msg); });
      
    position_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "position_cmd", 10, [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) { this->position_cmd_callback(msg); });

    enable_motors_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "motors", 2, [this](const std_msgs::msg::Bool::SharedPtr msg) { this->enable_motors_callback(msg); });
      
    corrections_sub_ = this->create_subscription<quadrotor_msgs::msg::Corrections>(
      "corrections", 10, [this](const quadrotor_msgs::msg::Corrections::SharedPtr msg) { this->corrections_callback(msg); });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->imu_callback(*msg); });
      
    if (record_log_)
      initLogRecorder();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void position_cmd_callback(
    const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void enable_motors_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void corrections_callback(const quadrotor_msgs::msg::Corrections::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu& imu);

  void initLogRecorder();
  void recordLog();

  SO3Control      controller_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr corrections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  bool        position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  bool record_log_{false};
  bool cur_acc_init_{false}, cur_odom_init_{false};
  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  Eigen::Vector3d cur_pos_, cur_vel_, cur_acc_;
  double          des_yaw_, des_yaw_dot_;
  double          current_yaw_;
  bool            enable_motors_;
  bool            use_external_yaw_;
  double          kR_[3], kOm_[3], corrections_[3];
  double          init_x_, init_y_, init_z_;

  std::ofstream logger;
  std::string logger_file_name;
};

void
SO3ControlNode::publishSO3Command(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);

  const Eigen::Vector3d&    force       = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

  auto so3_command = quadrotor_msgs::msg::SO3Command();
  so3_command.header.stamp    = this->now();
  so3_command.header.frame_id = frame_id_;
  so3_command.force.x         = force(0);
  so3_command.force.y         = force(1);
  so3_command.force.z         = force(2);
  so3_command.orientation.x   = orientation.x();
  so3_command.orientation.y   = orientation.y();
  so3_command.orientation.z   = orientation.z();
  so3_command.orientation.w   = orientation.w();
  for (int i = 0; i < 3; i++)
  {
    so3_command.k_r[i]  = kR_[i];
    so3_command.k_om[i] = kOm_[i];
  }
  so3_command.aux.current_yaw          = current_yaw_;
  so3_command.aux.kf_correction        = corrections_[0];
  so3_command.aux.angle_corrections[0] = corrections_[1];
  so3_command.aux.angle_corrections[1] = corrections_[2];
  so3_command.aux.enable_motors        = enable_motors_;
  so3_command.aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_->publish(so3_command);
}

void
SO3ControlNode::position_cmd_callback(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);

  if ( cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5 )
  {
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  }
  if ( cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5 )
  {
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishSO3Command();

  if (record_log_ && cur_acc_init_ && cur_odom_init_)
  {
    recordLog();
  }
  
}

void
SO3ControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  cur_odom_init_ = true;
  current_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
  cur_pos_ = position;
  cur_vel_ = velocity;

  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)
      publishSO3Command();
    position_cmd_updated_ = false;
  }
  else if ( init_z_ > -9999.0 )
  {
    des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des_vel_ = Eigen::Vector3d(0,0,0);
    des_acc_ = Eigen::Vector3d(0,0,0);
    publishSO3Command();
  }
  
}

void
SO3ControlNode::enable_motors_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data)
    RCLCPP_INFO(this->get_logger(), "Enabling motors");
  else
    RCLCPP_INFO(this->get_logger(), "Disabling motors");

  enable_motors_ = msg->data;
}

void
SO3ControlNode::corrections_callback(
  const quadrotor_msgs::msg::Corrections::SharedPtr msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void
SO3ControlNode::imu_callback(const sensor_msgs::msg::Imu& imu)
{
  cur_acc_init_ = true;
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  cur_acc_ = acc;
  controller_.setAcc(acc);
}

void SO3ControlNode::initLogRecorder()
{
  std::cout << "logger_file_name: " << logger_file_name << std::endl;
  std::string temp_file_name = logger_file_name + "PID_logger_";
  time_t timep;
  timep = time(0);
  char tmp[64];
  strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
  temp_file_name += tmp;
  temp_file_name += ".csv";
  if (logger.is_open())
  {
    logger.close();
  }
  logger.open(temp_file_name.c_str(), std::ios::out);
  std::cout << "PID logger: " << temp_file_name << std::endl;
  if (!logger.is_open())
  {
    std::cout << "cannot open the logger." << std::endl;
  }
  else
  {
    logger << "timestamp" << ',';
    logger << "cur_x" << ',';
    logger << "cur_y" << ',';
    logger << "cur_z" << ',';
    logger << "cur_vx" << ',';
    logger << "cur_vy" << ',';
    logger << "cur_vz" << ',';
    logger << "cur_ax" << ',';
    logger << "cur_ay" << ',';
    logger << "cur_az" << ',';
    logger << "not_use" << ',';
    logger << "not_use" << ',';
    logger << "cur_yaw" << ',';
    logger << "des_yaw" << ',';
    logger << "des_pos_x" << ',';
    logger << "des_pos_y" << ',';
    logger << "des_pos_z" << ',';
    logger << "des_vel_x" << ',';
    logger << "des_vel_y" << ',';
    logger << "des_vel_z" << ',';
    logger << "des_acc_x" << ',';
    logger << "des_acc_y" << ',';
    logger << "des_acc_z" << std::endl;
  }
}

void SO3ControlNode::recordLog(){
  if (logger.is_open())
  {
    logger << this->now().nanoseconds() << ',';
    logger << cur_pos_(0) << ',';
    logger << cur_pos_(1) << ',';
    logger << cur_pos_(2) << ',';
    logger << cur_vel_(0) << ',';
    logger << cur_vel_(1) << ',';
    logger << cur_vel_(2) << ',';
    logger << cur_acc_(0) << ',';
    logger << cur_acc_(1) << ',';
    logger << cur_acc_(2) << ',';
    logger << 0.0 << ',';
    logger << 0.0 << ',';
    logger << current_yaw_ << ',';
    logger << des_yaw_ << ',';
    logger << des_pos_(0) << ',';
    logger << des_pos_(1) << ',';
    logger << des_pos_(2) << ',';
    logger << des_vel_(0) << ',';
    logger << des_vel_(1) << ',';
    logger << des_vel_(2) << ',';
    logger << des_acc_(0) << ',';
    logger << des_acc_(1) << ',';
    logger << des_acc_(2) << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SO3ControlNode>());
  rclcpp::shutdown();
  return 0;
}