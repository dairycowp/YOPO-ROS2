#include "so3_control/NetworkControl.h"
#include <tf2/utils.h>

void NetworkControl::initLogRecorder()
{   
    // Use file count as name to avoid date confusion
    std::cout << "logger_file_name: " << logger_file_name << std::endl;
    int max_number = -1;
    boost::filesystem::path dir(logger_file_name);
    if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
        std::regex filename_pattern(R"(^(\d+)_.*$)"); // 匹配以数字开头，并以"_"分隔的文件名
        for (const auto& entry : boost::filesystem::directory_iterator(dir)) {
            if (boost::filesystem::is_regular_file(entry)) {
                std::string filename = entry.path().filename().string();
                std::smatch match;
                if (std::regex_match(filename, match, filename_pattern)) {
                    int number = std::stoi(match[1]); // 提取第一个"_"前的数字
                    max_number = std::max(max_number, number);
                }
            }
        }
    } else {
        std::cerr << "Error: invalid logger path" << std::endl;
    }
    std::string fileCountStr = std::to_string(max_number + 1);
    std::string temp_file_name = logger_file_name + fileCountStr + "_Net_logger_";
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
    std::cout << "logger: " << temp_file_name << std::endl;
    if (!logger.is_open())
    {
        std::cout << "cannot open the logger." << std::endl;
    }
    else
    {
        logger << "timestamp" << ',';
        logger << "cur_px" << ',';
        logger << "cur_py" << ',';
        logger << "cur_pz" << ',';
        logger << "cur_vx" << ',';
        logger << "cur_vy" << ',';
        logger << "cur_vz" << ',';
        logger << "cur_ax" << ',';
        logger << "cur_ay" << ',';
        logger << "cur_az" << ',';
        logger << "des_px" << ',';
        logger << "des_py" << ',';
        logger << "des_pz" << ',';
        logger << "des_vx" << ',';
        logger << "des_vy" << ',';
        logger << "des_vz" << ',';
        logger << "des_ax" << ',';
        logger << "des_ay" << ',';
        logger << "des_az" << ',';
        logger << "dis_ax" << ',';
        logger << "dis_ay" << ',';
        logger << "dis_az" << ',';
        logger << "px4_ax" << ',';
        logger << "px4_ay" << ',';
        logger << "px4_az" << ',';
        logger << "thrust" << ',';
        logger << "cur_yaw" << ',';
        logger << "des_yaw" << std::endl;
    }
}

void NetworkControl::recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw)
{
    if (logger.is_open())
    {
        logger << node_->now().nanoseconds() << ',';
        logger << cur_pos_(0) << ',';
        logger << cur_pos_(1) << ',';
        logger << cur_pos_(2) << ',';
        logger << cur_v(0) << ',';
        logger << cur_v(1) << ',';
        logger << cur_v(2) << ',';
        logger << cur_a(0) << ',';
        logger << cur_a(1) << ',';
        logger << cur_a(2) << ',';
        logger << des_pos_(0) << ',';
        logger << des_pos_(1) << ',';
        logger << des_pos_(2) << ',';
        logger << des_vel_(0) << ',';
        logger << des_vel_(1) << ',';
        logger << des_vel_(2) << ',';
        logger << des_a(0) << ',';
        logger << des_a(1) << ',';
        logger << des_a(2) << ',';
        logger << dis_a(0) << ',';
        logger << dis_a(1) << ',';
        logger << dis_a(2) << ',';
        logger << des_a(0) - dis_a(0) << ',';
        logger << des_a(1) - dis_a(1) << ',';
        logger << des_a(2) - dis_a(2) << ',';
        logger << last_thrust_ << ',';
        logger << cur_yaw << ',';
        logger << des_yaw << std::endl;
    }
}

Eigen::Vector3d NetworkControl::publishHoverSO3Command(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, 
                                                       Eigen::Vector3d des_acc, double des_yaw, double des_yaw_dot)
{
    Eigen::Vector3d kx(kx_xy, kx_xy, kx_z);
    Eigen::Vector3d kv(kv_xy, kv_xy, kv_z);
    so3_controller_.calculateControl(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot, kx, kv);

    Eigen::Vector3d force = so3_controller_.getComputedForce();
    Eigen::Quaterniond orientation = so3_controller_.getComputedOrientation();

    auto so3_command = quadrotor_msgs::msg::SO3Command();
    so3_command.header.stamp = node_->now();
    so3_command.force.x = force(0);
    so3_command.force.y = force(1);
    so3_command.force.z = force(2);
    so3_command.orientation.x = orientation.x();
    so3_command.orientation.y = orientation.y();
    so3_command.orientation.z = orientation.z();
    so3_command.orientation.w = orientation.w();
    so3_command.k_r[0] = 1.5;
    so3_command.k_r[1] = 1.5;
    so3_command.k_r[2] = 1.0;
    so3_command.k_om[0] = 0.13;
    so3_command.k_om[1] = 0.13;
    so3_command.k_om[2] = 0.1;
    so3_command.aux.current_yaw = cur_yaw_;
    so3_command.aux.enable_motors = true;
    so3_command_pub_->publish(so3_command);

    double thrust_norm = force.norm() / (mass_ * ONE_G) * hover_thrust_;
    mavros_interface_.pub_att_thrust_cmd(orientation, thrust_norm);
    last_thrust_ = thrust_norm;

    double thrust = force.norm() / mass_;
    Eigen::Matrix3d Cbn;    
    get_dcm_from_q(Cbn, orientation);
    Eigen::Vector3d att_acc = Eigen::Vector3d(0, 0, thrust);
    att_acc = Cbn * att_acc;
    att_acc(2) -= ONE_G;
    // std::cout<<"att_acc"<<att_acc.transpose()<<std::endl;
    return att_acc;
}

void NetworkControl::get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw, Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des)
{
    Eigen::Vector3d force_ = mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    force_.noalias() += mass_ * ref_acc;

    // Limit control angle to theta degree
    double theta = M_PI / 4;
    double c = cos(theta);
    Eigen::Vector3d f;
    f.noalias() = force_ - mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
    {
        double nf = f.norm();
        double A = c * c * nf * nf - f(2) * f(2);
        double B = 2 * (c * c - 1) * f(2) * mass_ * ONE_G;
        double C = (c * c - 1) * mass_ * mass_ * ONE_G * ONE_G;
        double s = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
        force_.noalias() = s * f + mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    }

    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(ref_yaw), sin(ref_yaw), 0);

    if (force_.norm() > 1e-6)
        b3c.noalias() = force_.normalized();
    else
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    b2c.noalias() = b3c.cross(b1d).normalized();
    b1c.noalias() = b2c.cross(b3c).normalized();

    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;

    quat_des = Eigen::Quaterniond(R);
    force_des = force_;

}

// 世界系的期望加速度：ref_acc（加上g）、期望yaw：ref_yaw
void NetworkControl::pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw)
{
    Eigen::Vector3d force;
    Eigen::Quaterniond quat_des;
    get_Q_from_ACC(ref_acc, ref_yaw, quat_des, force);
    auto so3_command = quadrotor_msgs::msg::SO3Command();
    so3_command.header.stamp = node_->now();
    so3_command.force.x = force(0);
    so3_command.force.y = force(1);
    so3_command.force.z = force(2);
    so3_command.orientation.x = quat_des.x();
    so3_command.orientation.y = quat_des.y();
    so3_command.orientation.z = quat_des.z();
    so3_command.orientation.w = quat_des.w();
    so3_command.k_r[0] = 1.5;
    so3_command.k_r[1] = 1.5;
    so3_command.k_r[2] = 1.0;
    so3_command.k_om[0] = 0.13;
    so3_command.k_om[1] = 0.13;
    so3_command.k_om[2] = 0.1;
    so3_command.aux.current_yaw = cur_yaw;
    so3_command.aux.enable_motors = true;
    so3_command_pub_->publish(so3_command);

    double thrust_norm = force.norm() / (mass_ * ONE_G) * hover_thrust_;
    mavros_interface_.pub_att_thrust_cmd(quat_des, thrust_norm);
    last_thrust_ = thrust_norm;
}

void NetworkControl::limite_acc(Eigen::Vector3d &acc){
    return;  // limited if needed
    acc[0] = std::max(-8.0, std::min(acc[0], 8.0));
    acc[1] = std::max(-8.0, std::min(acc[1], 8.0));
    acc[2] = std::max(-4.0, std::min(acc[2], 4.0));
}

void NetworkControl::network_cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
{
    if (!ctrl_valid_)
        return;

    bool arm_state = false;
    bool ofb_enable = false;
    mavros_interface_.get_status(arm_state, ofb_enable);
    if (!arm_state || !ofb_enable)
        return;

    position_cmd_init_ = true;

    Eigen::Vector3d des_acc = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    limite_acc(des_acc);

    double des_yaw = cmd->yaw;
    
    disturbance_observer_.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), 0.5, "dis_acc: %.3f, %.3f, %.3f", dis_acc_.x(), dis_acc_.y(), dis_acc_.z());
    // std::cout << "dis_acc: " << dis_acc_.transpose() << std::endl;

    Eigen::Vector3d att_acc;
    if (cmd->trajectory_flag == quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY)
    {    
        if (use_disturbance_observer_)
            att_acc = des_acc - dis_acc_;
        else
            att_acc = des_acc;
        pub_SO3_command(att_acc, des_yaw, cur_yaw_);
        // std::cout<<"acc: "<<des_acc.transpose()<<"   yaw:"<<des_yaw<<std::endl;
        if (record_log_)
            recordLog(cur_vel_, cur_acc_, des_acc, dis_acc_, cur_yaw_, des_yaw);
    }
    else
    {
        Eigen::Vector3d des_pos = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
        Eigen::Vector3d des_vel = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
        double des_yaw = cmd->yaw;
        double des_yaw_dot = cmd->yaw_dot;
        att_acc = publishHoverSO3Command(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot);
        if (record_log_)
            recordLog(cur_vel_, cur_acc_, att_acc, dis_acc_, cur_yaw_, des_yaw);
    }

    last_des_acc_ = att_acc;
}

void NetworkControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    cur_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
    cur_vel_ = Eigen::Vector3d(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

    cur_pos_ = Eigen::Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    cur_att_.w() = odom->pose.pose.orientation.w;
    cur_att_.x() = odom->pose.pose.orientation.x;
    cur_att_.y() = odom->pose.pose.orientation.y;
    cur_att_.z() = odom->pose.pose.orientation.z;

    // if(!is_simulation_)
    //     cur_acc_ = Eigen::Vector3d(odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z);

    so3_controller_.setPosition(cur_pos_);
    so3_controller_.setVelocity(cur_vel_);
    if (!state_init_)
        RCLCPP_INFO(node_->get_logger(), "Odom Recived! Ready to TakeOff...");
    state_init_ = true;
}

void NetworkControl::imu_callback(const sensor_msgs::msg::Imu &imu)
{
    Eigen::Vector3d acc(imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z);
    Eigen::Vector3d acc_world = cur_att_ * acc;
    acc_world(2) -= 9.8;
    cur_acc_ = acc_world;
    // so3_controller_.setAcc(acc_world);
}

void NetworkControl::timerCallback()
{
    if (!state_init_ || !ref_valid_)
        return;
    if (position_cmd_init_ && ctrl_valid_)
        return;

    mutex_.lock();
    Eigen::Vector3d des_pos_temp = des_pos_;
    mutex_.unlock();

    Eigen::Vector3d att_acc = publishHoverSO3Command(des_pos_temp, des_vel_, des_acc_, des_yaw_, des_yaw_dot_);

    if (takeoff_cmd_init_)
    {
        disturbance_observer_.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), 1.0, " dis_acc: (%f, %f, %f)", dis_acc_.x(), dis_acc_.y(), dis_acc_.z());
    }

    last_des_acc_ = att_acc;
    if (record_log_)
        recordLog(cur_vel_, cur_acc_, att_acc, dis_acc_, cur_yaw_, des_yaw_);
    takeoff_cmd_init_ = true;
}

void NetworkControl::takeoff_land_thread(quadrotor_msgs::srv::SetTakeoffLand::Request &req)
{
    mutex_.lock();
    float takeoff_altitude = req.takeoff_altitude;
    des_pos_ = cur_pos_;
    des_pos_(2) -= 0.2;
    des_yaw_ = cur_yaw_;
    mutex_.unlock();
    ref_valid_ = true;

    if (req.takeoff)
    {
        std::cout << "takeoff process start" << std::endl;
        if (!arm_disarm_vehicle(true))
        {
            std::cout << "Service failed because cannot Arm!" << std::endl;
            return;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        double takeoff_vel = 0.8;
        double takeoff_ddz = takeoff_vel * control_dt_;
        rclcpp::Rate takeoff_loop(1 / control_dt_);
        std::cout << "takeoff altitude: " << takeoff_altitude << " m" << std::endl;
        std::cout << "takeoff velocity: " << takeoff_vel << " m/s" << std::endl;
        auto start_takeoff_task_time = node_->now();
        while (rclcpp::ok() && (node_->now() - start_takeoff_task_time).seconds() < 8.0)
        {       
            mutex_.lock();
            des_pos_(2) += takeoff_ddz;
            mutex_.unlock();

            if (des_pos_(2) > takeoff_altitude)
            {
                RCLCPP_INFO(node_->get_logger(), "TakeOff Done! Ready to Flight...");
                ctrl_valid_ = true;
                break;
            }
            takeoff_loop.sleep();
        }
    }
    else
    {
        ctrl_valid_ = false;
        double land_vel = -0.4;
        double land_ddz = land_vel * control_dt_;
        rclcpp::Rate land_loop(1 / control_dt_);
        auto start_land_task_time = node_->now();
        while (rclcpp::ok() && (node_->now() - start_land_task_time).seconds() < 8.0)
        {
            mutex_.lock();
            des_pos_(2) += land_ddz;
            mutex_.unlock();

            if (fabs(cur_pos_(2)) < 0.1f && fabs(cur_vel_(2)) < 1.0f)
            {
                RCLCPP_INFO(node_->get_logger(), "detect land: disarm");
                arm_disarm_vehicle(false);
                break;
            }
            land_loop.sleep();
        }
    }
    RCLCPP_INFO(node_->get_logger(), "take off thread out");
    return;
}

bool NetworkControl::arm_disarm_vehicle(bool arm)
{
    if (arm)
    {   
        if (!state_init_){
            RCLCPP_WARN(node_->get_logger(), "State timeout, will not arm!");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "UAV will be armed!");
        if (is_simulation_)
            mavros_interface_.set_arm_and_offboard_manually();
        else if (mavros_interface_.set_arm_and_offboard())
            RCLCPP_INFO(node_->get_logger(), "Arm done!");
        else{
            RCLCPP_ERROR(node_->get_logger(), "Arm failure!");
            return false;
        }
        if (record_log_)
            initLogRecorder();
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "UAV will be disarmed!");
        if (is_simulation_)
            mavros_interface_.set_disarm_manually();
        else if (mavros_interface_.set_disarm())
            RCLCPP_INFO(node_->get_logger(), "Disarm done!");
        else {
            RCLCPP_ERROR(node_->get_logger(), "Disarm failure!");
            return false;
        }
        if (record_log_)
            logger.close();
    }
    return true;
}