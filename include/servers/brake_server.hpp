// brake_server.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robotlibpc/cpp/controller/pid.hpp"

class BrakeServer : public rclcpp::Node
{
public:
    explicit BrakeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void handleBrake(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void pidLoop();

    void odomCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void publishVel(float vx, float vy);

     
    PID x_pid_;
    PID y_pid_;

    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                service_;
    rclcpp::TimerBase::SharedPtr                                      pid_timer_;

    double x_kp_, x_ki_, x_kd_;
    double y_kp_, y_ki_, y_kd_;
    double max_brake_vel_;
    int    sample_time_;

    float odom_dist_x_  = 0.0f;
    float odom_dist_y_  = 0.0f;
    bool  odom_fresh_   = false;
    bool  brake_active_ = false;
};