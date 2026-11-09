// brake_server.cpp
#include "servers/brake_server.hpp"
#include <cmath>

BrakeServer::BrakeServer(const rclcpp::NodeOptions & options)
: Node("brake_server", options)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    this->declare_parameter("x_kp", 0.1);
    this->declare_parameter("x_ki", 0.0);
    this->declare_parameter("x_kd", 0.0);
    this->declare_parameter("y_kp", 0.1);
    this->declare_parameter("y_ki", 0.0);
    this->declare_parameter("y_kd", 0.0);
    this->declare_parameter("sample_time", 10);
    this->declare_parameter("max_brake_vel", 0.15);

    x_kp_ = this->get_parameter("x_kp").as_double();
    x_ki_ = this->get_parameter("x_ki").as_double();
    x_kd_ = this->get_parameter("x_kd").as_double();
    y_kp_ = this->get_parameter("y_kp").as_double();
    y_ki_ = this->get_parameter("y_ki").as_double();
    y_kd_ = this->get_parameter("y_kd").as_double();
    sample_time_    = this->get_parameter("sample_time").as_int();
    max_brake_vel_  = this->get_parameter("max_brake_vel").as_double();

    x_pid_ = PID(x_kp_, x_ki_, x_kd_, -max_brake_vel_, max_brake_vel_, sample_time_);
    y_pid_ = PID(y_kp_, y_ki_, y_kd_, -max_brake_vel_, max_brake_vel_, sample_time_);

    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/brake_server/cmd_vel", qos_profile);

    odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/odom_dist_m", qos_profile,
        std::bind(&BrakeServer::odomCallback, this, std::placeholders::_1));

    service_ = this->create_service<std_srvs::srv::SetBool>(
        "/brake",
        std::bind(&BrakeServer::handleBrake, this,
                  std::placeholders::_1, std::placeholders::_2));

    pid_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&BrakeServer::pidLoop, this));

    RCLCPP_INFO(this->get_logger(), "[BrakeServer] Ready.");
}

void BrakeServer::handleBrake(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data)
    {
        x_pid_.setpoint = odom_dist_x_;
        y_pid_.setpoint = odom_dist_y_;
        x_pid_.setMode(AUTOMATIC);
        y_pid_.setMode(AUTOMATIC);
        brake_active_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "[BrakeServer] Brake engaged — holding x=%.4f  y=%.4f",
                    odom_dist_x_, odom_dist_y_);

        response->success = true;
        response->message = "Brake engaged";
    }
    else
    {
        brake_active_ = false;
        x_pid_.setMode(MANUAL);
        y_pid_.setMode(MANUAL);
        publishVel(0.0f, 0.0f);

        RCLCPP_INFO(this->get_logger(), "[BrakeServer] Brake released");

        response->success = true;
        response->message = "Brake released";
    }
}

void BrakeServer::pidLoop()
{
    if (!brake_active_ || !odom_fresh_) return;

    x_pid_.input = odom_dist_x_;
    y_pid_.input = odom_dist_y_;

    x_pid_.compute();
    y_pid_.compute();

    publishVel(x_pid_.output, y_pid_.output);

    RCLCPP_INFO(this->get_logger(),
                "[BrakeServer] x_sp=%.4f x_in=%.4f x_out=%.4f | y_sp=%.4f y_in=%.4f y_out=%.4f",
                x_pid_.setpoint, x_pid_.input, x_pid_.output,
                y_pid_.setpoint, y_pid_.input, y_pid_.output);
}

void BrakeServer::odomCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(this->get_logger(),
                    "[BrakeServer] Odom data size < 2: %zu", msg->data.size());
        return;
    }
    odom_dist_x_ = msg->data[0];
    odom_dist_y_ = msg->data[1];
    odom_fresh_  = true;
}

void BrakeServer::publishVel(float vx, float vy)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = vx;
    twist.linear.y  = vy;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrakeServer>());
    rclcpp::shutdown();
    return 0;
}