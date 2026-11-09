#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "robotlibpc/cpp/controller/pid.hpp"
#include "ar_interfaces_2026/action/align_arena_twist.hpp"

using AlignTwist = ar_interfaces_2026::action::AlignArenaTwist;   
using GoalHandleAlign = rclcpp_action::ServerGoalHandle<AlignTwist>;

class AlignArenaServer : public rclcpp::Node
{
public:
    explicit AlignArenaServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void pidLoop();
    void publishVel(float vx, float vy, float omega);

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const AlignTwist::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleAlign> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleAlign> goal_handle);
    void execute(const std::shared_ptr<GoalHandleAlign> goal_handle);

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sick_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Server<AlignTwist>::SharedPtr action_server_;

    PID yaw_pid_;
    PID x_pid_;
    PID y_pid_;

    double dist_tol_{15.0};
    double max_vel_{3.0};
    int sample_time_{10};

    std::vector<float> sick_vals_;   // [front_left, front_right, side]
    bool sick_ready_{false};
    bool goal_active_{false};
};