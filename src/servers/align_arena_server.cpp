#include "servers/align_arena.hpp"
#include <cmath>
#include <thread>

AlignArenaServer::AlignArenaServer(const rclcpp::NodeOptions & options)
: Node("align_arena_server", options)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    this->declare_parameter("yaw_kp", 1.7);
    this->declare_parameter("yaw_ki", 0.0);
    this->declare_parameter("yaw_kd", 0.2);
    this->declare_parameter("x_kp", 0.009);
    this->declare_parameter("x_ki", 0.0);
    this->declare_parameter("x_kd", 0.003);
    this->declare_parameter("y_kp", 0.004);
    this->declare_parameter("y_ki", 0.0);
    this->declare_parameter("y_kd", 0.002);
    this->declare_parameter("dist_tol", 15.0);
    this->declare_parameter("max_vel", 3.0);
    this->declare_parameter("sample_time", 10);

    dist_tol_    = this->get_parameter("dist_tol").as_double();
    max_vel_     = this->get_parameter("max_vel").as_double();
    sample_time_ = this->get_parameter("sample_time").as_int();

    yaw_pid_ = PID(this->get_parameter("yaw_kp").as_double(),
                   this->get_parameter("yaw_ki").as_double(),
                   this->get_parameter("yaw_kd").as_double(),
                   -max_vel_, max_vel_, sample_time_);

    x_pid_ = PID(this->get_parameter("x_kp").as_double(),
                 this->get_parameter("x_ki").as_double(),
                 this->get_parameter("x_kd").as_double(),
                 -max_vel_, max_vel_, sample_time_);

    y_pid_ = PID(this->get_parameter("y_kp").as_double(),
                 this->get_parameter("y_ki").as_double(),
                 this->get_parameter("y_kd").as_double(),
                 -max_vel_, max_vel_, sample_time_);

    sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/sick_data", qos_profile,
        std::bind(&AlignArenaServer::sickCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);

    action_server_ = rclcpp_action::create_server<AlignTwist>(
        this,
        "align_arena_twist",
        std::bind(&AlignArenaServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AlignArenaServer::handleCancel, this, std::placeholders::_1),
        std::bind(&AlignArenaServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[AlignArenaServer] Ready.");
}

void AlignArenaServer::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4) {
        sick_vals_ = msg->data;
        sick_ready_ = true;
    }
}

void AlignArenaServer::pidLoop()
{
    if (!sick_ready_ || !goal_active_) return;

    float FR   = sick_vals_[0] * 10;
    float FL   = sick_vals_[3] * 10;
    // float side = sick_vals_[2];
    float side = 0.0;
    float x_avg = (FL + FR) * 0.5f;

    yaw_pid_.input = FL - FR;
    yaw_pid_.setpoint = 0.0f;
    yaw_pid_.compute();

    // x_pid_.input = x_avg;
    // x_pid_.compute();

    // y_pid_.input = side;
    // y_pid_.compute();

    // publishVel(x_pid_.output, y_pid_.output, yaw_pid_.output);
    publishVel(0.0, 0.0, yaw_pid_.output);

    RCLCPP_INFO(this->get_logger(),
        "[AlignArenaServer] FL=%.1f FR=%.1f side=%.1f | x_avg=%.1f x_sp=%.1f y_sp=%.1f",
        FL, FR, side, x_avg, x_pid_.setpoint, y_pid_.setpoint);
}

void AlignArenaServer::publishVel(float vx, float vy, float omega)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = vx;
    cmd.linear.y  = vy;
    cmd.angular.z = omega;
    cmd_pub_->publish(cmd);
}

rclcpp_action::GoalResponse AlignArenaServer::handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const AlignTwist::Goal> goal)
{
    if (!sick_ready_) {
        RCLCPP_WARN(this->get_logger(), "[AlignArenaServer] No sick data yet, rejecting");
        return rclcpp_action::GoalResponse::REJECT;
    }

    x_pid_.setpoint = goal->target_x;
    y_pid_.setpoint = goal->target_y;
    x_pid_.setMode(AUTOMATIC);
    y_pid_.setMode(AUTOMATIC);
    yaw_pid_.setMode(AUTOMATIC);
    goal_active_ = true;

    RCLCPP_INFO(this->get_logger(), "[AlignArenaServer] Goal received target_x=%.1f target_y=%.1f",
                goal->target_x, goal->target_y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AlignArenaServer::handleCancel(const std::shared_ptr<GoalHandleAlign>)
{
    goal_active_ = false;
    RCLCPP_WARN(this->get_logger(), "[AlignArenaServer] Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AlignArenaServer::handleAccepted(const std::shared_ptr<GoalHandleAlign> goal_handle)
{
    std::thread(std::bind(&AlignArenaServer::execute, this, goal_handle)).detach();
}

void AlignArenaServer::execute(const std::shared_ptr<GoalHandleAlign> goal_handle)
{
    auto result = std::make_shared<AlignTwist::Result>();
    rclcpp::Rate rate(1000 / sample_time_);

    while (rclcpp::ok() && goal_active_)
    {
        if (goal_handle->is_canceling()) {
            publishVel(0.0f, 0.0f, 0.0f);
            result->success = false;
            goal_handle->canceled(result);
            goal_active_ = false;
            return;
        }

        pidLoop();

        float FR = sick_vals_[0] * 10;
        float FL = sick_vals_[3] * 10;
        // float side = sick_vals_[2];
        float side = 0.0;
        float x_avg = (FL + FR) * 0.5f;

        bool yaw_ok = std::fabs(FL - FR) < 0.001;
        bool x_ok   = std::fabs(x_pid_.setpoint - x_avg) < dist_tol_;
        bool y_ok   = std::fabs(y_pid_.setpoint - side) < dist_tol_;

        if (yaw_ok && x_ok && y_ok) {
            publishVel(0.0f, 0.0f, 0.0f);
            result->success = true;
            result->final_x = x_avg;
            result->final_y = side;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "[AlignArenaServer] Aligned.");
            goal_active_ = false;
            return;
        }

        rate.sleep();
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlignArenaServer>());
    rclcpp::shutdown();
    return 0;
}