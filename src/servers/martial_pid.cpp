#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "ar_interfaces/action/movement.hpp"
#include "cpp/controller/pid.hpp"

class PIDServerNode : public rclcpp::Node
{
public:
    using Movement = ar_interfaces::action::Movement;
    using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

    PIDServerNode()
    : Node("pid_server_x")
    {
        this->declare_parameter<double>("pid.vx.kp", 0.07);
        this->declare_parameter<double>("pid.vx.ki", 0.0);
        this->declare_parameter<double>("pid.vx.kd", 0.0);

        pid_vx_ = std::make_unique<PID>(
            this->get_parameter("pid.vx.kp").as_double(),
            this->get_parameter("pid.vx.ki").as_double(),
            this->get_parameter("pid.vx.kd").as_double(),
            -1.0f, 1.0f, 10,
            PROPORTIONAL_ON_ERROR,
            DIRECT,
            AUTOMATIC
        );

        sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sick_data", 10,
            std::bind(&PIDServerNode::sickCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Movement>(
            this,
            "/movement_pid",
            std::bind(&PIDServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PIDServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&PIDServerNode::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Simple X-direction PID server started");
    }

private:
    rclcpp_action::Server<Movement>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sick_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::unique_ptr<PID> pid_vx_;

    std::vector<float> sick_vals_{0.0f, 0.0f};   // FL, FR only
    bool sick_ready_{false};

    float goal_sp_fl_{0.0f};
    float goal_sp_fr_{0.0f};
    bool goal_active_{false};

    void sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            sick_vals_[0] = msg->data[0]; // FL
            sick_vals_[1] = msg->data[1]; // FR
            sick_ready_ = true;
        }
    }

    void pidLoop()
    {
        if (!goal_active_ || !sick_ready_) return;

        float FL = sick_vals_[0];
        float FR = sick_vals_[1];

        float error_x = ((FL - goal_sp_fl_) + (FR - goal_sp_fr_)) * 0.5f;

        pid_vx_->input = error_x;
        pid_vx_->setpoint = 0.0f;
        pid_vx_->compute();

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -(pid_vx_->output);  // forward/backward only
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_pub_->publish(cmd);

        RCLCPP_DEBUG(this->get_logger(), "X error: %.3f | cmd_x: %.3f",
                     error_x, cmd.linear.x);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Movement::Goal> goal)
    {
        if (goal->setpoint_sick.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Goal rejected: need FL & FR setpoints");
            return rclcpp_action::GoalResponse::REJECT;
        }

        goal_sp_fl_ = goal->setpoint_sick[0] / 10.0f;
        goal_sp_fr_ = goal->setpoint_sick[1] / 10.0f;

        goal_active_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted | FL: %.2f FR: %.2f",
                    goal_sp_fl_, goal_sp_fr_);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMovement>)
    {
        goal_active_ = false;
        RCLCPP_WARN(this->get_logger(), "PID goal cancelled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
    {
        std::thread([this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleMovement>)
    {
        rclcpp::Rate rate(100);

        while (rclcpp::ok() && goal_active_ && sick_ready_) {
            pidLoop();
            rate.sleep();
        }

        goal_active_ = false;
        RCLCPP_INFO(this->get_logger(), "PID execution finished");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDServerNode>());
    rclcpp::shutdown();
    return 0;
}
