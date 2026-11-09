#include "servers/rotate_server.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#define DEG2RAD 0.0174533

namespace RotateExecServer
{

RotateServer::RotateServer(const rclcpp::NodeOptions & options)
: Node("rotate_exec_server", options)
, yaw_pid_()          
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    this->declare_parameter("yaw_kp",            1.7);
    this->declare_parameter("yaw_ki",            0.0);
    this->declare_parameter("yaw_kd",            0.2);
    this->declare_parameter("yaw_tolerance_rad", 0.02);
    this->declare_parameter("max_omega_p",        5.0);
    this->declare_parameter("max_omega_n",       -5.0);
    this->declare_parameter("sample_time_ms",     10);

    yaw_kp_           = this->get_parameter("yaw_kp").as_double();
    yaw_ki_           = this->get_parameter("yaw_ki").as_double();
    yaw_kd_           = this->get_parameter("yaw_kd").as_double();
    yaw_tolerance_rad_= this->get_parameter("yaw_tolerance_rad").as_double();
    max_omega_p_      = this->get_parameter("max_omega_p").as_double();
    max_omega_n_      = this->get_parameter("max_omega_n").as_double();
    sample_time_      = this->get_parameter("sample_time_ms").as_int();

    yaw_pid_.setTunings(
        static_cast<float>(yaw_kp_),
        static_cast<float>(yaw_ki_),
        static_cast<float>(yaw_kd_));
    yaw_pid_.setOutputLimits(
        static_cast<float>(max_omega_n_),
        static_cast<float>(max_omega_p_));
    yaw_pid_.setSampleTime(static_cast<uint16_t>(sample_time_));
    yaw_pid_.setMode(AUTOMATIC);
    yaw_pid_.init();

    imu_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/imu_data_received", qos_profile,
        std::bind(&RotateServer::imuCallback, this, std::placeholders::_1));

    cmd_yaw_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/rotate_server/cmd_vel", qos_profile);

    action_server_ = rclcpp_action::create_server<RotateExec>(
        this,
        "rotate_exec",
        std::bind(&RotateServer::handleGoal,     this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&RotateServer::handleCancel,   this, std::placeholders::_1),
        std::bind(&RotateServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "[RotateServer] Ready  kp=%.3f ki=%.3f kd=%.3f tol=%.4f rad",
                yaw_kp_, yaw_ki_, yaw_kd_, yaw_tolerance_rad_);
}

void RotateServer::imuCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    imu_yaw_      = normalizeAngle(msg->data);
    imu_received_ = true;
}

float RotateServer::normalizeAngle(float angle)
{
    while (angle >  M_PI) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -M_PI) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

rclcpp_action::GoalResponse RotateServer::handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const RotateExec::Goal> goal)
{
    RCLCPP_INFO(get_logger(),
                "[RotateServer] Goal received  target_angle=%.4f rad",
                goal->target_angle);

    if (!imu_received_) {
        RCLCPP_ERROR(get_logger(), "[RotateServer] No IMU data yet REJECT");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RotateServer::handleCancel(
    const std::shared_ptr<GoalHandleRotate> /*goal_handle*/)
{
    RCLCPP_WARN(get_logger(), "[RotateServer] Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RotateServer::handleAccepted(
    const std::shared_ptr<GoalHandleRotate> goal_handle)
{
    std::thread{
        std::bind(&RotateServer::execute, this, std::placeholders::_1),
        goal_handle
    }.detach();
}
void RotateServer::execute(
    const std::shared_ptr<GoalHandleRotate> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    auto prev_time = std::chrono::steady_clock::now();
    float imu_prev = imu_yaw_;
    float setpoint = goal->target_angle;
    float rotated_so_far = 0.0f;

    RCLCPP_INFO(get_logger(),
                "[RotateServer] Executing  imu_prev=%.4f  setpoint=%.4f rad",
                imu_prev, setpoint);

    yaw_pid_.setpoint = setpoint;
    yaw_pid_.input    = 0.0f;
    yaw_pid_.init();

    auto feedback = std::make_shared<RotateExec::Feedback>();
    auto result   = std::make_shared<RotateExec::Result>();

    rclcpp::Rate rate(1000 / sample_time_);

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.angular.z = 0.0f;
            cmd_yaw_pub_->publish(stop_cmd);

            result->success   = false;
            result->final_yaw = imu_yaw_;
            goal_handle->canceled(result);
            RCLCPP_WARN(get_logger(), "[RotateServer] Goal cancelled");
            return;
        }
        auto present_time = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(present_time - prev_time).count();

        if (elapsed_ms < sample_time_)
        {
            continue;
        }
     
        float del_imu = 0.0f;

        if ((imu_prev < 0.0f) != (imu_yaw_ < 0.0f))  
        {
            float e = imu_yaw_ - imu_prev;
            if (std::fabs(e) > M_PI)  
                del_imu = (2.0f * static_cast<float>(M_PI)) - std::fabs(e);
                if (e > 0)
                    del_imu = -del_imu;
            else
                del_imu = e;
        }
        else
        {
            del_imu = imu_yaw_ - imu_prev;
        }
        rotated_so_far  += del_imu;
      
        yaw_pid_.input = rotated_so_far;
        yaw_pid_.compute();

        float error = setpoint - rotated_so_far;

        float omega = yaw_pid_.output;
        if (std::fabs(error) < 0.1f)
            omega *= (std::fabs(error) / 0.1f);

        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = omega;
        cmd_yaw_pub_->publish(cmd);

        feedback->current_yaw_error = error;
        feedback->current_yaw       = imu_yaw_;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(get_logger(),
            "[RotateServer] rotated=%.4f  error=%.4f  omega=%.4f  change=%.4f  setpoint=%.4f rad",
            rotated_so_far, error, omega, del_imu, setpoint);

        if (std::fabs(error) <= static_cast<float>(yaw_tolerance_rad_))
        {
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.angular.z = 0.0f;
            cmd_yaw_pub_->publish(stop_cmd);

            result->success   = true;
            result->final_yaw = imu_yaw_;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(),
                        "[RotateServer] Succeeded  final_yaw=%.4f rad",
                        imu_yaw_);
            return;
        }
        prev_time = present_time; 
        imu_prev = imu_yaw_;  

        // rate.sleep();
    }

    result->success   = false;
    result->final_yaw = imu_yaw_;
    goal_handle->abort(result);
}

// void RotateServer::execute(
//     const std::shared_ptr<GoalHandleRotate> goal_handle)
// {
//     const auto goal = goal_handle->get_goal();

//     float setpoint = normalizeAngle(imu_yaw + goal->target_angle);
    

//     RCLCPP_INFO(get_logger(),
//                 "[RotateServer] Executing  imu_yaw=%.4f  setpoint=%.4f rad",
//                 imu_yaw_, setpoint);

//     yaw_pid_.setpoint = setpoint;
//     yaw_pid_.input    = imu_yaw_;
//     yaw_pid_.init();

//     auto feedback = std::make_shared<RotateExec::Feedback>();
//     auto result   = std::make_shared<RotateExec::Result>();

//     rclcpp::Rate rate(1000 / sample_time_);

//     while (rclcpp::ok())
//     {
//         if (goal_handle->is_canceling())
//         {
//             geometry_msgs::msg::Twist stop_cmd;
//             stop_cmd.angular.z = 0.0f;
//             cmd_yaw_pub_->publish(stop_cmd);

//             result->success   = false;
//             result->final_yaw = imu_yaw_;
//             goal_handle->canceled(result);
//             RCLCPP_WARN(get_logger(), "[RotateServer] Goal cancelled");
//             return;
//         }

//         yaw_pid_.input = imu_yaw_;
//         yaw_pid_.compute();

//         float error = normalizeAngle(setpoint - imu_yaw_);

//         float omega = yaw_pid_.output;
//         if (std::fabs(error) < 0.1f)
//             omega *= (std::fabs(error) / 0.1f);

//         geometry_msgs::msg::Twist cmd;
//         cmd.angular.z = omega;
//         cmd_yaw_pub_->publish(cmd);

//         feedback->current_yaw_error = error;
//         feedback->current_yaw       = imu_yaw_;
//         goal_handle->publish_feedback(feedback);

//         // RCLCPP_INFO(get_logger(),
//         //             // "[RotateServer] error=%.4f  omega=%.4f",
//         //             error, omega);

//         if (std::fabs(error) <= static_cast<float>(yaw_tolerance_rad_))
//         {
//             geometry_msgs::msg::Twist stop_cmd;
//             stop_cmd.angular.z = 0.0f;
//             cmd_yaw_pub_->publish(stop_cmd);

//             result->success   = true;
//             result->final_yaw = imu_yaw_;
//             goal_handle->succeed(result);
//             RCLCPP_INFO(get_logger(),
//                         "[RotateServer] Succeeded  final_yaw=%.4f rad",
//                         imu_yaw_);
//             return;
//         }

//         rate.sleep();
//     }

//     result->success   = false;
//     result->final_yaw = imu_yaw_;
//     goal_handle->abort(result);
// }

} 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotateExecServer::RotateServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}