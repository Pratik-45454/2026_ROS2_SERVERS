#include "servers/move_server.hpp"

#include <cmath>
#include <cstdio>
#include <thread>

static constexpr float ENCODER_X_SIGN = -1.0f;
static constexpr float ENCODER_Y_SIGN = -1.0f;

MoveServer::MoveServer(const rclcpp::NodeOptions & options)
: Node("move_server", options)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    this->declare_parameter("x_kp", 0.009); //0.009:forest good
    this->declare_parameter("x_ki", 0.0); //0.0
    this->declare_parameter("x_kd", 0.003); //0.003: forest good
    this->declare_parameter("y_kp", 0.004); //0.004: forest good
    this->declare_parameter("y_ki", 0.0); //0.0
    this->declare_parameter("y_kd", 0.002); //0.002: forest good
    
    this->declare_parameter("dist_tol",15.0);
    this->declare_parameter("max_vel",3.0);
    this->declare_parameter("sample_time",10);

    x_kp = this->get_parameter("x_kp").as_double();
    x_ki = this->get_parameter("x_ki").as_double();
    x_kd = this->get_parameter("x_kd").as_double();
    y_kp = this->get_parameter("y_kp").as_double();
    y_ki = this->get_parameter("y_ki").as_double();
    y_kd = this->get_parameter("y_kd").as_double();

    dist_tol_ = this->get_parameter("dist_tol").as_double();
    max_vel_ = this->get_parameter("max_vel").as_double();
    sample_time_ = this->get_parameter("sample_time").as_int();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/move_server/cmd_vel", qos_profile );

    odom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/odom", qos_profile,
        std::bind(&MoveServer::odomCallback, this, std::placeholders::_1));

    prox_front_sub_ = this->create_subscription<std_msgs::msg::Byte>(
        "/prox_front_received", qos_profile,
        std::bind(&MoveServer::proxFrontCallback, this, std::placeholders::_1));

    x_pid = PID(x_kp, x_ki, x_kd, -max_vel_, max_vel_, sample_time_);
    y_pid = PID(y_kp, y_ki, y_kd, -max_vel_, max_vel_, sample_time_);

    action_server_ = rclcpp_action::create_server<MoveTwist>(
        this,
        "move_twist_",
        std::bind(&MoveServer::handleGoal,     this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveServer::handleCancel,   this, std::placeholders::_1),
        std::bind(&MoveServer::handleAccepted, this, std::placeholders::_1));

    is_action_finished = false;
    is_movex_done = false;
    is_movey_done = false;
    RCLCPP_INFO(this->get_logger(), "[MoveServer] Ready.");
    // auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // auto file_sink    = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/move_pid.log", true);
    // pid_logger_ = std::make_shared<spdlog::logger>("move_pid", spdlog::sinks_init_list{console_sink, file_sink});
    // pid_logger_->set_level(spdlog::level::info);
}

rclcpp_action::GoalResponse MoveServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveTwist::Goal> goal)
{
    (void)uuid;

    odom_dist_x_start = odom_dist_x_;
    odom_dist_y_start = odom_dist_y_;

   
    x_pid.setTunings(goal->x_kp, goal->x_ki, goal->x_kd);  
    y_pid.setTunings(goal->y_kp, goal->y_ki, goal->y_kd);  

    x_pid.setOutputLimits(-max_vel_,max_vel_);
    y_pid.setOutputLimits(-max_vel_,max_vel_);
    x_pid.setMode(AUTOMATIC);
    y_pid.setMode(AUTOMATIC);

    x_pid.setpoint = goal->move_x;
    y_pid.setpoint = goal->move_y;

    is_action_finished = false;
    is_movex_done = false;
    is_movey_done = false;

    RCLCPP_INFO(this->get_logger(),"[MoveServer] Goal received : move_x=%.1f mm  move_y=%.1f mm",
                goal->move_x, goal->move_y);
   
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveServer::handleCancel(
    const std::shared_ptr<GoalHandleMove> /*goal_handle*/)
{
    RCLCPP_WARN(this->get_logger(), "[MoveServer] Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveServer::handleAccepted(const std::shared_ptr<GoalHandleMove> goal_handle)
{
    using namespace std::placeholders;
    std::thread{
        std::bind(&MoveServer::execute, this, _1),
        goal_handle,
    }
        .detach();
}
void MoveServer::execute(const std::shared_ptr<GoalHandleMove> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[MoveServer] Executing");

    auto feedback = std::make_shared<MoveTwist::Feedback>();
    auto result   = std::make_shared<MoveTwist::Result>();
   
    if(!odom_fresh_)
    {
        RCLCPP_INFO(this->get_logger(), "[MoveServer] odom not available");
        result->success          = false;
        result->final_avg_dist_m = 0.0;
        goal_handle->abort(result);
    }

    while (!is_action_finished)
    {
        double vx = 0.0f;
        double vy = 0.0f;

       
        x_pid.input = (odom_dist_x_ - odom_dist_x_start) * 1000;
        x_pid.compute();
        vx = x_pid.output;

        if (fabs(x_pid.setpoint - x_pid.input) < dist_tol_)
        {
            vx = 0.0;
            is_movex_done = true;
            printf("x true\n");
        }
        else is_movex_done = false;

        
        y_pid.input = (odom_dist_y_ - odom_dist_y_start) * 1000;
        y_pid.compute();
        vy = y_pid.output;

        if (fabs(y_pid.setpoint - y_pid.input) < dist_tol_)
        {
            vy = 0.0;
            is_movey_done = true;
            printf("y true\n");
        }
        else is_movey_done = false;

        RCLCPP_INFO(this->get_logger(),
            "[MoveServer] X_SETPOINT=%.1f mm  X_INPUT=%.1f mm  Y_SETPOINT=%.1f mm  Y_INPUT=%.1f mm",
            x_pid.setpoint, x_pid.input, y_pid.setpoint, y_pid.input);

        publishVel(vx, vy);

        if (is_movex_done && is_movey_done)
        {
            is_action_finished = true;
            publishVel(0.0f, 0.0f);
            RCLCPP_INFO(this->get_logger(), "[MoveServer] Completed");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sample_time_)));
    }

    if (goal_handle->is_active())
    {
        result->success = true;
        result->final_avg_dist_m = 0.0;
        goal_handle->succeed(result);
    }
}


    


// void MoveServer::execute(const std::shared_ptr<GoalHandleMove> goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "[MoveServer] Executing");

//     auto feedback = std::make_shared<MoveTwist::Feedback>();
//     auto result   = std::make_shared<MoveTwist::Result>();
   
//     if(!odom_fresh_)
//     {
//         RCLCPP_INFO(this->get_logger(), "[MoveServer] odom not available");
//         result->success          = false;
//         result->final_avg_dist_m = 0.0;
//         goal_handle->abort(result);
//     }

//     while (!is_action_finished)
//     {
//         float vx = 0.0f;
//         float vy = 0.0f;
  
//         x_pid.input = (odom_dist_x_ - odom_dist_x_start)*1000;
//         y_pid.input = (odom_dist_y_ - odom_dist_y_start)*1000;

//         x_pid.compute();
//         y_pid.compute();

//         vx = x_pid.output;
//         vy = y_pid.output;

//         RCLCPP_INFO(this->get_logger(),"[MoveServer] Goal received : X_SETPOINT=%.1f mm  X_INPUT=%.1f mm Y_SETPOINT=%.1f mm  Y_INPUT=%.1f mm ",
//                 x_pid.setpoint, x_pid.input, y_pid.setpoint, y_pid.input);
        
//         if (fabs(x_pid.setpoint - x_pid.input) < dist_tol_)
//         {   
//             vx = 0.0;
//             is_movex_done = true;
//             printf("x true\n");
//         }
//         else is_movex_done = false;
//         if (fabs(y_pid.setpoint - y_pid.input) < dist_tol_)
//         {
//             vy = 0.0;
//             is_movey_done = true;  
//             printf("y true\n");

//         }
//         else is_movey_done = false;  
//         publishVel(vx, vy);

//         if (is_movex_done && is_movey_done)
//         { 
//             is_action_finished = true;
//             publishVel(0.0f, 0.0f);
//             RCLCPP_INFO(this->get_logger(), "[MoveServer] Completed");

//         }
//     }

//     result->success = true;
//     result->final_avg_dist_m = 0.0;
//     goal_handle->succeed(result);
// }

void MoveServer::odomCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(this->get_logger(),
                    "[MoveServer] Odom data size < 2: %zu", msg->data.size());
        return;
    }
    odom_dist_x_ = msg->data[0];
    odom_dist_y_ = msg->data[1];
    odom_fresh_  = true;
}

void MoveServer::proxFrontCallback(const std_msgs::msg::Byte::SharedPtr msg)
{
    if  (msg->data == 1)
    {
        prox_front_triggered_ = true;
    }
}

void MoveServer::publishVel(float vx, float vy)
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
    auto node = std::make_shared<MoveServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

