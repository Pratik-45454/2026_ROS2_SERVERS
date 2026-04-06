#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

static constexpr double SMALL_VX   = 0.05;
static constexpr double SMALL_ROLL = 0.1;

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode() : Node("robot_state_machine")
  {
    auto qos = rclcpp::QoS(10).best_effort();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos);

    sick_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/sick_data", qos,
      std::bind(&StateMachineNode::sickCallback, this, std::placeholders::_1));

    prox_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/proximity_data", qos,
      std::bind(&StateMachineNode::proxCallback, this, std::placeholders::_1));

    state_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "/set_state",
      std::bind(&StateMachineNode::setStateCallback,
                this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      10ms, std::bind(&StateMachineNode::run, this));

    sick_data_ = {0, 0, 0, 0};
    // sick_thresholds_ = {200, 200, 200, 200};

    ak_up_client_ = this->create_client<std_srvs::srv::SetBool>(
      "/ak_motor_up_execute");

    ak_down_client_ = this->create_client<std_srvs::srv::SetBool>(
      "/ak_motor_down_exec");


    RCLCPP_INFO(this->get_logger(), "State machine node started");
  }

private:

  void sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() == 4)
      sick_data_ = msg->data;
  }

  void proxCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (!msg->data.empty())
      proximity_ = msg->data;
  }

  void setStateCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (req->data)
        state_ = State::SCAN;
    else
        state_ = State::IDLE;
    halted_ = false;

    RCLCPP_INFO(this->get_logger(),
                "State changed → %d", state_);

    res->success = true;
  }


  geometry_msgs::msg::Twist idle()
  {
    return geometry_msgs::msg::Twist();
    RCLCPP_INFO(this->get_logger(),
                "State is idle");
  }

  geometry_msgs::msg::Twist activeScan()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = SMALL_VX;
    cmd.angular.x = 0.0;
  
    if (sick_data_.empty())
      return cmd;
  
    float front_sick = sick_data_[0];
  
    if (front_sick < stair_threshold_)
    {
      cmd.linear.x = 0.0;   // stop forward
      cmd.angular.x = SMALL_ROLL;
    }
  
    return cmd;
  }

  void halt()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist()); //publish nothing when crash occurs
  }


  bool callAkUp()
    {
      if (!ak_up_client_->wait_for_service(1s))
      {
        RCLCPP_ERROR(this->get_logger(), "AK UP service not available");
        return false;
      }

      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      req->data = true;

      auto future = ak_up_client_->async_send_request(req);

      if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        return false;
      }

      return future.get()->success;
    }

  bool callAkDown()
    {
      if (!ak_down_client_->wait_for_service(1s))
      {
        RCLCPP_ERROR(this->get_logger(), "AK DOWN service not available");
        return false;
      }

      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      req->data = true;

      auto future = ak_down_client_->async_send_request(req);

      if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        return false;
      }

      return future.get()->success;
    }

  void run()
  {
    if (halted_)
    {
      halt();
      return;
    }

    geometry_msgs::msg::Twist cmd;

    switch (state_)
    {
      case IDLE:
        cmd = idle();
        break;

      case SCAN:
        cmd = activeScan();
        if (sick_data_[0] < stair_threshold_)
        {
          halt();
          state_ = AK_UP;
          return;
        }
        break;

      case AK_UP:
        halt();
        if (callAkUp())
          state_ = ROLL;
        else
          halted_ = true;
        return;

      case ROLL:
        cmd.angular.x = SMALL_ROLL;
        if (proximity_ == 0x09):
        {
          halt();
          state_ = AK_DOWN;
          return;
        }
        break;

      case AK_DOWN:
        halt();
        if (callAkDown())
          state_ = SCAN_NEXT;
        else
          halted_ = true;
        return;

      case SCAN_NEXT:
        cmd = activeScan();
        break;

      default:
        halted_ = true;
        return;
    }


    cmd_pub_->publish(cmd); //lastma publish
  }


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sick_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr prox_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr state_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ak_up_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ak_down_client_;


  std::vector<float> sick_data_;
//   std::vector<float> sick_thresholds_;
  float stair_threshold_ = 150.0f;

  enum State
  {
    IDLE = 0,
    SCAN = 1,
    AK_UP = 2,
    ROLL = 3,
    AK_DOWN = 4,
    SCAN_NEXT = 5
  };

  uint8_t proximity_{0};

  State state_{IDLE};
  bool halted_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
