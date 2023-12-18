#include <functional>
#include <future>
#include <memory>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/point.hpp"
#include "omni_wheel_interfaces/action/omni_wheel_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace omni_wheel
{
class OmniWheelActionClient : public rclcpp::Node
{
public:
  using OmniWheelControl = omni_wheel_interfaces::action::OmniWheelControl;
  using GoalHandleOmniWheel = rclcpp_action::ClientGoalHandle<OmniWheelControl>;

  explicit OmniWheelActionClient(const rclcpp::NodeOptions & options)
  : Node("omni_wheel_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<OmniWheelControl>(
      this, "omni_wheel");

    this->subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/center_coordinate", 5, std::bind(&OmniWheelActionClient::topic_callback, this, std::placeholders::_1));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&OmniWheelActionClient::timer_callback, this));
  }

  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    last_received_ = this->now();
    
    if (is_turning_) {
      this->client_ptr_->async_cancel_all_goals();
      is_turning_ = false;
	}

    double x = msg->x;
    double y = msg->y;

    // 画像中心からの相対座標
    double dx = x - 320;
    double dy = y - 240;

    // 角度の計算 (ラジアンから度に変換)
    double angle_rad = atan2(dy, dx);
    double angle_deg = angle_rad * (180.0 / M_PI);

    RCLCPP_INFO(this->get_logger(), "Object angle: %f degrees", angle_deg);
    send_goal(angle_deg, false);
   }

  void timer_callback()
  {
    if ((this->now() - last_received_).seconds() >= 0.5 && !is_turning_) {
      // If no message received for 0.5 seconds and not already turning
      send_goal(1, true);
      is_turning_ = true;
    }
  }

  void send_goal(double angle_deg, bool turning)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = OmniWheelControl::Goal();
    goal_msg.target_angle = angle_deg;
    goal_msg.move_millimeter = 5; // 仮の値
    goal_msg.is_turning = turning;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<OmniWheelControl>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&OmniWheelActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&OmniWheelActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&OmniWheelActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<OmniWheelControl>::SharedPtr client_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_received_;
  bool is_turning_;
  GoalHandleOmniWheel::SharedPtr current_goal_;

  void goal_response_callback(const GoalHandleOmniWheel::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleOmniWheel::SharedPtr,
    const std::shared_ptr<const OmniWheelControl::Feedback> feedback)
  {
    // Feedback処理 (必要に応じて実装)
  }

  void result_callback(const GoalHandleOmniWheel::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    // Result処理 (必要に応じて実装)
  }
};  // class OmniWheelActionClient

}  // namespace omni_wheel_client

RCLCPP_COMPONENTS_REGISTER_NODE(omni_wheel::OmniWheelActionClient)
