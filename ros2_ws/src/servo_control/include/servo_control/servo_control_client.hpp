#ifndef _SERVO_CONTROL_CLIENT_
#define _SERVO_CONTROL_CLIENT_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>


#include "servo_control_interfaces/action/servo_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace servo_control_client
{
  class ServoControlActionClient : public rclcpp::Node
  {
    public:
      using ServoControl = servo_control_interfaces::action::ServoControl;
      using GoalHandleServo = rclcpp_action::ClientGoalHandle<ServoControl>;

      explicit ServoControlActionClient(const rclcpp::NodeOptions & options);

      void send_goal();

    private:
      rclcpp_action::Client<ServoControl>::SharedPtr servo_control_client_ptr_;
      //rclcpp::TimerBase::SharedPtr timer_;

      void goal_response_callback(const GoalHandleServo::SharedPtr & goal_handle);

      void feedback_callback(
          GoalHandleServo::SharedPtr,
          const std::shared_ptr<const ServoControl::Feedback> feedback);

      void result_callback(const GoalHandleServo::WrappedResult & result);
  };  // class ServoControlActionClient

}  // namespace action_tutorials_cpp

#endif
