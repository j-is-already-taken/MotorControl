#ifndef OMNI_WHEEL_H_
#define OMNI_WHEEL_H_

#include <functional>
#include <memory>
#include <thread>

#include "omni_wheel_interfaces/action/omni_wheel_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "omni_wheel_control.hpp"
#include "motor_control.hpp"


namespace omni_wheel
{
  class OmniWheelActionServer : public rclcpp::Node
  {
    public:
      using OmniWheel = omni_wheel_interfaces::action::OmniWheelControl;
      using GoalHandleOmniWheel = rclcpp_action::ServerGoalHandle<OmniWheel>;

      explicit OmniWheelActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    private:
      rclcpp_action::Server<OmniWheel>::SharedPtr action_server_;

      rclcpp_action::GoalResponse handle_goal(
          const rclcpp_action::GoalUUID & uuid,
          std::shared_ptr<const OmniWheel::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
          const std::shared_ptr<GoalHandleOmniWheel> goal_handle);

      void handle_accepted(const std::shared_ptr<GoalHandleOmniWheel> goal_handle);

      void execute(const std::shared_ptr<GoalHandleOmniWheel> goal_handle);

      OmniWheelControl omni_wheel_control;
      MotorControl motor_control_;
  };  // class OmniWheelActionServer

}  // namespace omni_wheel


#endif //OMNI_WHEEL_H_
