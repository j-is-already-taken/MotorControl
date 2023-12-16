#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#include <functional>
#include <memory>
#include <thread>

#include "servo_control_interfaces/action/omni_wheel_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "servo_control.hpp"



namespace servo_control
{
  class ServoControlActionServer : public rclcpp::Node
  {
    public:
      using ServoControl = servo_control_interfaces::action::ServoControl;
      using GoalHandleServo = rclcpp_action::ServerGoalHandle<ServoControl>;

      explicit ServoControlActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    private:
      rclcpp_action::Server<ServoControl>::SharedPtr action_server_;

      rclcpp_action::GoalResponse handle_goal(
          const rclcpp_action::GoalUUID & uuid,
          std::shared_ptr<const ServoControl::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
          const std::shared_ptr<GoalHandleServoControl> goal_handle);

      void handle_accepted(const std::shared_ptr<GoalHandleServo> goal_handle);

      void execute(const std::shared_ptr<GoalHandleServo> goal_handle);

      ServoControler servo_controler_;
      //OmniWheelControl omni_wheel_control;
      //MotorControl motor_control_;
  };  // class OmniWheelActionServer

}  // namespace omni_wheel


#endif //OMNI_WHEEL_H_
