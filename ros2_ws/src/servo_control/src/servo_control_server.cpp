#include "servo_control/servo_control_server.hpp"

#include <chrono>


namespace servo_control
{

  ServoControlActionServer::ServoControlActionServer(const rclcpp::NodeOptions & options)
    : Node("servo_control_action_server", options)
  {
    //using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ServoControl>(
        this,
        "omni_wheel",
        std::bind(&ServoControlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ServoControlActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ServoControlActionServer::handle_accepted, this, std::placeholders::_1));
  }


  rclcpp_action::GoalResponse ServoControlActionServer::handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ServoControl::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received servo move %s", (goal->is_move ? "true" : "false"));
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ServoControlActionServer::handle_cancel(
      const std::shared_ptr<GoalHandleServo> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ServoControlActionServer::handle_accepted(const std::shared_ptr<GoalHandleServo> goal_handle)
  {
    //using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ServoControlActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void ServoControlActionServer::execute(const std::shared_ptr<GoalHandleServo> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(50);
    //rclcpp::Time time = rclcpp::Time::now();
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ServoControl::Feedback>();
    //auto & millimeter = feedback->partial_millimeter;
    auto result = std::make_shared<ServoControl::Result>();


    //for (int i = 1; (i < goal->move_millimeter) && rclcpp::ok(); ++i) {
    bool success = false;
    if(goal->is_move){
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) 
      {
        result->success = success;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update millimeter

      // Publish feedback
      //goal_handle->publish_feedback(feedback);
      //RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //auto start = std::chrono::high_resolution_clock::now();
      loop_rate.sleep();
    }
    success = true;

    //motor_control_.stopMotor();
    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = success;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(servo_control::ServoControlActionServer)

