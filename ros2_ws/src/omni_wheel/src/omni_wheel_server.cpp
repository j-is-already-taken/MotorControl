#include "omni_wheel/omni_wheel_server.hpp"


namespace omni_wheel
{

  OmniWheelActionServer::OmniWheelActionServer(const rclcpp::NodeOptions & options)
    : Node("omni_wheel_action_server", options)
  {
    //using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<OmniWheel>(
        this,
        "omni_wheel",
        std::bind(&OmniWheelActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&OmniWheelActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&OmniWheelActionServer::handle_accepted, this, std::placeholders::_1));
  }


  rclcpp_action::GoalResponse OmniWheelActionServer::handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const OmniWheel::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d %s", goal->target_angle, (goal->is_turning ? "true" : "false"));
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse OmniWheelActionServer::handle_cancel(
      const std::shared_ptr<GoalHandleOmniWheel> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void OmniWheelActionServer::handle_accepted(const std::shared_ptr<GoalHandleOmniWheel> goal_handle)
  {
    //using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&OmniWheelActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void OmniWheelActionServer::execute(const std::shared_ptr<GoalHandleOmniWheel> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(50);
    rclcpp::Time time = rclcpp::Time::now();
    int32_t duty_ratio = 255;
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OmniWheel::Feedback>();
    auto & millimeter = feedback->partial_millimeter;
    auto result = std::make_shared<OmniWheel::Result>();
    auto [motor1_ratio, motor2_ratio, motor3_ratio] = omni_wheel_control.moveRobot(Angle(AngleType::Degree, goal->target_angle));

    motor_control.setDutyCycle({motor1_ratio * duty_ratio, motor2_ratio * duty_ratio, moto3_ratio * duty_ratio});
    auto start = std::chrono::high_resolution_clock::now();
    //for (int i = 1; (i < goal->move_millimeter) && rclcpp::ok(); ++i) {
    double actual_move_millimeter = 0.0;
    while(goal->move_millimeter > actual_move_millimeter)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) 
      {
        result->millimeter = millimeter;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update millimeter
      auto elpasd = std::chrono::high_resolution_clock::now();
      actual_move_millimeter = std::duration_cast<std::chrono::milliseconds>(elpasd - start).count() / 1000.;//todo: オドメトリなどから計算する
      millimeter = actual_move_millimeter;

      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

    auto start = std::chrono::high_resolution_clock::now();
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->millimeter = millimeter;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(omni_wheel::OmniWheelActionServer)

