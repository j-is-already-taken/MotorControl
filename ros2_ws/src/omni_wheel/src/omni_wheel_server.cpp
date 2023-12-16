#include "omni_wheel/omni_wheel_server.hpp"

#include <chrono>


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
    //rclcpp::Time time = rclcpp::Time::now();
    int32_t duty_ratio = 255;
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OmniWheel::Feedback>();
    auto & millimeter = feedback->partial_millimeter;
    auto result = std::make_shared<OmniWheel::Result>();
    std::array<double, 3> motor_ratio;
    if(goal->is_turning){
      RotationalDirection rotation_direction = RotationalDirection::CW;
      if(goal->target_angle >= 0)
      {
        rotation_direction = RotationalDirection::CCW;
      }
      auto [motor1_ratio, motor2_ratio, motor3_ratio] = omni_wheel_control.turnRobot(rotation_direction, duty_ratio);
      motor_ratio.at(0) = motor1_ratio;
      motor_ratio.at(1) = motor2_ratio;
      motor_ratio.at(2) = motor3_ratio;
    }else{
      auto [motor1_ratio, motor2_ratio, motor3_ratio] = omni_wheel_control.moveRobot(Angle(AngleType::Degree, goal->target_angle), duty_ratio);
      motor_ratio.at(0) = motor1_ratio;
      motor_ratio.at(1) = motor2_ratio;
      motor_ratio.at(2) = motor3_ratio;
    }

    //std::array<double, 3> motor_ratio{motor1_ratio, motor2_ratio, motor3_ratio};

    std::vector<MotorCommand> motor_command;
    for(const auto &tmp_motor_ratio: motor_ratio)
    {
      std::cout << tmp_motor_ratio << std::endl;
      RotationDirection rotation_direction = RotationDirection::CW;
      if(tmp_motor_ratio < 0)
      {
        rotation_direction = RotationDirection::CCW;
      }
      motor_command.emplace_back(MotorCommand{static_cast<uint32_t>(std::abs(tmp_motor_ratio)) , rotation_direction});
    }


    //motor_control_.setDutyCycle({motor1_ratio * duty_ratio, motor2_ratio * duty_ratio, motor3_ratio * duty_ratio});

    motor_control_.moveMotor(motor_command);
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
	motor_control_.stopMotor();
        return;
      }
      // Update millimeter
      auto elpasd = std::chrono::high_resolution_clock::now();
      actual_move_millimeter = std::chrono::duration_cast<std::chrono::milliseconds>(elpasd - start).count() / 1000.;//todo: オドメトリなどから計算する
      millimeter = actual_move_millimeter;

      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //auto start = std::chrono::high_resolution_clock::now();
      loop_rate.sleep();
    }

    motor_control_.stopMotor();
    // Check if goal is done
    if (rclcpp::ok()) {
      result->millimeter = millimeter;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(omni_wheel::OmniWheelActionServer)

