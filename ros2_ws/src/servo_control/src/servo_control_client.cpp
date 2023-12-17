#include "servo_control/servo_control_client.hpp"

namespace servo_control_client
{
  ServoControlActionClient::ServoControlActionClient(const rclcpp::NodeOptions & options)
    : Node("servo_control_client", options)
  {
    this->servo_control_client_ptr_ = rclcpp_action::create_client<ServoControl>(
        this,
        "servo_control");
    this->psd_sensor_sub_ =  this->create_subscription<std_msgs::msg::String>(
		    "/Serial_in", 1, std::bind(&ServoControlActionClient::psdSensorTopicCallback, this, std::placeholders::_1));

    /*
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ServoControlActionClient::send_goal, this));
    */
  }

  void ServoControlActionClient::psdSensorTopicCallback(const std_msgs::msg::String::SharedPtr msg) 
  {
    //using namespace std::placeholders;
	  std::string raw_data = msg->data;
	  std::string::size_type pos = raw_data.find("\n");
	  double psd_sensor_value = 0.0;
	  if(pos == std::string::npos){
		  psd_sensor_value = std::stod(raw_data);
	  }else{
		  std::string tmp_num = raw_data.substr(0, pos);
		  psd_sensor_value = std::stod(raw_data);
	  }
	  if(300 < psd_sensor_value && psd_sensor_value < 600){
		  send_goal();
	  }
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", psd_sensor_value);
  }

  void ServoControlActionClient::send_goal()
  {
    //using namespace std::placeholders;

    //this->timer_->cancel();

    if (!this->servo_control_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ServoControl::Goal();
    goal_msg.is_move = true;
    //goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<ServoControl>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ServoControlActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&ServoControlActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ServoControlActionClient::result_callback, this, std::placeholders::_1);
    this->servo_control_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }


  void ServoControlActionClient::goal_response_callback(const GoalHandleServo::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void ServoControlActionClient::feedback_callback(
      GoalHandleServo::SharedPtr,
      const std::shared_ptr<const ServoControl::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void ServoControlActionClient::result_callback(const GoalHandleServo::WrappedResult & result)
  {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
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
    std::stringstream ss;
    ss << "Result received: ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    //rclcpp::shutdown();
  }

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(servo_control_client::ServoControlActionClient)
