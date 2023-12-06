#include "omni_wheel/motor_control.hpp"

template <class... Args>
bool isCheckError(int ret, std::string error_msg, Args... args)
{
  std::size_t function_parameter_pack_size = sizeof...(args);
  int error_state[] = { args... };
  //for(const auto &error_state : args)
  for(std::size_t i=0; i < function_parameter_pack_size;i++)
  {
    if(ret == error_state[i])
    {
      std::cerr << error_msg << " " << ret << std::endl;
      return true;
    }
  }
  return false;
}

MotorControl::MotorControl(): pi_state_(-1), frequency_(50), dutycycle_(255)
{
  /*
  std::vector<MotorUsePin> each_pin{MotorUsePin(12, 7, 8, 10, 25), MotorUsePin(13, 5, 6, 9, 11), MotorUsePin(16, 13, 19, 20, 21)};
  for(std::size_t i=0;i<motor_use_num_;i++)
  {
    motor_use_pin_.at(i).pwm_pin = each_pin.at(i).pwm_pin;

    motor_use_pin_.at(i).rotate_control_pin1 = each_pin.at(i).rotate_control_pin1;
    motor_use_pin_.at(i).rotate_control_pin2 = each_pin.at(i).rotate_control_pin2;

    motor_use_pin_.at(i).encorder_input_pin1 = each_pin.at(i).encorder_input_pin1;
    motor_use_pin_.at(i).encorder_input_pin2 = each_pin.at(i).encorder_input_pin2;
  }
  */
  pi_state_ = pigpio_start(nullptr, nullptr);
  if(pi_state_ < 0){
    std::cerr << "not started. check running pigpiod daemon" << std::endl;
    return;
  }
  initSetup();
}
int MotorControl::initSetup()
{
  std::vector<MotorUsePin> each_pin{MotorUsePin(12, 7, 8, 10, 25), MotorUsePin(13, 5, 6, 9, 11), MotorUsePin(16, 13, 19, 20, 21)};
  for(std::size_t i=0;i<motor_use_num_;i++)
  {
    motor_use_pin_.at(i).pwm_pin = each_pin.at(i).pwm_pin;

    motor_use_pin_.at(i).rotate_control_pin1 = each_pin.at(i).rotate_control_pin1;
    motor_use_pin_.at(i).rotate_control_pin2 = each_pin.at(i).rotate_control_pin2;

    motor_use_pin_.at(i).encorder_input_pin1 = each_pin.at(i).encorder_input_pin1;
    motor_use_pin_.at(i).encorder_input_pin2 = each_pin.at(i).encorder_input_pin2;
  }
  pi_state_ = pigpio_start(nullptr, nullptr);
  if(isCheckError(pi_state_, "not started. check running pigpiod daemon", 0)) return -1;
  
  for(std::size_t i=0;i<motor_use_num_;i++)
  {
    if(isCheckError(set_mode(pi_state_, motor_use_pin_.at(i).pwm_pin, PI_OUTPUT), "GPIO set error", PI_BAD_GPIO, PI_BAD_MODE, PI_NOT_PERMITTED)) return -1;

    if(isCheckError(set_mode(pi_state_, motor_use_pin_.at(i).rotate_control_pin1, PI_OUTPUT), "GPIO set error", PI_BAD_GPIO, PI_BAD_MODE, PI_NOT_PERMITTED)) return -1;
    if(isCheckError(set_mode(pi_state_, motor_use_pin_.at(i).rotate_control_pin2, PI_OUTPUT), "GPIO set error", PI_BAD_GPIO, PI_BAD_MODE, PI_NOT_PERMITTED)) return -1;

    if(isCheckError(gpio_write(pi_state_, motor_use_pin_.at(i).rotate_control_pin1, PI_LOW), "GPIO Write error", PI_BAD_GPIO, PI_BAD_LEVEL, PI_NOT_PERMITTED)) return -1;
    if(isCheckError(gpio_write(pi_state_, motor_use_pin_.at(i).rotate_control_pin2, PI_LOW), "GPIO Write error", PI_BAD_GPIO, PI_BAD_LEVEL, PI_NOT_PERMITTED)) return -1;


    if(isCheckError(set_mode(pi_state_, motor_use_pin_.at(i).encorder_input_pin1, PI_INPUT), "GPIO set error", PI_BAD_GPIO, PI_BAD_MODE, PI_NOT_PERMITTED)) return -1;
    if(isCheckError(set_mode(pi_state_, motor_use_pin_.at(i).encorder_input_pin2, PI_INPUT), "GPIO set error", PI_BAD_GPIO, PI_BAD_MODE, PI_NOT_PERMITTED)) return -1;

    if(isCheckError(set_pull_up_down(pi_state_, motor_use_pin_.at(i).encorder_input_pin1, PI_PUD_DOWN), "set pull down error", PI_BAD_GPIO, PI_BAD_PUD, PI_NOT_PERMITTED)) return -1;
    if(isCheckError(set_pull_up_down(pi_state_, motor_use_pin_.at(i).encorder_input_pin2, PI_PUD_DOWN), "set pull down error", PI_BAD_GPIO, PI_BAD_PUD, PI_NOT_PERMITTED)) return -1;

    if(isCheckError(set_PWM_dutycycle(pi_state_, motor_use_pin_.at(i).pwm_pin, dutycycle_), "set pwm dutycycle error ", PI_BAD_USER_GPIO, PI_BAD_DUTYCYCLE, PI_NOT_PERMITTED)) return -1;
    if(isCheckError(set_PWM_frequency(pi_state_, motor_use_pin_.at(i).pwm_pin, frequency_), "set pwm frequency error ", PI_BAD_USER_GPIO, PI_NOT_PERMITTED)) return -1;

  }

  if(isCheckError(callback(pi_state_, motor_use_pin_.at(0).encorder_input_pin1, FALLING_EDGE, motor1CallBack), "call back set error ", pigif_bad_malloc, pigif_duplicate_callback, pigif_bad_callback)) return -1;
  if(isCheckError(callback(pi_state_, motor_use_pin_.at(0).encorder_input_pin2, FALLING_EDGE, motor1CallBack), "call back set error ", pigif_bad_malloc, pigif_duplicate_callback, pigif_bad_callback)) return -1;

  if(isCheckError(callback(pi_state_, motor_use_pin_.at(1).encorder_input_pin1, FALLING_EDGE, motor2CallBack), "call back set error ", pigif_bad_malloc, pigif_duplicate_callback, pigif_bad_callback)) return -1;
  if(isCheckError(callback(pi_state_, motor_use_pin_.at(1).encorder_input_pin2, FALLING_EDGE, motor2CallBack), "call back set error ", pigif_bad_malloc, pigif_duplicate_callback, pigif_bad_callback)) return -1;

  if(isCheckError(callback(pi_state_, motor_use_pin_.at(2).encorder_input_pin1, FALLING_EDGE, motor3CallBack), "call back set error ", pigif_bad_malloc, pigif_duplicate_callback, pigif_bad_callback)) return -1;
  if(isCheckError(callback(pi_state_, motor_use_pin_.at(2).encorder_input_pin2, FALLING_EDGE, motor3CallBack), "call back set error ", pigif_bad_malloc, pigif_duplicate_callback, pigif_bad_callback)) return -1;



  return 0;
}

void MotorControl::motor1CallBack(int pi, uint32_t user_gpio, uint32_t level, uint32_t tick)
{
  if(user_gpio == motor_use_pin_.at(0).encorder_input_pin1){
  }else if(user_gpio == motor_use_pin_.at(0).encorder_input_pin2){
  }
}

void MotorControl::motor2CallBack(int pi, uint32_t user_gpio, uint32_t level, uint32_t tick)
{
  if(user_gpio == motor_use_pin_.at(1).encorder_input_pin1){
  }else if(user_gpio == motor_use_pin_.at(1).encorder_input_pin2){
  }
}

void MotorControl::motor3CallBack(int pi, uint32_t user_gpio, uint32_t level, uint32_t tick)
{
  if(user_gpio == motor_use_pin_.at(2).encorder_input_pin1){
  }else if(user_gpio == motor_use_pin_.at(2).encorder_input_pin2){
  }
}

int MotorControl::setDutyCycle(const std::vector<uint32_t> &duty_cycle)
{
  if(duty_cycle.size() != motor_use_num_) return -1;
  for(std::size_t i=0;i<motor_use_num_;i++)
  {
    if(isCheckError(set_PWM_dutycycle(pi_state_, motor_use_pin_.at(i).pwm_pin, duty_cycle.at(i)), "set pwm dutycycle error ", PI_BAD_USER_GPIO, PI_BAD_DUTYCYCLE, PI_NOT_PERMITTED)) return -1;
  }

  return 0;
}

void motorMove(const std::vector<MotorCommand> &motor_command)
{
  if(motor_command.size() != motor_use_num_) return;
  std::vector<uint32_t> send_duty_cycle;
  for(std::size_t i=0;i<motor_use_num_;i++)
  {
    send_duty_cycle.push_back(motor_command.at(i).dutycycle);
  }
  int ret = setDutyCycle(send_duty_cycle);
  if(ret) return;
  if(motor_command.rotation_direction == RotationDirection::CW){
  }else {
  }

}

