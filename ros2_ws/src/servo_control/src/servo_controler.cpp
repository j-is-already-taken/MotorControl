#include "servo_control/servo_controler.hpp"

#include <algorithm>

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

ServoControler::ServoControler(): pi_state_(-1), pin_num_(25), frequency_(50)
{
  pi_state_ = pigpio_start(nullptr, nullptr);
  if(pi_state_ < 0){
    std::cerr << "not started. check running pigpiod daemon" << std::endl;
    return;
  }
  initSetup();
}

int ServoControler::initSetup()
{
    if(isCheckError(set_mode(pi_state_, pin_num_, PI_OUTPUT), "GPIO set error", PI_BAD_GPIO, PI_BAD_MODE, PI_NOT_PERMITTED)) return -1;
    if(isCheckError(set_PWM_frequency(pi_state_, pin_num_, frequency_), "set PWM error", PI_BAD_USER_GPIO, PI_NOT_PERMITTED)) return -1;

    if(isCheckError(set_servo_pulsewidth(pi_state_, pin_num_, 500), "set pulse width error", PI_BAD_USER_GPIO, PI_BAD_PULSEWIDTH, PI_NOT_PERMITTED)) return -1;
}

bool moveServoForBallRepel(const uint32_t &start_angle_pulse, const uint32_t &target_angle_pulse)
{
  uint32_t start_angle = std::clamp(start_angle_pulse, MIN_ANGLE_PULSE, MAX_ANGLE_PULSE);
  uint32_t target_angle = std::clamp(target_angle_pulse, MIN_ANGLE_PULSE, MAX_ANGLE_PULSE);

  if(isCheckError(set_servo_pulsewidth(pi_state_, pin_num_, target_angle), "set pulse width error", PI_BAD_USER_GPIO, PI_BAD_PULSEWIDTH, PI_NOT_PERMITTED)) return false;
  time_sleep(1);
  if(isCheckError(set_servo_pulsewidth(pi_state_, pin_num_, start_angle), "set pulse width error", PI_BAD_USER_GPIO, PI_BAD_PULSEWIDTH, PI_NOT_PERMITTED)) return false;

  return true;
}
