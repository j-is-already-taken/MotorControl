#include "omni_wheel/motor_control.hpp"

MotorControl::MotorControl(): pi_state(-1)
{
  for(std::size_t i=0;i<motor_use_num;i++)
  {
    motor_use_pin_.at(i).pwm_pin_ = 0;
    motor_use_pin_.at(i).rotate_control_pin1_ = 0;
    motor_use_pin_.at(i).rotate_control_pin2_ = 0;
  }
  //pi_state = pigpio_start(nullptr, nullptr);
  if(pi_state < 0){
  }
}
