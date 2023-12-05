#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

//#include <pigpiod_if2.h>

#include <iostream>
#include <array>


class GPIOControl{
};

struct MotorUsePin{
  uint32_t pwm_pin_;

  uint32_t rotate_control_pin1_;
  uint32_t rotate_control_pin2_;
};

class MotorControl{
public:
  MotorControl();
  ~MotorControl();
private:
  int pi_state;

  const std::size_t motor_use_num=3;
  //std::array<MotorUsePin, motor_use_num> motor_use_pin_;
  std::array<MotorUsePin, 3> motor_use_pin_;
};

#endif //MOTOR_CONTROL_HPP_
