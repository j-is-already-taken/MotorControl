#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

//#include <pigpiod_if2.h>

#include <iostream>
#include <array>


class GPIOControl{
};

struct MotorUsePin{
  uint32_t pwm_pin;

  uint32_t rotate_control_pin1;
  uint32_t rotate_control_pin2;

  uint32_t encorder_input_pin1;
  uint32_t encorder_input_pin2;
};

class MotorControl{
public:
  MotorControl();
  ~MotorControl();

  motor1CallBack(int , uint32_t, uint32_t, uint32_t);
  motor2CallBack(int , uint32_t, uint32_t, uint32_t);
  motor3CallBack(int , uint32_t, uint32_t, uint32_t);
private:
  int initSetup();
  int pi_state_;
  uint32_t frequency_;

  const std::size_t motor_use_num_=3;
  //std::array<MotorUsePin, motor_use_num> motor_use_pin_;
  std::array<MotorUsePin, 3> motor_use_pin_;
};

#endif //MOTOR_CONTROL_HPP_
