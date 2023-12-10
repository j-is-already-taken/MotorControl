#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include <pigpiod_if2.h>

#include <iostream>
#include <array>
#include <vector>


class GPIOControl{
};

enum class TargetMotor{
  Motor1, 
  Motor2, 
  Motor3, 
  Motor1And2, 
  Motor1And3, 
  Motor2And3, 
  All
};

struct MotorUsePin{
  uint32_t pwm_pin;

  uint32_t rotate_control_pin1;
  uint32_t rotate_control_pin2;

  uint32_t encorder_input_pin1;
  uint32_t encorder_input_pin2;
};

enum class RotationDirection
{
  CW, 
  CCW, 
  Right=CW, 
  Left=CCW
};

struct MotorCommand
{
  MotorCommand(): dutycycle(0), rotation_direction(RotationDirection::CW)
  {}
  MotorCommand(uint32_t tmp_dutycycle, RotationDirection tmp_rotation_direction): dutycycle(tmp_dutycycle), rotation_direction(tmp_rotation_direction)
  {}
  uint32_t dutycycle;
  RotationDirection rotation_direction;

};

class MotorControl{
public:
  MotorControl();
  ~MotorControl();

  static void motor1CallBack(int , uint32_t, uint32_t, uint32_t);
  static void motor2CallBack(int , uint32_t, uint32_t, uint32_t);
  static void motor3CallBack(int , uint32_t, uint32_t, uint32_t);

  int setDutyCycle(const std::vector<uint32_t> & );
  void moveMotor(const std::vector<MotorCommand> &);
  void stopMotor();

private:
  int initSetup();
  int pi_state_;
  uint32_t frequency_;
  uint32_t dutycycle_;

  const std::size_t motor_use_num_=3;
  //std::array<MotorUsePin, motor_use_num> motor_use_pin_;
  static std::array<MotorUsePin, 3> motor_use_pin_;
};

#endif //MOTOR_CONTROL_HPP_

//std::array<MotorUsePin, 3> MotorControl::motor_use_pin_({0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
