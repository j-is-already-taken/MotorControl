
#include "omni_wheel/omni_wheel_control.hpp"


int main(int argc, char* argv[])
{
  OmniWheelControl omni_wheel_control;
  for(int i=0; i< 360;i++)
  {
    std::cout << "angle = " << i << std::endl;;
    auto [motor1_ratio, motor2_ratio, motor3_ratio] = omni_wheel_control.moveRobot(Angle{AngleType::Degree, i}, 255);
    //omni_wheel_control.moveRobot(Angle{AngleType::Degree, static_cast<double>(i)}, 255).printAll();

    std::array<double, 3> motor_ratio{motor1_ratio, motor2_ratio, motor3_ratio};
    std::vector<MotorCommand> motor_command;
    for(const auto &tmp_motor_ratio: motor_ratio)
    {
      RotationDirection rotation_direction = RotationDirection::CW;
      if(tmp_motor_ratio < 0)
      {
        rotation_direction = RotationDirection::CCW;
      }
      motor_command.emplace_back(MotorCommand{std::abs(tmp_motor_ratio), rotation_direction});
    }
  }
  //auto [] = omni_wheel_control();

  return 0;
}
