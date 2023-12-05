#include "omni_wheel/omni_wheel_control.hpp"

#include <Eigen/Core>


OmniWheelControl::OmniWheelControl()
{
}

OmniWheelControl::~OmniWheelControl()
{
}

OmniWheelMoterSpeed OmniWheelControl::moveRobot(const Angle &move_angle, const double &speed)
{
  double velocity_x = std::abs(speed * std::cos(move_angle.radian_));
  double velocity_y = std::abs(speed * std::sin(move_angle.radian_));
  double omega = 0.0;



  double L = 0.0;
  //Eigen::MatrixXd global_V (velocity_x, velocity_y, omega);
  Eigen::Vector3d global_V (velocity_x, velocity_y, omega);
  Eigen::Matrix3d rotate_local;
  rotate_local << std::cos(move_angle.radian_), std::sin(move_angle.radian_), L,
                  -std::cos(move_angle.radian_ + (2 * std::numbers::pi) / 3), std::sin(move_angle.radian_ + (2 * std::numbers::pi) / 3), L, 
                  -std::cos(move_angle.radian_ + std::numbers::pi / 3), std::sin(move_angle.radian_ + std::numbers::pi / 3), L;

  Eigen::VectorXd local_velo = rotate_local * global_V;

  std::cout << local_velo << std::endl;

  OmniWheelMoterSpeed each_moter_speed(local_velo(0), local_velo(1), local_velo(2));

  return each_moter_speed;
}

int OmniWheelControl::moveTurn(const Angle &turning_angle, const RotationalDirection &direction)
{
  return 0;
}
