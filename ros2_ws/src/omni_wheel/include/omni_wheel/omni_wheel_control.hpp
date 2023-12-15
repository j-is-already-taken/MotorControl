#ifndef MOTER_CONTROL_H_
#define MOTER_CONTROL_H_

#include <iostream>
#include <numbers>
#include <cmath>


struct Position_t{

};

enum class RotationalDirection
{
  Right, 
  Left, 
  CW = Right,
  CCW = Left
};

struct Velocity
{
  Velocity(): x_(0.0), y_(0.0), theta_(0.0)
  {
  }
  Velocity(double x, double y, double theta) : x_(x), y_(y), theta_(theta)
  {
  }

  double x_;
  double y_;
  double theta_;
};

struct OmniWheelMoterSpeed
{
  OmniWheelMoterSpeed():moter_1_(0.0), moter_2_(0.0), moter_3_(0.0)
  {
  }
  OmniWheelMoterSpeed(double moter_1, double moter_2, double moter_3):moter_1_(moter_1), moter_2_(moter_2), moter_3_(moter_3)
  {
  }
  double  moter_1_;
  double  moter_2_;
  double  moter_3_;
};

enum class AngleType
{
  Degree,
  Radian
};

struct Angle{
  Angle(): degree_(0), radian_(0){}
  Angle(const AngleType &type, const double &angle): degree_(0), radian_(0)
  {
    if(type == AngleType::Degree)
    {
      degree_ = angle;
      radian_ = translateDegreeToRadian(degree_);
    }else{
      radian_ = angle;
      degree_ = translateRadianToDegree(radian_);
    }
  }

  double translateDegreeToRadian(const double &degree)
  {
    double return_radian = degree * (std::numbers::pi / 180);
    return return_radian;
  }
  
  double translateRadianToDegree(const double &radian)
  {
    double return_degree = radian * (180 / std::numbers::pi);
    return return_degree;

  }

  void setDegree(const double &angle)
  {
    degree_ = angle;
    radian_ = translateDegreeToRadian(degree_);
  }

  void setRadian(const double &angle)
  {
    radian_ = angle;
    degree_ = translateRadianToDegree(radian_);
  }

  double degree_;
  double radian_;
};

class OmniWheelControl{
public:
  OmniWheelControl();
  ~OmniWheelControl();
  OmniWheelMoterSpeed moveRobot(const Angle &move_angle, const double &speed=1.0);
  OmniWheelMoterSpeed turnRobot(const RotationalDirection &direction, const double &speed=1.0);
private:

};

#endif // MOTER_CONTROL_H_
