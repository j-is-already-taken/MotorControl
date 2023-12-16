#ifndef SERVO_CONTROLER_H_
#define SERVO_CONTROLER_H_

#include <pigpiod_if2.h>
#include <iostream>

class ServoControler{
  public:
    ServoControler();
    ~ServoControler();
    bool moveServoForBallRepel(const uint32_t &start_angle_pulse=500, const uint32_t &target_angle_pulse=950);
    int initSetup();
  private:
    int pi_state_;
    uint32_t pin_num_;
    uint32_t frequency_;
    const uint32_t MAX_ANGLE_PULSE=2500;
    const uint32_t MIN_ANGLE_PULSE=500;
};

#endif
