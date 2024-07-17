#pragma once

#include <Arduino.h>

#define LEFTWHEEL 16
#define RIGHTWHEEL 16
#define DISTANCEMULTI 1.135
#define AXLELENGTH 98

namespace mtrn3100 {


class Movement {
public:
    Movement(PIDController& controllerLeft, PIDController& controllerRight, DualEncoder& encoder)
    : controllerLeft(controllerLeft), controllerRight(controllerRight), encoder(encoder){}


    void Movement::forward(float distance) {
      controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), (-distance / LEFTWHEEL) * DISTANCEMULTI);
      controllerRight.zeroAndSetTarget(encoder.getRightRotation(), (-distance / RIGHTWHEEL) * DISTANCEMULTI);
    }

    // void Movement::left() {
    //   controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), ((AXLELENGTH * PI)/ (4 * WHEELRAD)) * 0.93);
    //   controllerRight.zeroAndSetTarget(encoder.getRightRotation(), ((-AXLELENGTH * PI) / (4 * WHEELRAD)) * 1.14);
    // }

    // void Movement::right() {
    //   controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), ((-AXLELENGTH * PI)/ (4 * WHEELRAD)) * 0.93);
    //   controllerRight.zeroAndSetTarget(encoder.getRightRotation(), ((AXLELENGTH * PI) / (4 * WHEELRAD)) * 1.11);
    //}
    void Movement::stop() {
      controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), 0);
      controllerRight.zeroAndSetTarget(encoder.getRightRotation(), 0);
    }

  private:
        PIDController& controllerLeft;
        PIDController& controllerRight;
        DualEncoder& encoder;
  };
}
