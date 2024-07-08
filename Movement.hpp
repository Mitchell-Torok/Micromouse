#pragma once

#include <Arduino.h>

#define WHEELRAD 16
#define AXLELENGTH 98

namespace mtrn3100 {


class Movement {
public:
    Movement(PIDController& controllerLeft, PIDController& controllerRight, DualEncoder& encoder)
    : controllerLeft(controllerLeft), controllerRight(controllerRight), encoder(encoder){}


    void Movement::forward(float distance) {
      controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), distance / WHEELRAD);
      controllerRight.zeroAndSetTarget(encoder.getRightRotation(), -distance / WHEELRAD);
    }

    void Movement::left() {
      controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), (AXLELENGTH * PI)/ (4 * WHEELRAD));
      controllerRight.zeroAndSetTarget(encoder.getRightRotation(), (AXLELENGTH * PI) / (4 * WHEELRAD));
    }

    void Movement::right() {
      controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), (-AXLELENGTH * PI)/ (4 * WHEELRAD));
      controllerRight.zeroAndSetTarget(encoder.getRightRotation(), (-AXLELENGTH * PI) / (4 * WHEELRAD));
    }

  private:
        PIDController& controllerLeft;
        PIDController& controllerRight;
        DualEncoder& encoder;
  };
}
