#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {

        //TODO: Calculate the change in radians since the last update.
        float delta_left_radians = leftValue - lastLPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        float delta_right_radians = - (rightValue - lastRPos); // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 


        //TODO: Calculate the foward kinematics
        x += (delta_left_radians + delta_right_radians) * R / 2 * cos(h);
        y += (delta_left_radians + delta_right_radians) * R / 2 * sin(h);
        h += ((-R * delta_left_radians) / (2 * L)) + (R * delta_right_radians) / (2 * L);;
        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, L;
    float lastLPos, lastRPos;
};

}
