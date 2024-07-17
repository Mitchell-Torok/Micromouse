#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {


// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
        // TODO: Set both pins as output
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        prev_pwm = 0;
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm, int tuner) {
      
      
      // Serial.println(diff);
      

      if (pwm >= 0) {
        digitalWrite(dir_pin, HIGH);
        
      } else {
        digitalWrite(dir_pin, LOW);
      }
      int output = abs(pwm);
      if (output > 225) {
        output = 225;
      }
      output = output - (tuner * 2);

      if (output <= 0) {
        output = 0;
      }
      int diff = abs(output - getPwm());

      if (diff >= 5 && output >= prev_pwm) {
        output = prev_pwm + (diff / 40);
      }
      
      


      prev_pwm = output;
      

      

      //output = output - (tuner * 20);
    
 
      
      
      analogWrite(pwm_pin, output);
      // TODO: Output digital direction pin based on if input signal is positive or negative.
      // TODO: Output PWM signal between 0 - 255.
    }

int getPwm() {
    return prev_pwm;
}

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
    int prev_pwm;
};

}  // namespace mtrn3100
