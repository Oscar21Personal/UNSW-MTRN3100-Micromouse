#pragma once

#include <Arduino.h>

namespace mtrn3100 {
  
class EncoderOdometry {
public:
    EncoderOdometry() = default;

    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue, float rightValue) {
        // tL, tR are delta_theta_L, delta_theta_R
        float tL = leftValue - lastLPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 
        float tR = rightValue - lastRPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 

        float delta_s = (R * tL / 2) + (R * tR / 2);
        float delta_theta = (-R * tL / L) + (R * tR / L);

        x += delta_s * cos(h);
        y += delta_s * sin(h);
        h += delta_theta;   // h is theta
        
        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    void reset() {
      x = 0;
      y = 0;
      h = 0;
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
