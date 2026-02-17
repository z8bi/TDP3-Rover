// rovercontrol.hpp
#ifndef ROVERCONTROL_HPP
#define ROVERCONTROL_HPP

#include "pinassignments.hpp"
#include <cmath>

// Configure once
inline void motors_init(float pwm_period_s = 0.001f) {
    ENABLE_A.period(pwm_period_s);
    ENABLE_B.period(pwm_period_s);

    // Default: stop
    ENABLE_A.write(0.0f);
    ENABLE_B.write(0.0f);

    INI1_A = 0; INI2_A = 0;
    INI3_B = 0; INI4_B = 0;
}

// Signed speed in [-1, +1]. Sign sets direction, magnitude sets PWM duty.
inline void motor_set(float left, float right) {
    auto clamp = [](float x) {
        if (x > 1.0f) return 1.0f;
        if (x < -1.0f) return -1.0f;
        return x;
    };
    left = clamp(left);
    right = clamp(right);

    // Left motor direction (INI1_A / INI2_A)
    if (left > 0.0f) { INI1_A = 1; INI2_A = 0; }
    else if (left < 0.0f) { INI1_A = 0; INI2_A = 1; }
    else { INI1_A = 0; INI2_A = 0; }

    // Right motor direction (INI3_B / INI4_B)
    // Your existing FORWARD() uses INI3_B=0, INI4_B=1 as "forward".
    if (right > 0.0f) { INI3_B = 0; INI4_B = 1; }
    else if (right < 0.0f) { INI3_B = 1; INI4_B = 0; }
    else { INI3_B = 0; INI4_B = 0; }

    ENABLE_A.write(std::fabs(left));
    ENABLE_B.write(std::fabs(right));
}

inline void motor_brake(bool enable) {
    if (!enable) return;
    // Fast stop (brake): enable high, inputs same (per your comment)
    ENABLE_A.write(1.0f);
    ENABLE_B.write(1.0f);
    INI1_A = 1; INI2_A = 1;
    INI3_B = 1; INI4_B = 1;
}

inline void motor_coast_stop() {
    ENABLE_A.write(0.0f);
    ENABLE_B.write(0.0f);
    INI1_A = 0; INI2_A = 0;
    INI3_B = 0; INI4_B = 0;
}

#endif
