#ifndef ROVERCONTROL_HPP
#define ROVERCONTROL_HPP

#include "pinassignments.hpp"


void FORWARD() { //AS PER DATASHEET, ONE INPUT MUST BE HIG AND ONE LOW, WIHT ENABLES BEING HIGH
    ENABLE_A.period(0.001);
    ENABLE_A.pulsewidth(0.0007);
    ENABLE_B.period(0.001);
    ENABLE_B.pulsewidth(0.0007);
    INI1_A = 1;
    INI2_A = 0;
    INI3_B = 0;
    INI4_B = 1;
    ThisThread::sleep_for(1000);
}

void REVERSE() { // SAME AS FORWADR FUNCTION EXCEPT FLIP THE INPUT THAT IS HIGH
    ENABLE_A.period(0.001);
    ENABLE_A.pulsewidth(0.0007);
    ENABLE_B.period(0.001);
    ENABLE_B.pulsewidth(0.0007);
    INI1_A = 0;
    INI2_A = 1;
    INI3_B = 1;
    INI4_B = 0;
    ThisThread::sleep_for(1000);
}

void FAST_MOTOR_STOP() { //AS PER DATASHEET, ENABLES MUST BE HIGH AND INPUT MUST BE THE SAME
    ENABLE_A.period(0.001);
    ENABLE_A.pulsewidth(0.001);
    ENABLE_B.period(0.001);
    ENABLE_B.pulsewidth(0.001);
    INI1_A = 1;
    INI2_A = 1;
    INI3_B = 1;
    INI4_B = 1;
    ThisThread::sleep_for(1000);
}


void FREE_RUNNING_MOTOR_STOP() { // AS PER DATASHEET, ENABLE HAS TO BE LOW AND INPUTS ARE DONT CARE
    ENABLE_A = 0;
    ENABLE_B = 0;
    INI1_A = 1;
    INI2_A = 1;
    INI3_B = 1;
    INI4_B = 1;
    ThisThread::sleep_for(1000);
}

void SKID_RIGHT() {
    ENABLE_A.period(0.001);
    ENABLE_A.pulsewidth(0.0008);
    ENABLE_B.period(0.001);
    ENABLE_B.pulsewidth(0.0008);
    INI1_A = 1;
    INI2_A = 0;
    INI3_B = 1;
    INI4_B = 0;
    ThisThread::sleep_for(1000);
}

void SKID_LEFT() {
    ENABLE_A.period(0.001);
    ENABLE_A.pulsewidth(0.00085);
    ENABLE_B.period(0.001);
    ENABLE_B.pulsewidth(0.00085);
    INI1_A = 0;
    INI2_A = 1;
    INI3_B = 0;
    INI4_B = 1;
    ThisThread::sleep_for(1000);
}

void TURN_LEFT() {
    ENABLE_A.period(0.001);
    ENABLE_A.pulsewidth(0.001);
    ENABLE_B.period(0.001);
    ENABLE_B.pulsewidth(0.001);
    INI1_A = 1;
    INI2_A = 1;
    INI3_B = 0;
    INI4_B = 1;
    ThisThread::sleep_for(1000);
}

void TURN_RIGHT() {
    ENABLE_A.period(0.001);
    ENABLE_A.pulsewidth(0.001);
    ENABLE_B.period(0.001);
    ENABLE_B.pulsewidth(0.001);
    INI1_A = 1;
    INI2_A = 0;
    INI3_B = 1;
    INI4_B = 1;
    ThisThread::sleep_for(1000);
}
#endif // ROVERCONTROL_HPP