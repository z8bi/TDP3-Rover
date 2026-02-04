#ifndef PINASSIGNMENTS_HPP
#define PINASSIGNMENTS_HPP

#include "mbed.h"

PwmOut ENABLE_A(D3); //PTA12
PwmOut ENABLE_B(D5); //PTA5

DigitalOut INI1_A(D2); //PTD4
DigitalOut INI2_A(D9); //PTD5
DigitalOut INI3_B(D8); //PTA13
DigitalOut INI4_B(D10); //PTD0
DigitalIn Left_TCRT(D0); //PTA2
DigitalIn Right_TCRT(D1); //PTA1
DigitalIn Centre_TCRT(D4); //PTA4

#endif // PINASSIGNMENTS_HPP