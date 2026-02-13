#include "pinassignments.hpp"
#include "rovercontrol.hpp"

int main() {
    
    while (1) {


    
    FORWARD();
    wait_us(3000000);
    FAST_MOTOR_STOP();
    SKID_LEFT();
    wait_us(2500000);
    FAST_MOTOR_STOP();
    REVERSE();
    wait_us(3000000);
    FAST_MOTOR_STOP();

    FORWARD();
    wait_us(100000);
    FAST_MOTOR_STOP();
    SKID_LEFT();
    wait_us(39000);
    FAST_MOTOR_STOP();
    FORWARD();
    wait_us(100000);
    FAST_MOTOR_STOP();
    SKID_LEFT();
    wait_us(39000);
    FAST_MOTOR_STOP();
    FORWARD();
    wait_us(100000);
    FAST_MOTOR_STOP();
    SKID_LEFT();
    wait_us(39000);
    FAST_MOTOR_STOP();
    FORWARD();
    wait_us(100000);
    FAST_MOTOR_STOP();
    SKID_LEFT();
    wait_us(39000);
    FAST_MOTOR_STOP();
    

    TURN_LEFT();
    wait_us(4800000);
    FAST_MOTOR_STOP();
    TURN_RIGHT();
    wait_us(4800000);
    FAST_MOTOR_STOP();
    wait_us(3000000);


    }
}

