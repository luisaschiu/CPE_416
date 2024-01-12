#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define SERVO_0_CAL 1.25

void motor(uint8_t, int8_t);

int main(void) {
    init();  //initialize board hardware
    motor(0,0);
    motor(1,0);
    // _delay_ms(100);
    // motor(0,5);
    // motor(1,5);
    _delay_ms(1000);
    motor(0,-10);
    motor(1,10);
    _delay_ms(2050);
    motor(0,100);
    motor(1,100);
    _delay_ms(2000);
}

void motor(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    // left wheel
    if (num == LEFT_MOTOR) {
        servo_speed = 127 + (int8_t) (SERVO_0_CAL*speed*127/100 * 0.5);
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == RIGHT_MOTOR) {
        servo_speed = 127 - (int8_t) (speed*127/100 * 0.5);
        set_servo(num, servo_speed);
    }
}