#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h> 
#include <time.h> 

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_LINE_SENSOR 0
#define RIGHT_LINE_SENSOR 1
#define Kp 1.45
#define MOTOR_CAL 1.258
#define SENSOR_READ_DELAY 100
#define MAX_DELTA 255
#define RANGE_SENSOR_PIN 2
#define SERVO_0_CAL 1.17
#define MAX_SPEED 25 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED
#define CAN_SENSOR_THRESHOLD 85
#define EDGE_SENSOR_THRESHOLD 120

#define TICKS_PER_CM 3.988239 
// #define TICKS_PER_DEGREE 0.588333
// #define MS_PER_DEGREE 22.486
#define MS_PER_DEGREE 15.8 //Value found from trial and error
// #define MS_PER_CM 65.07011
#define MS_PER_CM 35 //Value found from trial and error
#define RETRACT_MS_PER_CM 70 // CURRENTLY TESTING
#define SQUARE_UP_SPEED_POS 100
#define SQUARE_UP_SPEED_NEG -5
#define HALF_SQUARE_CM 39
#define QUARTER_SQUARE_CM 19.5 

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define FORWARD 0 
#define BACKWARD 1
#define RETRACT_DISTANCE_CM 10

#define ROTATE_SPEED_FORWARD 10 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED
#define ROTATE_SPEED_BACKWARD -10 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED

#define SQUARE_UP_SPEED_POS_A 20
#define SQUARE_UP_SPEED_NEG_A -18
// #define SQUARE_UP_SPEED_POS_L 20
// #define SQUARE_UP_SPEED_NEG_L -10

#define SQUARE_UP_SPEED_POS_L 100
#define SQUARE_UP_SPEED_NEG_L -5

void motor(uint8_t, int8_t);
void retract_from_line(uint8_t);

int main(void){
    init();
    retract_from_line(10);
}

void retract_from_line(uint8_t cm){
    uint8_t left_motor;
    uint8_t right_motor;
    uint16_t count = 0;
    uint16_t num_ms = (u16) (cm * RETRACT_MS_PER_CM);
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    left_motor = -MAX_SPEED;
    right_motor = -MAX_SPEED;
    _delay_ms(1000);
    while (count < num_ms) {
        motor(LEFT_MOTOR, left_motor); 
        motor(RIGHT_MOTOR, right_motor);
        _delay_ms(1);
        count++;
    }
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
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