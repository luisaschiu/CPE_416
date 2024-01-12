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
#define MAX_SPEED 25
#define CAN_SENSOR_THRESHOLD 85
#define EDGE_SENSOR_THRESHOLD 120

#define TICKS_PER_CM 3.988239
// #define TICKS_PER_DEGREE 0.588333
#define TICKS_PER_DEGREE 0.8
#define HALF_SQUARE_CM 39
#define QUARTER_SQUARE_CM 19.5 

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define FORWARD 0 
#define BACKWARD 1
#define RETRACT_DISTANCE_CM 10

#define ROTATE_SPEED_FORWARD 10
#define ROTATE_SPEED_BACKWARD -10

void motor(uint8_t, int8_t);
uint8_t rotate_robot_deg(uint8_t, uint16_t);

int main(void){
    init();
    uint16_t count = 0; 
    uint16_t full_circle = 8095; 
    // init_encoder();
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    _delay_ms(1000);
    // CLOCKWISE
    // while(1){
    //     motor(LEFT_MOTOR, ROTATE_SPEED_FORWARD);
    //     motor(RIGHT_MOTOR, ROTATE_SPEED_BACKWARD);
    //     _delay_ms(8095);
    //     break;
    // }
    while(count < full_circle){
        motor(LEFT_MOTOR, ROTATE_SPEED_FORWARD);
        motor(RIGHT_MOTOR, ROTATE_SPEED_BACKWARD);
        _delay_ms(1);
        count ++;
        // break;
    }
    // while(1){
    //     motor(LEFT_MOTOR, ROTATE_SPEED_FORWARD);
    //     motor(RIGHT_MOTOR, ROTATE_SPEED_BACKWARD);
    //     _delay_ms(2024);
    //     break;
    // }

    // _delay_ms(2000);
    // COUNTER-CLOCKWISE
    // while(1){
    //     motor(LEFT_MOTOR, ROTATE_SPEED_BACKWARD);
    //     motor(RIGHT_MOTOR, ROTATE_SPEED_FORWARD);
    //     _delay_ms(8165);
    //     break;
    // }
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