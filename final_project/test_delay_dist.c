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
#define MAX_SPEED_FAST 100
#define CAN_SENSOR_THRESHOLD 85
#define EDGE_SENSOR_THRESHOLD 120

// #define MS_PER_CM 65.07011
#define MS_PER_CM 76.21951
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
void motor_fast(uint8_t, int8_t);
uint8_t rotate_robot_deg(uint8_t, uint16_t);

int main(void){
    init();
    uint16_t count = 0; 
    // uint16_t test_dist = 2500; 
    // uint16_t desired_dist = 30;
    uint16_t converted_dist= (u16) MS_PER_CM*desired_dist;
    // init_encoder();
    motor_fast(LEFT_MOTOR, 0);
    motor_fast(RIGHT_MOTOR, 0);
    _delay_ms(1000);
    while(1){
        motor_fast(LEFT_MOTOR, 0);
        motor_fast(RIGHT_MOTOR, 0);
        count = 0;
        while(!get_btn());
        while(count < test_dist){
        motor_fast(LEFT_MOTOR, MAX_SPEED_FAST);
        motor_fast(RIGHT_MOTOR, MAX_SPEED_FAST);
        _delay_ms(1);
        count ++;
        // break;
        }
        }
    
    // while(count < converted_dist){
    //     // motor(LEFT_MOTOR, MAX_SPEED);
    //     // motor(RIGHT_MOTOR, MAX_SPEED);
    //     motor_fast(LEFT_MOTOR, MAX_SPEED_FAST);
    //     motor_fast(RIGHT_MOTOR, MAX_SPEED_FAST);
    //     _delay_ms(1);
    //     count ++;
    //     // break;
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

void motor_fast(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    // left wheel
    if (num == LEFT_MOTOR) {
        servo_speed = 127 + (int8_t) (SERVO_0_CAL*speed*127/100);
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == RIGHT_MOTOR) {
        servo_speed = 127 - (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
    }
}