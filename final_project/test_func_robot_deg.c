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
#define CAN_SENSOR_THRESHOLD 95
#define EDGE_SENSOR_THRESHOLD 120

// #define MS_PER_CM 35 //Value found from trial and error
// #define TICKS_PER_DEGREE 0.588333
// #define MS_PER_DEGREE 22.486 // value found from testing separately
#define MS_PER_DEGREE 15.8 //Value found from trial and error
#define HALF_SQUARE_CM 39
#define QUARTER_SQUARE_CM 19.5 

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define FORWARD 0 
#define BACKWARD 1
#define RETRACT_DISTANCE_CM 10

#define ROTATE_SPEED_FORWARD 10
#define ROTATE_SPEED_BACKWARD -10

#define MAX_SPEED_FAST 100
#define MS_PER_CM 52
#define MS_PER_DEGREE 15.8

void motor(uint8_t, int8_t);
void motor_fast(uint8_t, int8_t);
uint8_t rotate_robot_deg(uint8_t, uint16_t);

int main(void){
    init();
    motor_fast(LEFT_MOTOR, 0);
    motor_fast(RIGHT_MOTOR, 0);
    uint8_t result_rotate;
    _delay_ms(1000);
    while(1){
        result_rotate = rotate_robot_deg(CLOCKWISE, 360);
        motor_fast(LEFT_MOTOR, 0);
        motor_fast(RIGHT_MOTOR, 0);
        print_num(result_rotate);
        break;
    }
}

uint8_t rotate_robot_deg(uint8_t direction, uint16_t degrees) {
    uint8_t left_sensor;
    uint8_t right_sensor;
    uint8_t left_motor;
    uint8_t right_motor;
    uint16_t count = 0;
    uint16_t num_ms = (u16) (degrees * MS_PER_DEGREE);
    /* Determine motor speeds based off direction */
    if (direction == CLOCKWISE) {
        left_motor = ROTATE_SPEED_FORWARD;
        right_motor = ROTATE_SPEED_BACKWARD;
    }
    else {
        left_motor = ROTATE_SPEED_BACKWARD;
        right_motor = ROTATE_SPEED_FORWARD;
    }
    while (count < num_ms) {
        motor_fast(LEFT_MOTOR, left_motor); 
        motor_fast(RIGHT_MOTOR, right_motor);
        /* Continue rotating */
        // lcd_cursor(0, 0);
        // print_num(count);
        // lcd_cursor(0, 1);
        // print_num(num_ms);
        _delay_ms(1);
        /* Edge checking */
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        if ((left_sensor > EDGE_SENSOR_THRESHOLD) || (right_sensor > EDGE_SENSOR_THRESHOLD)) {
            // Edge has been detected. Square the robot up against the line, then stop.
            motor_fast(LEFT_MOTOR, 0);
            motor_fast(RIGHT_MOTOR, 0);
            clear_screen();
            lcd_cursor(0,0);
            print_string("Edge");
            while (right_sensor < EDGE_SENSOR_THRESHOLD){
                motor_fast(LEFT_MOTOR, -13);
                motor_fast(RIGHT_MOTOR, 15);
                left_sensor = analog(LEFT_LINE_SENSOR);
                right_sensor = analog(RIGHT_LINE_SENSOR);
            }
            while (left_sensor < EDGE_SENSOR_THRESHOLD){
                motor_fast(LEFT_MOTOR, 15);
                motor_fast(RIGHT_MOTOR, -13);
                left_sensor = analog(LEFT_LINE_SENSOR);
                right_sensor = analog(RIGHT_LINE_SENSOR);
            }
            motor_fast(LEFT_MOTOR, 0);
            motor_fast(RIGHT_MOTOR, 0);
            lcd_cursor(0,1);
            print_string("Square");
            return 1;
        }
        /* Can checking */
        if (analog(RANGE_SENSOR_PIN) > CAN_SENSOR_THRESHOLD) {
            motor_fast(LEFT_MOTOR, 0);
            motor_fast(RIGHT_MOTOR, 0);
            lcd_cursor(0,1);
            print_string("Can");
            return 2;
        }
        count ++;
    }
    return 0;
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
    uint8_t servo_delta;
    // left wheel
    
    if (num == LEFT_MOTOR) {
        servo_delta = (int8_t) (SERVO_0_CAL*speed*127/100);
        if (servo_delta >= 127){
            servo_delta = 127;
        }
        else if (servo_delta < -127){
            servo_delta = -127;
        }
        servo_speed = 127 + servo_delta;
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == RIGHT_MOTOR) {
        servo_speed = 127 - (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
    }
}