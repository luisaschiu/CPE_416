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
#define HALF_SQUARE_CM 39
#define QUARTER_SQUARE_CM 19.5 

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define FORWARD 0 
#define BACKWARD 1
#define RETRACT_DISTANCE_CM 10

#define ROTATE_SPEED_FORWARD 10 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED
#define ROTATE_SPEED_BACKWARD -10 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED


void motor(uint8_t, int8_t);
void push_can_off_edge();

int main(void){
    init();
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    _delay_ms(1000);
    push_can_off_edge();
}

void push_can_off_edge() {
    uint8_t left_sensor = analog(LEFT_LINE_SENSOR);
    uint8_t right_sensor = analog(RIGHT_LINE_SENSOR);
    motor(0, MAX_SPEED);
    motor(1, MAX_SPEED);
    while ((left_sensor < EDGE_SENSOR_THRESHOLD) && (right_sensor < EDGE_SENSOR_THRESHOLD)) {
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
    } 
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    clear_screen();
    lcd_cursor(0, 0);
    print_string("Line");
    while (right_sensor < EDGE_SENSOR_THRESHOLD){
        motor(LEFT_MOTOR, -13);
        motor(RIGHT_MOTOR, 15);
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        // clear_screen();
        // lcd_cursor(5,0);
        // print_num(right_sensor);
        // lcd_cursor(0,0);
        // print_num(left_sensor);
        // _delay_ms(100);
    }
    while (left_sensor < EDGE_SENSOR_THRESHOLD){
        motor(LEFT_MOTOR, 15);
        motor(RIGHT_MOTOR, -13);
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        // clear_screen();
        // lcd_cursor(5,0);
        // print_num(right_sensor);
        // lcd_cursor(0,0);
        // print_num(left_sensor);
        // _delay_ms(100);
    }
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    lcd_cursor(0,1);
    print_string("Square");
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