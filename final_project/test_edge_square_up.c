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

#define ROTATE_SPEED_FORWARD 10
#define ROTATE_SPEED_BACKWARD -10
#define SQUARE_UP_SPEED_POS_A 20
#define SQUARE_UP_SPEED_NEG_A -18
// #define SQUARE_UP_SPEED_POS_L 20
// #define SQUARE_UP_SPEED_NEG_L -10

#define SQUARE_UP_SPEED_POS_L 100
#define SQUARE_UP_SPEED_NEG_L -5

void motor(uint8_t, int8_t);
void square_up_Asa(uint8_t, uint8_t);
void square_up_Luisa(uint8_t, uint8_t);

int main(void){
    init();
    u08 left_sensor;
    u08 right_sensor;
    motor(0, 0);
    motor(1, 0);
    _delay_ms(1000);
    while(1) {
        motor(0, MAX_SPEED);
        motor(1, MAX_SPEED);
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        clear_screen();
        lcd_cursor(5,0);
        print_num(right_sensor);
        lcd_cursor(0,0);
        print_num(left_sensor);
        if ((left_sensor > EDGE_SENSOR_THRESHOLD) || (right_sensor > EDGE_SENSOR_THRESHOLD)) {
            // Edge has been detected. Square the robot up against the line, then stop.
            // edge_detect_Asa(left_sensor, right_sensor);
            square_up_Luisa(left_sensor, right_sensor);
            break;
        } 
    }
}

void square_up_Asa(uint8_t left_sensor, uint8_t right_sensor){
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    lcd_cursor(5,0);
    print_num(right_sensor);
    lcd_cursor(0,0);
    print_num(left_sensor);
    lcd_cursor(0, 1);
    print_string("Edge");
    while (right_sensor < EDGE_SENSOR_THRESHOLD){
        motor(LEFT_MOTOR, SQUARE_UP_SPEED_NEG_A);
        motor(RIGHT_MOTOR, SQUARE_UP_SPEED_POS_A);
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
    }
    while (left_sensor < EDGE_SENSOR_THRESHOLD){
        motor(LEFT_MOTOR, SQUARE_UP_SPEED_POS_A);
        motor(RIGHT_MOTOR, SQUARE_UP_SPEED_NEG_A);
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
    }
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    lcd_cursor(0,1);
    print_string("Square");
    // return;
}

void square_up_Luisa(uint8_t left_sensor, uint8_t right_sensor){
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    if (left_sensor > EDGE_SENSOR_THRESHOLD){
        clear_screen();
        lcd_cursor(0,1);
        print_string("Edge");
        while (right_sensor < EDGE_SENSOR_THRESHOLD){
            motor(LEFT_MOTOR, SQUARE_UP_SPEED_NEG_L);
            motor(RIGHT_MOTOR, SQUARE_UP_SPEED_POS_L);
            left_sensor = analog(LEFT_LINE_SENSOR);
            right_sensor = analog(RIGHT_LINE_SENSOR);
        }
    }
    else if (right_sensor > EDGE_SENSOR_THRESHOLD){
        while (left_sensor < EDGE_SENSOR_THRESHOLD){
            motor(LEFT_MOTOR, SQUARE_UP_SPEED_POS_L);
            motor(RIGHT_MOTOR, SQUARE_UP_SPEED_NEG_L);
            left_sensor = analog(LEFT_LINE_SENSOR);
            right_sensor = analog(RIGHT_LINE_SENSOR);
        }
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
