/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment: Lab 2 part 4
 * Description: This program uses a proportional control algorithm to make the robot follow a line using data from two IR sensors.
*/

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_SENSOR 0
#define RIGHT_SENSOR 1

#define INIT_SPEED 5
#define MIN_SPEED 0
#define MAX_SPEED 100

#define LIMIT_SPEED 50

#define Kp 1.45
#define MOTOR_CAL 1.1811


#define SENSOR_READ_DELAY 100
#define MAX_DELTA 255

void motor(uint8_t, int8_t);


int main(void) {
    init();  //initialize board hardware
    u08 left_sensor;
    u08 right_sensor;
    int8_t left_speed;
    int8_t right_speed;

    /**
     * stores the difference between left & right sensor readings.
     * (+) left sensor is larger than right sensor
     * (-) right sensor is larger than left sensor
    */
    int16_t sensor_delta;
    int8_t speed_delta;

    // delay 5s before starting motors
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    _delay_ms(5000);

    // start motors at initial speed
    motor(LEFT_MOTOR, INIT_SPEED);
    motor(RIGHT_MOTOR, INIT_SPEED);

    while(1) {
        _delay_ms(SENSOR_READ_DELAY);
        left_sensor = analog(LEFT_SENSOR); // left wheel
        right_sensor = analog(RIGHT_SENSOR); // right wheel
        
        sensor_delta = left_sensor - right_sensor;

        speed_delta = sensor_delta * MAX_SPEED / MAX_DELTA * Kp;

        /*
        // limit validation
        if (speed_delta > MAX_SPEED) {
            speed_delta = MAX_SPEED;
        }
        if (speed_delta < MIN_SPEED) {
            speed_delta = MIN_SPEED;
        }
        */

        left_speed = INIT_SPEED - speed_delta;
        right_speed = INIT_SPEED + speed_delta;
        /* new limit validation */
        if (left_speed < 0) {
            left_speed = 0;
        }
        if (left_speed > LIMIT_SPEED) {
            left_speed = LIMIT_SPEED;
        }
        if (right_speed < 0) {
            right_speed = 0;
        }
        if (right_speed > LIMIT_SPEED) {
            right_speed = LIMIT_SPEED;
        }
        
        motor(LEFT_MOTOR, left_speed);
        motor(RIGHT_MOTOR, right_speed);

        /* testing */
        clear_screen();
        lcd_cursor(0,0);
        print_num(left_sensor);
        lcd_cursor(5,0);
        print_num(right_sensor);
        lcd_cursor(0,1);
        print_num(-left_speed);
        lcd_cursor(5,1);
        print_num(right_speed);

    }
    
}


void motor(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    if (num == LEFT_MOTOR) {
        servo_speed = 127 + (int8_t) (MOTOR_CAL*speed*127/100 * 0.5);
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == RIGHT_MOTOR) {
        servo_speed = 127 - (int8_t) (speed*127/100 * 0.5);
        set_servo(num, servo_speed);
    }
    
}

