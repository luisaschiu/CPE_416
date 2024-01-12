#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#define LEFT_WHEEL_PIN 0
#define RIGHT_WHEEL_PIN 1
#define LEFT_SENSOR_PIN 0
#define RIGHT_SENSOR_PIN 1
#define MIN_SENSOR_VAL 20
#define MAX_SENSOR_VAL 204
#define INIT_SPEED 6
#define MIN_SPEED 2
#define MOTOR_CAL 1.1811
#define SENSOR_READ_DELAY_MS 100
#define FIVE_SECONDS 5000
#define DELTA_THRESHOLD 175
#define TURN_TRIGGER 15
#define WHITE_THRESHOLD 31
// #define MOTOR_CAL 1
void motor(uint8_t, int8_t);

int main(void) {
    init();  //initialize board hardware
    u08 last_read_white = 0;
    u16 turn_enable_counter = 0;
    u08 left_sensor;
    u08 right_sensor;
    u08 turn_detect_left;
    u08 turn_detect_right;
    u08 sensor_delta;
    int8_t left_speed = INIT_SPEED;
    int8_t right_speed = INIT_SPEED;
    motor(LEFT_WHEEL_PIN, 0);
    motor(RIGHT_WHEEL_PIN, 0);
    _delay_ms(5000);
    
    /*
    while (1){
        motor(LEFT_WHEEL_PIN, 5);
        motor(RIGHT_WHEEL_PIN, -5);
        left_sensor = analog(LEFT_SENSOR_PIN); // left wheel
        right_sensor = analog(RIGHT_SENSOR_PIN); // right wheel
        lcd_cursor(0,0);
        print_num(left_sensor);
        lcd_cursor(5,0);
        print_num(right_sensor);
        lcd_cursor(5,1);
        print_num(right_speed);
        lcd_cursor(0,1);
        print_num(left_speed);
        _delay_ms(200);
        clear_screen();
    }
    */
    motor(LEFT_WHEEL_PIN, left_speed);
    motor(RIGHT_WHEEL_PIN, right_speed);
    while (1) {
        
        left_sensor = analog(LEFT_SENSOR_PIN); // left wheel
        right_sensor = analog(RIGHT_SENSOR_PIN); // right wheel
        lcd_cursor(0,0);
        print_num(left_sensor);
        lcd_cursor(5,0);
        print_num(right_sensor);
        lcd_cursor(5,1);
        print_num(right_speed);
        lcd_cursor(0,1);
        print_num(left_speed);
        _delay_ms(100);
        clear_screen();

        if (left_sensor < WHITE_THRESHOLD && right_sensor < WHITE_THRESHOLD) {
            if (last_read_white) {
                turn_enable_counter++;
            }
            else {
                last_read_white = 1;
                turn_enable_counter = 1;
            }

            if (turn_enable_counter >= TURN_TRIGGER) {
                turn_enable_counter = 0;    // reset turn enable counter
                motor(LEFT_WHEEL_PIN, 0);
                motor(RIGHT_WHEEL_PIN, 0);
                print_string("STOP");
                return 0;
            }
        }
        else {
            last_read_white = 0;
            turn_enable_counter = 0;
        }
        /*
        if (turn_enable && (sensor_delta > DELTA_THRESHOLD) ) {

            _delay_ms(100);
            turn_detect_left = analog(LEFT_SENSOR_PIN);
            turn_detect_right = analog(RIGHT_SENSOR_PIN);
            if (turn_detect_left < 50 && turn_detect_right < 50) {


                clear_screen();
                // if right sensor is large, turn right by slowing down right motor AND speeding up left motor
                if (right_sensor > left_sensor) {
                    print_string("DELTA");
                    sensor_delta = right_sensor - left_sensor;
                    // LIMIT SENSOR DELTA
                    if (sensor_delta > DELTA_THRESHOLD) {
                        sensor_delta = DELTA_THRESHOLD;
                    }
                    // right_speed = INIT_SPEED - sensor_delta*INIT_SPEED/(DELTA_THRESHOLD);
                    right_speed = 0 - sensor_delta*INIT_SPEED/(DELTA_THRESHOLD);    //NEW
                    // if (right_speed < MIN_SPEED) {   // NEW
                    //     right_speed = MIN_SPEED;
                    // }
                    left_speed = INIT_SPEED + sensor_delta*INIT_SPEED/(DELTA_THRESHOLD);
                    lcd_cursor(5,1);
                    print_num(right_speed);
                    lcd_cursor(0,1);
                    print_num(left_speed);
                    lcd_cursor(7,1);
                    print_string("R");
                    motor(RIGHT_WHEEL_PIN, right_speed);
                    motor(LEFT_WHEEL_PIN, left_speed);
                    
                }
                // if left sensor is large, turn left by slowing down left motor AND speeding up right motor
                else if (left_sensor > right_sensor) {
                    print_string("DELTA");
                    sensor_delta = left_sensor - right_sensor;
                    // LIMIT SENSOR DELTA
                    if (sensor_delta > DELTA_THRESHOLD) {
                        sensor_delta = DELTA_THRESHOLD;
                    }
                    // left_speed = INIT_SPEED - sensor_delta*INIT_SPEED/(DELTA_THRESHOLD);
                    left_speed = 0 - sensor_delta*INIT_SPEED/(DELTA_THRESHOLD); // NEW
                    // if (left_speed < MIN_SPEED) {    // NEW
                    //     left_speed = MIN_SPEED;
                    // }
                    right_speed = INIT_SPEED + sensor_delta*INIT_SPEED/(DELTA_THRESHOLD);
                    lcd_cursor(5,1);
                    print_num(right_speed);
                    lcd_cursor(0,1);
                    print_num(left_speed);
                    lcd_cursor(7,1);
                    print_string("L");
                    motor(LEFT_WHEEL_PIN, left_speed);
                    motor(RIGHT_WHEEL_PIN, right_speed);
                }

                
                // _delay_ms(1650);
                _delay_ms(1100);
                // _delay_ms(1000);
            }
        }
        */

        // right sensor is on black tape
        if (right_sensor > MIN_SENSOR_VAL){
            u08 right_sensor_delta = right_sensor-MIN_SENSOR_VAL;
            // limit max speed of right wheel
            if (right_sensor_delta > MAX_SENSOR_VAL-MIN_SENSOR_VAL){
                right_sensor_delta = MAX_SENSOR_VAL-MIN_SENSOR_VAL;
            }
            right_speed = INIT_SPEED - right_sensor_delta*INIT_SPEED/(MAX_SENSOR_VAL-MIN_SENSOR_VAL);
            if (right_speed < MIN_SPEED){
                right_speed = MIN_SPEED;
            }
            motor(RIGHT_WHEEL_PIN, right_speed);
        }
        else {
            motor(RIGHT_WHEEL_PIN, INIT_SPEED);
        }
        if (left_sensor > MIN_SENSOR_VAL){
            u08 left_sensor_delta = left_sensor-MIN_SENSOR_VAL;
            // limit max speed of left wheel
            if (left_sensor_delta > MAX_SENSOR_VAL-MIN_SENSOR_VAL){
                left_sensor_delta = MAX_SENSOR_VAL-MIN_SENSOR_VAL;
            }
            left_speed = INIT_SPEED - left_sensor_delta*INIT_SPEED/(MAX_SENSOR_VAL-MIN_SENSOR_VAL);
            if (left_speed < MIN_SPEED){
                left_speed = MIN_SPEED;
            }
            motor(LEFT_WHEEL_PIN, left_speed);
        }
        else {
            motor(LEFT_WHEEL_PIN, INIT_SPEED);
        }
    }
}

void motor(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    // clear_screen();
    // left wheel
    if (num == LEFT_WHEEL_PIN) {
        servo_speed = 127 + (int8_t) (MOTOR_CAL*speed*127/100);
        set_servo(num, servo_speed);
        // lcd_cursor(0,1);
        // print_num(servo_speed);
    }
    // right wheel
    if (num == RIGHT_WHEEL_PIN) {
        servo_speed = 127 - (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
        // lcd_cursor(5,1);
        // print_num(servo_speed);
    }
    // _delay_ms(200);
    
}