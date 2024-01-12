#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEFT_WHEEL_PIN 0
#define RIGHT_WHEEL_PIN 1
#define LEFT_SENSOR_PIN 0
#define RIGHT_SENSOR_PIN 1
#define MIN_SENSOR_VAL 20
#define MAX_SENSOR_VAL 204
#define INIT_SPEED 5
#define MIN_SPEED 0
#define MOTOR_CAL 1.1811
#define MIN_SENSOR_DELTA 0
#define MAX_SENSOR_DELTA 184
#define DELTA_THRESHOLD 170
#define DELTA_LOWER_THRESHOLD 10
void motor(uint8_t, int8_t);

int main(void) {
    init();  //initialize board hardware
    u08 left_sensor;
    u08 right_sensor;
    int8_t left_speed = INIT_SPEED;
    int8_t right_speed = INIT_SPEED;
    u08 sensor_delta;
    u08 speed_delta;

    motor(LEFT_WHEEL_PIN, 0);
    motor(RIGHT_WHEEL_PIN, 0);
    _delay_ms(5000);
    
    /*
    while (1){
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

        // int16_t sensor_delta = right_sensor - left_sensor;

        if (right_sensor > left_sensor) {
            sensor_delta = right_sensor - left_sensor;
            if (sensor_delta < MIN_SENSOR_DELTA) {
                motor(LEFT_WHEEL_PIN, INIT_SPEED);
                motor(RIGHT_WHEEL_PIN, INIT_SPEED);
            }
            
            else {

                /* removing this bc impossible
                if (sensor_delta < MIN_SENSOR_DELTA) {
                    sensor_delta = MIN_SENSOR_DELTA;
                }
                */
                // limit delta to expected range
                if (sensor_delta > MAX_SENSOR_DELTA) {
                    sensor_delta = MAX_SENSOR_DELTA;
                }

                speed_delta = sensor_delta*INIT_SPEED/MAX_SENSOR_DELTA;

                /* alternative
                // if sensor_delta is below threshold, just slow right wheel.
                if (sensor_delta < DELTA_THRESHOLD) {
                    motor(RIGHT_WHEEL_PIN, INIT_SPEED - speed_delta);
                } 

                // if sensor_delta is above threshold, slow right wheel AND speed up left wheel
                else if (sensor_delta >= DELTA_THRESHOLD) {
                    motor(RIGHT_WHEEL_PIN, INIT_SPEED - speed_delta);
                    motor(LEFT_WHEEL_PIN, INIT_SPEED + speed_delta);
                }
                */

                // if sensor_delta is above threshold, slow right wheel AND speed up left wheel
                if (sensor_delta >= DELTA_THRESHOLD) {
                    // motor(LEFT_WHEEL_PIN, INIT_SPEED + speed_delta);
                    motor(LEFT_WHEEL_PIN, INIT_SPEED);
                }
                else {
                    motor(LEFT_WHEEL_PIN, INIT_SPEED);
                }

                right_speed = 0 - speed_delta;
                // if (right_speed < MIN_SPEED) {
                //     right_speed = MIN_SPEED;
                // }
                motor(RIGHT_WHEEL_PIN, right_speed);

                /* alternative
                if (INIT_SPEED - speed_delta < MIN_SPEED) {
                    motor(RIGHT_WHEEL_PIN, MIN_SPEED);
                }
                else {
                    motor(RIGHT_WHEEL_PIN, INIT_SPEED - speed_delta);
                }
                */
            }
        }

        if (left_sensor > right_sensor) {
            sensor_delta = left_sensor - right_sensor;
            if (sensor_delta < MIN_SENSOR_DELTA) {
                motor(LEFT_WHEEL_PIN, INIT_SPEED);
                motor(RIGHT_WHEEL_PIN, INIT_SPEED);
            }
            else {

                /* removing this bc impossible
                if (sensor_delta < MIN_SENSOR_DELTA) {
                    sensor_delta = MIN_SENSOR_DELTA;
                }
                */
                // limit delta to expected range
                if (sensor_delta > MAX_SENSOR_DELTA) {
                    sensor_delta = MAX_SENSOR_DELTA;
                }

                speed_delta = sensor_delta*INIT_SPEED/MAX_SENSOR_DELTA;

                // if sensor_delta is above threshold, slow left wheel AND speed up right wheel
                if (sensor_delta >= DELTA_THRESHOLD) {
                    // motor(RIGHT_WHEEL_PIN, INIT_SPEED + speed_delta);
                    motor(RIGHT_WHEEL_PIN, INIT_SPEED);
                }
                else {
                    motor(RIGHT_WHEEL_PIN, INIT_SPEED);
                }

                left_speed = 0 - speed_delta;
                // if (left_speed < MIN_SPEED) {
                //     left_speed = MIN_SPEED;
                // }
                motor(LEFT_WHEEL_PIN, left_speed);
            }
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