/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment Lab 3 part 1
 * Description: Line following proportional controller. Based on Lab2_Part4,
 * this controller is fine-tuned to follow an oval course smoothly without
 * turning the wheels backwards.
*/

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define INIT_SPEED 5
#define SENSOR_READ_DELAY 200

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define LEFT_SENSOR 0
#define RIGHT_SENSOR 1

// #define Kp 1.45
// #define Kp 0.25
// #define Kp 0.2
#define Kp 0.05

#define SERVO_0_CAL 1.25
// #define SERVO_0_CAL 1.1811

#define MIN_SPEED 2
#define MAX_SPEED 100
#define MAX_DELTA 255

struct motor_command {
    uint8_t left;
    uint8_t right;
};

void motor(uint8_t, int8_t);
struct motor_command compute_proportional(uint8_t, uint8_t);


int main(void) {
    init();  //initialize board hardware
    u08 left_sensor;
    u08 right_sensor;

    struct motor_command motor_inst;

    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    _delay_ms(5000);

    while(1) {
        _delay_ms(SENSOR_READ_DELAY);
        left_sensor = analog(LEFT_SENSOR); // left wheel
        right_sensor = analog(RIGHT_SENSOR); // right wheel
        
        motor_inst = compute_proportional(left_sensor, right_sensor);

        motor(LEFT_MOTOR, motor_inst.left);
        motor(RIGHT_MOTOR, motor_inst.right);

        /* testing */
        clear_screen();
        lcd_cursor(0,0);
        print_num(left_sensor);
        lcd_cursor(5,0);
        print_num(right_sensor);
        lcd_cursor(0,1);
        print_num(motor_inst.left);
        lcd_cursor(5,1);
        print_num(motor_inst.right);
    }



    

}

struct motor_command compute_proportional(uint8_t left_sensor, uint8_t right_sensor) {
    int16_t sensor_delta;
    int8_t speed_delta;

    sensor_delta = left_sensor - right_sensor;

    speed_delta = sensor_delta * MAX_SPEED / MAX_DELTA * Kp;

    struct motor_command motor_inst;

    if (INIT_SPEED -speed_delta < MIN_SPEED) {
        motor_inst.left = MIN_SPEED;
    }
    else {
        motor_inst.left = INIT_SPEED -speed_delta;
    }

    if (INIT_SPEED +speed_delta < MIN_SPEED) {
        motor_inst.right = MIN_SPEED;
    }
    else {
        motor_inst.right = INIT_SPEED +speed_delta;
    }

    /*
    if (speed_delta < 0) {
        motor_inst.left = -speed_delta;
        motor_inst.right = 0;
    }
    else if (speed_delta > 0) {
        motor_inst.left = 0;
        motor_inst.right = speed_delta;
    }
    else {
        motor_inst.left = 5;
        motor_inst.right = 5;
        
    }
    */

    return motor_inst;
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


