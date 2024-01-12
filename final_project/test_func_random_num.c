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
uint16_t random_number_between(uint16_t, uint16_t);

int main(void){
    init();
    motor(0, 0);
    motor(1, 0);
    srand(time(NULL)); 
    uint16_t random_deg;
    uint8_t random_direction;
    uint16_t random_cm;
    while(1){
        random_deg = random_number_between( 0, 180 );
        random_cm = random_number_between( QUARTER_SQUARE_CM, HALF_SQUARE_CM );
        random_direction = random_number_between( 0, 1 );
        lcd_cursor(0,0);
        print_num(random_deg);
        lcd_cursor(0,1);
        print_num(random_cm);
        lcd_cursor(5,1);
        print_num(random_direction);
        _delay_ms(500);
        clear_screen();
    }
}

uint16_t random_number_between(uint16_t lower_limit, uint16_t upper_limit) {
    uint16_t num = (rand() % (upper_limit - lower_limit + 1)) + lower_limit;
    return num;
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