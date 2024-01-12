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
#define SERVO_0_CAL 1.25
#define MAX_SPEED 25
#define CAN_SENSOR_THRESHOLD 85
#define EDGE_SENSOR_THRESHOLD 120

#define TICKS_PER_CM 3.988239
#define TICKS_PER_DEGREE 0.588333
#define HALF_SQUARE_CM 39
#define QUARTER_SQUARE_CM 19.5 

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define FORWARD 0 
#define BACKWARD 1
#define RETRACT_DISTANCE_CM 10

#define ROTATE_SPEED_FORWARD 7
#define ROTATE_SPEED_BACKWARD -7

volatile uint16_t left_encoder = 0;
volatile uint16_t right_encoder = 0;

void init_encoder();
void motor(uint8_t, int8_t);
uint8_t rotate_robot_deg(uint8_t, uint16_t);
uint16_t random_number_between(uint16_t, uint16_t);
uint8_t move_robot_cm(uint8_t, uint8_t);
void push_can_off_edge();


int main(void){
    init();
    init_encoder();

    // Use current time as seed for random generator 
    srand(time(0)); 

    typedef enum {
        SEARCH, // Search for a can
        MOVE_ROTATE_RANDOM, // Move and rotate semi-randomly to search new areas 
        PUSH, // Push the can to the edge
        RETRACT, // Pull away from edge
        HALF_SCAN, // Scan away from edge that was just detected
        RETURN_TO_VANTAGE // Return to a good vantage point and continue searching
    } state_var_type;
    state_var_type state = SEARCH;
    uint8_t result_rotate;
    uint8_t result_move;
    uint16_t random_deg;
    uint8_t random_direction;
    uint16_t random_cm;
    while(1){
        switch(state) {
            case SEARCH:
                // Rotate 360 degrees searching for cans.
                result_rotate = rotate_robot_deg(CLOCKWISE, 360);
                // If edge is detected, back up.
                if (result_rotate == 1) {
                    state = RETRACT;
                }
                // If can is detected, push it off.
                else if (result_rotate == 2) {
                    state = PUSH;
                }
                // Otherwise, rotate & move randomly, and search again
                else {
                    state = MOVE_ROTATE_RANDOM;
                }
                break;
            case MOVE_ROTATE_RANDOM:

                random_deg = random_number_between( 0, 180 );
                random_cm = random_number_between( QUARTER_SQUARE_CM, HALF_SQUARE_CM );
                random_direction = random_number_between( 0, 1 );

                result_rotate = rotate_robot_deg(random_direction, random_deg);
                if (result_rotate == 1) {
                    state = RETRACT;
                }
                else if (result_rotate == 2) {
                    state = PUSH;
                }
                else {
                    result_move = move_robot_cm(FORWARD, random_cm);
                    if (result_move == 1) {
                        state = RETRACT;
                    }
                    else if (result_move == 2) {
                        state = PUSH;
                    }
                    else {
                        state = SEARCH;
                    }
                }
                break;
            case PUSH:
                push_can_off_edge();
                state = RETRACT;
                break;
            case RETRACT:
                move_robot_cm(BACKWARD, RETRACT_DISTANCE_CM);
                /**
                 * No need to check the result of the move_robot_cm function. The result
                 * will not be 1 since we just backed up from the edge. If the result is 2, 
                 * this is an edge case. If there is a can right after a line was detected,
                 * it is possible that the robot was unable to get the can off.
                 * For now, we will assume this is a false positive and ignore it, 
                 * instead just moving to the half scan state.
                */

                // Face robot parallel to line before half scan
                rotate_robot_deg(CLOCKWISE, 90);
                state = HALF_SCAN;
            case HALF_SCAN:
                result_rotate = rotate_robot_deg(CLOCKWISE, 180);
                if (result_rotate == 1) {
                    state = RETRACT;
                }
                else if (result_rotate == 2) {
                    state = PUSH;
                }
                else {
                    // Face robot back towards centerish of course
                    rotate_robot_deg(COUNTER_CLOCKWISE, 90);
                    state = RETURN_TO_VANTAGE;
                }
            case RETURN_TO_VANTAGE:
                result_move = move_robot_cm(FORWARD, HALF_SQUARE_CM);
                if (result_move == 1) {
                    state = RETRACT;
                }
                else if (result_move == 2) {
                    state = PUSH;
                }
                else {
                    state = SEARCH;
                }
                break;

        }
    }
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
        motor(LEFT_MOTOR, -1);
        motor(RIGHT_MOTOR, 7);
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        clear_screen();
        lcd_cursor(5,0);
        print_num(right_sensor);
        lcd_cursor(0,0);
        print_num(left_sensor);
        _delay_ms(100);
    }
    while (left_sensor < EDGE_SENSOR_THRESHOLD){
        motor(LEFT_MOTOR, 7);
        motor(RIGHT_MOTOR, -1);
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        clear_screen();
        lcd_cursor(5,0);
        print_num(right_sensor);
        lcd_cursor(0,0);
        print_num(left_sensor);
        _delay_ms(100);
    }
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    lcd_cursor(0,1);
    print_string("Square");
}

uint16_t random_number_between(uint16_t lower_limit, uint16_t upper_limit) {
    uint16_t num = (rand() % (upper_limit - lower_limit + 1)) + lower_limit;
    return num;
}

/**
 * Takes an integer number of degrees and rotates the robot the specified angle.
 * Uses a static conversion between degrees and number of ticks on the encoder.
 * @param direction {uint8_t} - Number indicating the rotation direction. 
 * @param degrees {uint_16t} - Number of degrees to rotate the robot
 * Does not account for motion noise.
 * @return an integer representing the success of the rotate command:
 *      0 - successfully rotated without detecting edge/can
 *      1 - stopped movement due to detecting an edge
 *      2 - stopped movement due to detecting a can
*/
uint8_t rotate_robot_deg(uint8_t direction, uint16_t degrees) {
    uint8_t left_sensor;
    uint8_t right_sensor;
    uint8_t left_motor;
    uint8_t right_motor;
    /* Stop the robot to get its current encoder value(s). */
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    /**
     * @todo: decide if we want to use one encoder, or both for rotational movement.
     * My instinct is to use 1, and to use the more reliable one.
    */
    uint16_t cur_right_encoder = right_encoder;
    float num_ticks = degrees * TICKS_PER_DEGREE;

    /* Determine motor speeds based off direction */
    if (direction == CLOCKWISE) {
        left_motor = ROTATE_SPEED_FORWARD;
        right_motor = ROTATE_SPEED_BACKWARD;
    }
    else {
        left_motor = ROTATE_SPEED_BACKWARD;
        right_motor = ROTATE_SPEED_FORWARD;
    }

    while (right_encoder < cur_right_encoder+num_ticks) {
        /* Continue rotating */
        motor(LEFT_MOTOR, left_motor); 
        motor(RIGHT_MOTOR, right_motor);
        /* Edge checking */
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        if ((left_sensor > EDGE_SENSOR_THRESHOLD) || (right_sensor > EDGE_SENSOR_THRESHOLD)) {
            // Edge has been detected. Square the robot up against the line, then stop.
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            clear_screen();
            lcd_cursor(0,0);
            print_string("Edge");
            while (right_sensor < EDGE_SENSOR_THRESHOLD){
                motor(LEFT_MOTOR, -1);
                motor(RIGHT_MOTOR, 7);
                left_sensor = analog(LEFT_LINE_SENSOR);
                right_sensor = analog(RIGHT_LINE_SENSOR);
            }
            while (left_sensor < EDGE_SENSOR_THRESHOLD){
                motor(LEFT_MOTOR, 7);
                motor(RIGHT_MOTOR, -1);
                left_sensor = analog(LEFT_LINE_SENSOR);
                right_sensor = analog(RIGHT_LINE_SENSOR);
            }
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            lcd_cursor(0,1);
            print_string("Square");
            return 1;
        }  
        /* Can checking */
        if (analog(RANGE_SENSOR_PIN) > CAN_SENSOR_THRESHOLD) {
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            return 2;
        }
    }

    return 0;
}

/**
 * Takes an integer number of centimers and moves the robot the specified distance.
 * Uses a static conversion between centimers and number of ticks on the encoder.
 * @param cm {uint_8t} - Number of centimeters to move the robot
 * Does not account for motion noise.
 * @return an integer representing the success of the move command:
 *      0 - successfully moved without detecting edge/can 
 *      1 - stopped movement due to detecting an edge
 *      2 - stopped movement due to detecting a can
*/
uint8_t move_robot_cm(uint8_t direction, uint8_t cm) {
    uint8_t left_sensor;
    uint8_t right_sensor;
    uint8_t left_motor;
    uint8_t right_motor;
    /* Stop the robot to get its current encoder value(s). */
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
    /**
     * @todo: decide if we want to use one encoder, or both for movement.
     * My instinct is to use 1, and to use the more reliable one.
    */
    uint16_t cur_right_encoder = right_encoder;
    float num_ticks = cm * TICKS_PER_CM;

    /* Determine motor speeds based off direction */
    if (direction == FORWARD) {
        left_motor = MAX_SPEED;
        right_motor = MAX_SPEED;
    }
    else {
        left_motor = MAX_SPEED;
        right_motor = MAX_SPEED;
    }

    while (right_encoder < cur_right_encoder+num_ticks) {
        /* Continue moving */
        motor(LEFT_MOTOR, left_motor); 
        motor(RIGHT_MOTOR, right_motor);
        /* Edge checking */
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        if ((left_sensor > EDGE_SENSOR_THRESHOLD) || (right_sensor > EDGE_SENSOR_THRESHOLD)) {
            // Edge has been detected. Square the robot up against the line, then stop.
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            clear_screen();
            lcd_cursor(0,0);
            print_string("Edge");
            while (right_sensor < EDGE_SENSOR_THRESHOLD){
                motor(LEFT_MOTOR, -1);
                motor(RIGHT_MOTOR, 7);
                left_sensor = analog(LEFT_LINE_SENSOR);
                right_sensor = analog(RIGHT_LINE_SENSOR);
            }
            while (left_sensor < EDGE_SENSOR_THRESHOLD){
                motor(LEFT_MOTOR, 7);
                motor(RIGHT_MOTOR, -1);
                left_sensor = analog(LEFT_LINE_SENSOR);
                right_sensor = analog(RIGHT_LINE_SENSOR);
            }
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            lcd_cursor(0,1);
            print_string("Square");
            return 1;
        }  
        /* Can checking */
        if (analog(RANGE_SENSOR_PIN) > CAN_SENSOR_THRESHOLD) {
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            return 2;
        }
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

/* Encoder functions */
void init_encoder() {

    // enable encoder interrupts

    EIMSK = 0;
    EIMSK |= _BV(PCIE1) | _BV(PCIE0);

    PCMSK1 |= _BV(PCINT13); //PB5 - digital 5
    PCMSK0 |= _BV(PCINT6);  //PE6 - digital 4

    // enable pullups

    PORTE |= _BV(PE6);
    PORTB |= _BV(PB5);
}

ISR(PCINT0_vect) {

   left_encoder++;  //increment left encoder

}


ISR(PCINT1_vect) {

   right_encoder++;  //increment right encoder

}