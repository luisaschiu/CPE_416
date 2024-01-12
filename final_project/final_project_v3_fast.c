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
// #define MAX_SPEED 25 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED
#define MAX_SPEED 25
#define CAN_SENSOR_THRESHOLD 85
#define EDGE_SENSOR_THRESHOLD 120

#define TICKS_PER_CM 3.988239 
// #define TICKS_PER_DEGREE 0.588333
// #define MS_PER_DEGREE 22.486
#define MS_PER_DEGREE 15.8 //Value found from trial and error
// #define MS_PER_CM 65.07011
// #define MS_PER_CM 35 //Value found from trial and error
#define RETRACT_MS_PER_CM 70
#define SQUARE_UP_SPEED_POS 100
#define SQUARE_UP_SPEED_NEG -5
#define HALF_SQUARE_CM 39
#define QUARTER_SQUARE_CM 19.5 

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define FORWARD 0 
#define BACKWARD 1
#define RETRACT_DISTANCE_CM 10

#define ROTATE_SPEED_FORWARD 10 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED
#define ROTATE_SPEED_BACKWARD -10 //NOTE: BE CAREFUL WHEN CHANGING THE SPEEDS NOW, THE DELAY WAS DESIGNED AROUND THIS CURRENT SPEED

#define SQUARE_UP_SPEED_POS_A 20
#define SQUARE_UP_SPEED_NEG_A -18
// #define SQUARE_UP_SPEED_POS_L 20
// #define SQUARE_UP_SPEED_NEG_L -10

#define SQUARE_UP_SPEED_POS_L 100
#define SQUARE_UP_SPEED_NEG_L -5

// FAST VALUES
#define MAX_SPEED_FAST 100
#define MS_PER_CM 52

void init_encoder();
void motor(uint8_t, int8_t);
void motor_fast(uint8_t, int8_t);
uint8_t rotate_robot_deg(uint8_t, uint16_t);
uint16_t random_number_between(uint16_t, uint16_t);
uint8_t move_robot_cm(uint8_t, uint8_t);
void push_can_off_edge();
void retract_from_line(uint8_t);
void square_up_Asa(uint8_t, uint8_t);
void square_up_Luisa(uint8_t, uint8_t);


int main(void){
    init();

    // Use current time as seed for random generator 
    srand(time(0)); 

    typedef enum {
        READY,
        SEARCH, // Search for a can
        MOVE_ROTATE_RANDOM, // Move and rotate semi-randomly to search new areas 
        PUSH, // Push the can to the edge
        RETRACT, // Pull away from edge
        HALF_SCAN, // Scan away from edge that was just detected
        RETURN_TO_VANTAGE // Return to a good vantage point and continue searching
    } state_var_type;
    state_var_type state = READY;
    uint8_t result_rotate;
    uint8_t result_move;
    uint16_t random_deg;
    uint8_t random_direction;
    uint16_t random_cm;
    while(1){
        switch(state) {
            case READY:
                clear_screen();
                motor(LEFT_MOTOR, 0);
                motor(RIGHT_MOTOR, 0);
                lcd_cursor(0, 0);
                print_string("READY");
                while(!get_btn());
                state = SEARCH;
                break;
            case SEARCH:
                clear_screen();
                lcd_cursor(0, 0);
                print_string("SEARCH");
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
                clear_screen();
                lcd_cursor(0, 0);
                print_string("MOVE_ROTATE_RANDOM");
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
                clear_screen();
                lcd_cursor(0, 0);
                print_string("PUSH");
                push_can_off_edge();
                state = RETRACT;
                break;
            case RETRACT:
                clear_screen();
                lcd_cursor(0, 0);
                print_string("RETRACT");
                // move_robot_cm(BACKWARD, RETRACT_DISTANCE_CM);
                retract_from_line(RETRACT_DISTANCE_CM);
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
                break;
            case HALF_SCAN:
                clear_screen();
                lcd_cursor(0, 0);
                print_string("HALF_SCAN");
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
                clear_screen();
                lcd_cursor(0, 0);
                print_string("RETURN_TO_VANTAGE");
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
    motor_fast(0, MAX_SPEED_FAST);
    motor_fast(1, MAX_SPEED_FAST);
    while ((left_sensor < EDGE_SENSOR_THRESHOLD) && (right_sensor < EDGE_SENSOR_THRESHOLD)) {
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
    } 
    square_up_Luisa(left_sensor, right_sensor);
    // square_up_Asa(left_sensor, right_sensor);
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
        motor(LEFT_MOTOR, left_motor); 
        motor(RIGHT_MOTOR, right_motor);
        /* Continue rotating */
        // lcd_cursor(0, 0);
        // print_num(count);
        // lcd_cursor(0, 1);
        // print_num(num_ms);
        _delay_ms(1);
        /* Edge Checking */
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        if ((left_sensor > EDGE_SENSOR_THRESHOLD) || (right_sensor > EDGE_SENSOR_THRESHOLD)) {
            // Edge has been detected. Square the robot up against the line, then stop.
            square_up_Luisa(left_sensor, right_sensor);
            // square_up_Asa(left_sensor, right_sensor);
            return 1;
        }
        /* Can checking */
        if (analog(RANGE_SENSOR_PIN) > CAN_SENSOR_THRESHOLD) {
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            lcd_cursor(0,1);
            print_string("Can");
            return 2;
        }
        count ++;
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
    uint16_t count = 0;
    uint16_t num_ms = (u16) (cm * MS_PER_CM);
    /* Stop the robot to get its current encoder value(s). */
    // motor(LEFT_MOTOR, 0);
    // motor(RIGHT_MOTOR, 0);
    // uint16_t cur_right_encoder = right_encoder;
    // float num_ticks = cm * TICKS_PER_CM;

    /* Determine motor speeds based off direction */
    if (direction == FORWARD) {
        left_motor = MAX_SPEED;
        right_motor = MAX_SPEED;
    }
    else {
        left_motor = -MAX_SPEED;
        right_motor = -MAX_SPEED;
    }

    while (count < num_ms) {
        // lcd_cursor(0,0);
        // print_num(count);
        // lcd_cursor(0,1);
        // print_num(num_ms);
        /* Continue moving */
        motor(LEFT_MOTOR, left_motor); 
        motor(RIGHT_MOTOR, right_motor);
        _delay_ms(1);
        /* Edge Checking */
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        if ((left_sensor > EDGE_SENSOR_THRESHOLD) || (right_sensor > EDGE_SENSOR_THRESHOLD)) {
            // Edge has been detected. Square the robot up against the line, then stop.
            square_up_Luisa(left_sensor, right_sensor);
            // square_up_Asa(left_sensor, right_sensor);
            return 1;
        }
        /* Can checking */
        if (analog(RANGE_SENSOR_PIN) > CAN_SENSOR_THRESHOLD) {
            motor(LEFT_MOTOR, 0);
            motor(RIGHT_MOTOR, 0);
            lcd_cursor(0,1);
            print_string("Can");
            return 2;
        }
        count ++;
    }
    return 0;
}

void retract_from_line(uint8_t cm){
    uint8_t left_motor;
    uint8_t right_motor;
    uint16_t count = 0;
    uint16_t num_ms = (u16) (cm * RETRACT_MS_PER_CM);
    left_motor = -MAX_SPEED;
    right_motor = -MAX_SPEED;
    while (count < num_ms) {
        motor(LEFT_MOTOR, left_motor); 
        motor(RIGHT_MOTOR, right_motor);
        _delay_ms(1);
        count++;
    }
    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);
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
        // servo_speed = 127 + (int8_t) (SERVO_0_CAL*speed*127/100 * 0.5);
        servo_speed = 127 + (int8_t) (SERVO_0_CAL*speed*127/100);
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == RIGHT_MOTOR) {
        // servo_speed = 127 - (int8_t) (speed*127/100 * 0.5);
        servo_speed = 127 - (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
    }
}

void motor_fast(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    int8_t servo_delta;
    // left wheel
    
    if (num == LEFT_MOTOR) {
        servo_delta = (int8_t) SERVO_0_CAL*speed*127/100;
        if (servo_delta > 127){
            servo_delta = 127;
        }
        else if (servo_delta < -127){
            servo_delta = -127;
        }
        servo_speed = 127 + servo_delta;
        set_servo(num, servo_speed);
        // lcd_cursor(0,0);
        // print_num(speed);
        // lcd_cursor(0,0);
        // print_num(abs(servo_delta));
        // lcd_cursor(0,1);
        // print_num(servo_speed);
        // _delay_ms(200);
    }
    // right wheel
    if (num == RIGHT_MOTOR) {
        servo_delta = (int8_t) (speed*127/100);
        if (servo_delta < -127){
            servo_delta = -127;
        }
        else if (servo_delta > 127){
            servo_delta = 127;
        }
        servo_speed = 127 - (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
    }
}