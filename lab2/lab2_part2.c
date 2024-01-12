/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment: Lab 2 part 2
 * Description: This program implements Braitenberg vehicles 2a(fear) and 2b(aggression) using two photoresistors; pressing the on-board button toggles between the 2 vehicles.
 * such that pressing the on-board button toggles between the two vehicles.
*/

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEFT_WHEEL_PIN 0
#define RIGHT_WHEEL_PIN 1
#define LEFT_SENSOR 0
#define RIGHT_SENSOR 1
void motor(uint8_t, int8_t);

int main(void) {
    init();  //initialize board hardware
    
    // turn motors off initially
    motor(LEFT_WHEEL_PIN, 0);
    motor(RIGHT_WHEEL_PIN, 0);

    int8_t left_speed;
    int8_t right_speed;
    typedef enum {
        FEAR_2A,
        AGGRESSION_2B
    } state_var_type;

    state_var_type state = FEAR_2A;
    while (1){
        switch(state) {
            case FEAR_2A:
                left_speed = (int8_t) (analog(LEFT_SENSOR)*100/255);
                motor(LEFT_WHEEL_PIN, left_speed);
                right_speed = (int8_t) (analog(RIGHT_SENSOR)*100/255);
                motor(RIGHT_WHEEL_PIN, right_speed);

                /**
                 * testing
                */
                clear_screen();
                lcd_cursor(0,0);
                print_string("2A");
                lcd_cursor(0,1);
                print_num(left_speed);
                lcd_cursor(5,1);
                print_num(right_speed);
                _delay_ms(200);

                if (get_btn()){
                    state = AGGRESSION_2B;
                }
                else {
                    state = FEAR_2A;
                }
                break;
            case AGGRESSION_2B:
                left_speed = (int8_t) (analog(RIGHT_SENSOR)*100/255);
                motor(LEFT_WHEEL_PIN, left_speed);
                right_speed = (int8_t) (analog(LEFT_SENSOR)*100/255);
                motor(RIGHT_WHEEL_PIN, right_speed);

                /**
                 * testing
                */
                clear_screen();
                lcd_cursor(0,0);
                print_string("2B");
                lcd_cursor(0,1);
                print_num(left_speed);
                lcd_cursor(5,1);
                print_num(right_speed);
                _delay_ms(200);

                if (get_btn()){
                    state = FEAR_2A;
                }
                else {
                    state = AGGRESSION_2B;
                }
                break;
        }
    }
}

void motor(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    // left wheel
    if (num == 0) {
        servo_speed = 127 + (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == 1) {
        servo_speed = 127 - (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
    }
}