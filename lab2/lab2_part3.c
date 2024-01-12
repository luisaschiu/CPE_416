/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment: Lab 2 part 3
 * Description: This program implements Braitenberg vehicles 3a(attraction) and 3b(shy) using two photoresistors; pressing the on-board button toggles between the 2 vehicles.
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
        ATTRACTION_3A,
        SHY_3B
    } state_var_type;

    state_var_type state = ATTRACTION_3A;
    while (1){
        switch(state) {
            case ATTRACTION_3A:
                
                left_speed = 100 - (int8_t) (analog(LEFT_SENSOR)*100/255);
                motor(LEFT_WHEEL_PIN, left_speed);
                right_speed = 100 - (int8_t) (analog(RIGHT_SENSOR)*100/255);
                motor(RIGHT_WHEEL_PIN, right_speed);
                
                /**
                 * testing
                */
                clear_screen();
                lcd_cursor(0,0);
                print_string("3A");
                lcd_cursor(0,1);
                print_num(left_speed);
                lcd_cursor(5,1);
                print_num(right_speed);
                _delay_ms(200);


                if (get_btn()){
                    state = SHY_3B;
                }
                else {
                    state = ATTRACTION_3A;
                }
                break;
            case SHY_3B:
                left_speed = 100 - (int8_t) (analog(RIGHT_SENSOR)*100/255);
                motor(LEFT_WHEEL_PIN, left_speed);
                right_speed = 100 - (int8_t) (analog(LEFT_SENSOR)*100/255);
                motor(RIGHT_WHEEL_PIN, right_speed);

                /**
                 * testing
                */
                clear_screen();
                lcd_cursor(0,0);
                print_string("3B");
                lcd_cursor(0,1);
                print_num(left_speed);
                lcd_cursor(5,1);
                print_num(right_speed);
                _delay_ms(200);

                if (get_btn()){
                    state = ATTRACTION_3A;
                }
                else {
                    state = SHY_3B;
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