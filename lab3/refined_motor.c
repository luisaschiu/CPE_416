#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
void motor(uint8_t, int8_t);

int main(void) {
    init();  //initialize board hardware
    int8_t motor_speed;
    motor(0, 0);
    motor(1, 0);

    for (motor_speed=0; motor_speed <= 100; motor_speed += 10){
        clear_screen();
        print_num(motor_speed);
        motor(0, motor_speed);
        motor(1, motor_speed);
        _delay_ms(750);
    }
    for (motor_speed=100; motor_speed >= 0; motor_speed -= 10){
        clear_screen();
        print_num(motor_speed);
        motor(0, motor_speed);
        motor(1, motor_speed);
        _delay_ms(750);
    }
    for (motor_speed=0; motor_speed >= -100; motor_speed -= 10){
        clear_screen();
        lcd_cursor(0,0);
        if (motor_speed != 0){
            print_string("-");
            lcd_cursor(1,0);
        }
        print_num(abs(motor_speed));
        motor(0, motor_speed);
        motor(1, motor_speed);
        _delay_ms(750);
    }
    for (motor_speed=-100; motor_speed <= 0; motor_speed += 10){
        clear_screen();
        lcd_cursor(0,0);
        if (motor_speed != 0){
            print_string("-");
            lcd_cursor(1,0);
        }
        print_num(abs(motor_speed));
        motor(0, motor_speed);
        motor(1, motor_speed);
        _delay_ms(750);
    }
}


void motor(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    // left wheel
    if (num == 0) {
        servo_speed = ( 127 + (int8_t) (speed*127/100 * 0.5) );
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == 1) {
        servo_speed = ( 127 - (int8_t) (speed*127/100 * 0.5) );
        set_servo(num, servo_speed);
    }
}
