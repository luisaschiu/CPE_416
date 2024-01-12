#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>


#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_SENSOR 0
#define RIGHT_SENSOR 1
#define INIT_SPEED 5
#define MIN_SPEED -20
#define MAX_SPEED 15
#define Kp 1.45
// #define MOTOR_CAL 1.1075
#define MOTOR_CAL 1.1
#define SENSOR_READ_DELAY 100
#define MAX_DELTA 255
#define RANGE_SENSOR_PIN 2

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

volatile uint16_t left_encoder = 0;
volatile uint16_t right_encoder = 0;

void motor(uint8_t, int8_t);

int main(void){
    init();
    init_encoder();
    motor(0, 0);
    motor(1, 0);
    _delay_ms(3000);
    // Clockwise
    // L 129 R 69
    // Counter clockwise
    // L 118 R 73
    // L 118 R 83
    // L 118 R 74
    // L 116 R 71
    // L 116 R 73
    while(1){
        if (get_btn())
            break;
    }
    print_string("start");
    _delay_ms(3000);
    motor(0, -5);
    motor(1, 5);
    while(1){
        if (get_btn())
            break;
        clear_screen();
        lcd_cursor(0,0);
        print_string("L");
        lcd_cursor(2,0);
        print_num(left_encoder);
        lcd_cursor(0,1);
        print_string("R");
        lcd_cursor(2,1);
        print_num(right_encoder);
        _delay_ms(100);
        if (right_encoder >= 211){
            motor(0, 0);
            motor(1, 0);
            break;
        }
    }
    clear_screen();
    lcd_cursor(0,0);
    print_string("L");
    lcd_cursor(2,0);
    print_num(left_encoder);
    lcd_cursor(0,1);
    print_string("R");
    lcd_cursor(2,1);
    print_num(right_encoder);
    
    // _delay_ms(100);
}

void motor(uint8_t num, int8_t speed) {
    uint8_t servo_speed;
    if (num == LEFT_MOTOR) {
        servo_speed = 127 + (int8_t) (MOTOR_CAL*speed*127/100);
        set_servo(num, servo_speed);
    }
    // right wheel
    if (num == RIGHT_MOTOR) {
        servo_speed = 127 - (int8_t) (speed*127/100);
        set_servo(num, servo_speed);
    }
    
}





ISR(PCINT0_vect) {

   left_encoder++;  //increment left encoder

}


ISR(PCINT1_vect) {

   right_encoder++;  //increment right encoder

}

