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
#define SERVO_0_CAL 1.17
#define SENSOR_READ_DELAY 100
#define MAX_DELTA 255
#define RANGE_SENSOR_PIN 2
#define TICKS_PER_DEG 2

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
void move_deg(uint8_t, int8_t);

int main(void){
    init();
    init_encoder();
    motor(0, 0);
    motor(1, 0);
    _delay_ms(3000);
    while(1){
        // motor(0, 5);
        // motor(1, 5);
        _delay_ms(100);
        clear_screen();
        lcd_cursor(5,0);
        print_num(right_encoder);
        lcd_cursor(0,0);
        print_num(left_encoder);
        motor(0, 25);
        motor(1, 25);
        if (left_encoder >= 200 || right_encoder >= 200){
            motor(0, 0);
            motor(1, 0);
            lcd_cursor(5,0);
            print_num(right_encoder);
            lcd_cursor(0,0);
            print_num(left_encoder);
            break;
        }
        // if (right_encoder >= 100){
        //     motor(0, 0);
        //     motor(1, 0);
        //     break;
        // }
    }
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

// void move_deg(uint8_t deg, int8_t speed, int8_t direction){
//     // convert deg to ticks
//     //
//     u16 ticks = (uint16_t) (TICKS_PER_DEG*deg);
//     while ((left_encoder <= ticks) || right_encoder <= ticks){
//         motor(0, )
//     }
// }



ISR(PCINT0_vect) {

   left_encoder++;  //increment left encoder

}


ISR(PCINT1_vect) {

   right_encoder++;  //increment right encoder

}

