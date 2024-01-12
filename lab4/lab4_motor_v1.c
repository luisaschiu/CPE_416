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
#define MOTOR_CAL 1.258
#define SENSOR_READ_DELAY 100
#define MAX_DELTA 255
#define RANGE_SENSOR_PIN 2
#define SERVO_0_CAL 1.25


struct motor_command {
    uint8_t left;
    uint8_t right;
};

struct motor_command compute_proportional(uint8_t, uint8_t);
void line_follow(uint8_t, uint8_t);
void init_encoder();

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
    u08 left_sensor;
    u08 right_sensor;
    while(1) {
        left_sensor = analog(LEFT_SENSOR);
        right_sensor = analog(RIGHT_SENSOR);
        motor(LEFT_MOTOR, 0);
        motor(RIGHT_MOTOR, 0);
        line_follow(left_sensor, right_sensor);
        clear_screen();
        lcd_cursor(0,0);
        print_num(right_encoder);
        lcd_cursor(0,1);
        print_num(left_encoder);
        _delay_ms(100);
    }

}
    // while(1){
    //     clear_screen();
    //     lcd_cursor(0,0);
    //     print_num(right_encoder);
    //     lcd_cursor(0,1);
    //     print_num(left_encoder);
    //     _delay_ms(100);
    //     motor(0, 5);
    //     motor(1, 5);
    //     if (right_encoder >= 20){
    //         motor(0, 0);
    //         motor(1, 0);
    //         break;
    //     }
    // }





ISR(PCINT0_vect) {

   left_encoder++;  //increment left encoder

}


ISR(PCINT1_vect) {

   right_encoder++;  //increment right encoder

}



void line_follow(uint8_t left_motor_sensor, uint8_t right_motor_sensor){
    _delay_ms(SENSOR_READ_DELAY);
    
    struct motor_command motor_inst = compute_proportional(left_motor_sensor, right_motor_sensor);

    motor(LEFT_MOTOR, motor_inst.left);
    motor(RIGHT_MOTOR, motor_inst.right);

    /* testing */
    clear_screen();
    lcd_cursor(0,0);
    print_num(left_motor_sensor);
    lcd_cursor(5,0);
    print_num(right_motor_sensor);
    lcd_cursor(0,1);
    print_num(motor_inst.left);
    lcd_cursor(5,1);
    print_num(motor_inst.right);
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

