#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

#define RANGE_SENSOR_PIN 3
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define RANGE_MOTOR 2
#define SERVO_0_CAL 1.25

void motor(uint8_t, int8_t);
void range_scan(int8_t, int8_t);
void delay_ms_variable(u08);

int main(void) {
    init(); 
    u16 sensor_val;
    motor(0,0);
    motor(1,0);
    motor(RANGE_MOTOR, 0);
    _delay_ms(200);
    while(1){
        sensor_val = analog(RANGE_SENSOR_PIN);
        // _delay_ms(1);
        // clear_screen();
        // range_scan(100, 200);
       if (sensor_val > 90){
            motor(RANGE_MOTOR, 0);
            break;
       }
        motor(RANGE_MOTOR, 100);
        // // set_servo(RANGE_MOTOR, 190);
        _delay_ms(500);
        motor(RANGE_MOTOR, -100);
        _delay_ms(500);
        // sensor_val = analog(RANGE_SENSOR_PIN);
        // lcd_cursor(0,0);
        // print_num(sensor_val);
        // _delay_ms(200);
        // clear_screen();
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
    if (num == RANGE_MOTOR) {
        servo_speed = 127 + (int8_t) (SERVO_0_CAL*speed*127/100 * 0.5);
        set_servo(num, servo_speed);
    }
}

void range_scan(int8_t speed, int8_t scan_frequency){ //scan_freq in ms
    // motor(RANGE_MOTOR, 100);
    // static unsigned long pulse = 0;
    u16 sensor_val;
    for (u16 i=0; i<=500; i += 1) {
        sensor_val = analog(RANGE_SENSOR_PIN);
        lcd_cursor(0,0);
        print_num(sensor_val);
        motor(RANGE_MOTOR, speed);
        _delay_ms(1);
        clear_screen();
        if (sensor_val > 90){
            motor(RANGE_MOTOR, 0);
            return;
        }
    }
    for (u16 i=0; i<=500; i += 1) {
        sensor_val = analog(RANGE_SENSOR_PIN);
        lcd_cursor(0,0);
        print_num(sensor_val);
        motor(RANGE_MOTOR, -speed);
        _delay_ms(1);
        clear_screen();
        if (sensor_val > 90){
            motor(RANGE_MOTOR, 0);
            return;
        }
    }
    // if (sensor_val > 90){
    //         motor(RANGE_MOTOR, 0);
    //         return;
    // }
}

void delay_ms_variable(u08 ms) {
   for (u08 i = 0; i < ms; i++) {
      _delay_ms(1);
   }
}