#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

#define RANGE_SENSOR_PIN 3
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define SERVO_0_CAL 1.25

void motor(uint8_t, int8_t);

int main(void) {
    init(); 
    u16 sensor_val;
    motor(0,0);
    motor(1,0);
    while(1){
        sensor_val = analog(RANGE_SENSOR_PIN);
        lcd_cursor(0,0);
        print_num(sensor_val);
        _delay_ms(200);
        clear_screen();
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