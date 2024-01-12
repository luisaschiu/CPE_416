#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

#define LEFT_SENSOR_PIN 0
#define RIGHT_SENSOR_PIN 1
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define SERVO_0_CAL 1.1811
#define Kp 0.05

#define INIT_SPEED 5
#define MIN_SPEED 2
#define MAX_SPEED 100
#define MAX_DELTA 255

#define SENSOR_READ_DELAY 200

#define RANGE_SENSOR_PIN 2

#define NUM_PARTICLES 100
#define a_free
#define b_free
#define c_free
#define d_free
#define a_tower
#define b_tower
#define c_tower
#define d_tower
#define LOCATION_TOLERANCE 2 // if a value is less than 2 degrees away

struct particle {
    uint8_t classification; //0 if free space, 1 if tower
    float angle;
    float weight;
};

void motor(uint8_t, int8_t);
u08 probability_free_space(uint16_t);
u08 probability_tower(uint16_t);
void classify_particle(uint8_t, uint16_t[], struct particle);

int main(void) {
    init();  //initialize board hardware

    /**
     * INITIALIZE PARTICLE ARRAY
    */
    struct particle particle_array[NUM_PARTICLES];
    for (uint8_t i = 0; i < NUM_PARTICLES; i++){
        particle_array[i].angle = (360.0/NUM_PARTICLES)*i;
    }

    /**
     * USER INPUT
    */
    uint8_t num_tower;
    uint8_t target_tower;

    /**
     * @todo: replace with user input
    */
    num_tower = 3; // subject to change, tester value for now.
    uint16_t tower_loc[num_tower];
    tower_loc[1] = 0;
    tower_loc[2] = 90;
    tower_loc[3] = 225;
    target_tower 2 // subject to change, tester value for now.

    /**
     * MONTE CARLO LOCALIZATOIN
    */
    
    // encompass in a while loop
    // move robot X ticks
    u16 sensor_val = analog(3); // get sensor value

    // for each particle
    for (uint8_t i = 0; i < NUM_PARTICLES; i ++){
        struct particle current_particle = particle_array[i];
        // move particle X ticks + noise 
        classify_particle(num_tower, tower_loc, current_particle);
        if (current_particle.classification == 0) {     // free space particle
            current_particle.weight = probability_free_space(sensor_val);
        }
        else {                                          // tower particle
            current_particle.weight = probability_tower(sensor_val);
        }
        // resample
    }

    /**
     * MOVE FROM CURRENT LOCATION TO TARGET LOCATION
    */

    /**
     * TURN 90 DEGREES CLOCKWISE
    */

    /**
     * TAKE OUT DARTH VADER!
    */
    motor(LEFT_MOTOR, 100);
    motor(RIGHT_MOTOR, 100);
}

// define which particles are towers or not, based off of given inputs.
void classify_particle(uint8_t num_tower, uint16_t tower_loc[], struct particle p){
    for (uint8_t i = 0; i < num_tower; i++){
        if (abs(p.angle-tower_loc[i];) < LOCATION_TOLERANCE){
            p.classification = 1;
        }
        else {
            p.classification = 0;
        }
    }
}

// Compute probability of particle being a tower or free space
u08 probability_free_space(uint16_t sensor_val){
    u08 P;
    float u_free = 2.0/(d_free + c_free - b_free - a_free);
    if ((sensor_val >= b_free) and (sensor_val < c_free)){
        P = u_free;
    }
    else if ((sensor_val >= a_free) and (sensor_val < b_free)){
        P = u_free *((sensor_val-a_free)/(b_free-a_free));
    }
    else if ((sensor_val >= c_free) and (sensor_val < d_free)){
        P = u_free *((d_free-sensor_val)/(d_free-c_free));
    }
    else{
        P = 0;
    }
    return P;
}

u08 probability_tower(uint16_t sensor_val){
    u08 P;
    float u_tower = 2.0/(d_tower + c_tower - b_tower - a_tower);
    if ((sensor_val >= b_tower) and (sensor_val < c_tower)){
        P = u_tower;
    }
    else if ((sensor_val >= a_tower) and (sensor_val < b_tower)){
        P = u_tower *((sensor_val-a_tower)/(b_tower-a_tower));
    }
    else if ((sensor_val >= c_tower) and (sensor_val < d_tower)){
        P = u_tower *((d_tower-sensor_val)/(d_tower-c_tower));
    }
    else{
        P = 0;
    }
    return P;
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
