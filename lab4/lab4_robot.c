#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define NUM_PARTICLES 100
// #define a_free  0 
// #define b_free  20
// #define c_free  50
// #define d_free  70
// #define a_tower 60
// #define b_tower 90
// #define c_tower 110
// #define d_tower 120
#define a_free  20
#define b_free  30
#define c_free  70
#define d_free  100
#define a_tower 95
#define b_tower 150
#define c_tower 170
#define d_tower 184
#define LOCATION_TOLERANCE 8.67 // if a value is less than 8.67 degrees away
#define ANGLE_THRESHOLD 45
#define THRESHOLD_PERCENTAGE 0.25
#define SIMULATED_SENSOR_TOWER 100
#define SIMULATED_SENSOR_FREE 35

#define MEAN 3 // 1 TICK = 1.55 DEGREES
#define STD_DEV 0.3

#define STD_DEV_THRESHOLD 5

#define INIT_SPEED 5
#define SENSOR_READ_DELAY 200
#define RANGE_SENSOR_PIN 2
#define LEFT_LINE_SENSOR 0
#define RIGHT_LINE_SENSOR 1
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define Kp 0.05
#define SERVO_0_CAL 1.25
#define MIN_SPEED 2
#define MAX_SPEED 100
#define MAX_DELTA 255


volatile uint16_t left_encoder = 0;
volatile uint16_t right_encoder = 0;

struct particle {
    uint8_t classification; //0 if free space, 1 if tower
    float angle;
    float weight;
};

struct motor_command {
    uint8_t left;
    uint8_t right;
};


uint8_t get_simulated_sensor_value(float, uint8_t, uint16_t[]);
float probability_free_space(uint8_t);
float probability_tower(uint8_t);
void normalize_weights(struct particle[]);
void resample2(struct particle[]);
void classify_particle(uint8_t, uint16_t[], struct particle*);
void update_particle_weight(struct particle*, uint8_t);
float min_diff_between_angles(float, float);
float calculate_std_dev(uint8_t, struct particle[]);
float box_muller_transform(void);
float calculate_std_dev(uint8_t, struct particle[]);
void init_encoder(void);

void motor(uint8_t, int8_t);
void line_follow(uint8_t, uint8_t);
struct motor_command compute_proportional(uint8_t, uint8_t);
float get_mean(uint8_t, struct particle[]);
void move_robot_two_ticks();

int main() {
    init();
    init_encoder();
    /* initialize tower & robot location info */
    uint8_t num_towers = 3;
    uint8_t target_tower = 2;
    uint16_t tower_loc[] = { 0, 90, 225 };
    // uint16_t tower_loc[] = { 0, 45, 90 };
    // float robot_position = 42;
    // GOOD: 225, 2, 20
    // BAD: 224
    float std_dev;
    uint8_t sensor_val;

    struct particle particle_array[NUM_PARTICLES];
    for (uint8_t i = 0; i < NUM_PARTICLES; i++){
        particle_array[i].angle = (360.0/NUM_PARTICLES)*i;
    }

    /* localization algorithm */
    while(1) {
        move_robot_two_ticks();
        motor(LEFT_MOTOR, 0);
        motor(RIGHT_MOTOR, 0);
        sensor_val = analog(RANGE_SENSOR_PIN);

        // for each particle, move it, classify it, then update its weight
        for (uint8_t i = 0; i < NUM_PARTICLES; i ++){
            struct particle* current_particle_ptr = &(particle_array[i]);

            // move particle MEAN degrees
            current_particle_ptr->angle += MEAN;
            // add gaussian noise
            current_particle_ptr->angle += STD_DEV*box_muller_transform();
            // bound check angle
            if (current_particle_ptr->angle > 360.0) {
                current_particle_ptr->angle -= 360.0;
            }

            classify_particle(num_towers, tower_loc, current_particle_ptr);
            update_particle_weight(current_particle_ptr, sensor_val);
        }

        /* normalize weights */
        normalize_weights(particle_array);
        /* resample particles */
        resample2(particle_array);

        /* calculate std dev */
        std_dev = calculate_std_dev(NUM_PARTICLES, particle_array);
        if (std_dev < STD_DEV_THRESHOLD) {
            break;
        }
        /* add 5% random particles */
        for (uint8_t i = 0; i < 0.03 * NUM_PARTICLES; i++) {
            // generate a random angle between 0 & 360
            float random_angle = ( (float) rand() ) / ( (float) RAND_MAX ) * 360.0;
            uint8_t random_idx = (uint8_t) (( (float) rand() ) / ( (float) RAND_MAX ) * 99);
            struct particle p = 
            {
                .angle = random_angle,
                .classification = 0, // unimportant, will get updated in the next iteration of algorithm
                .weight = 0.0 // unimportant, will get updated in the next iteration of algorithm
            };
            particle_array[random_idx] = p;
        }

    }
    uint16_t mean = (uint16_t) get_mean(NUM_PARTICLES, particle_array);
    
    clear_screen();
    print_string("LOCALIZED");
    lcd_cursor(0, 1);
    print_num(mean);

    return 0;
}

float get_mean(uint8_t size, struct particle particle_array[]) {
    float mean_x;
    float mean_y;
    float mean_degrees;
    float total_x = 0;
    float total_y = 0;

    float angle_rad;
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        angle_rad = particle_array[i].angle * M_PI / 180.0;
        total_x += cos(angle_rad);
        total_y += sin(angle_rad);
    }
    mean_x = total_x/size;
    mean_y = total_y/size;
    mean_degrees = atan2(mean_y, mean_x) * 180.0 / M_PI;
    mean_degrees += 360;
    while (mean_degrees > 360) {
        mean_degrees -= 360;
    }

    return mean_degrees;
}

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

void move_robot_two_ticks() {
    uint16_t cur_encoder = right_encoder;
    uint8_t left_sensor;
    uint8_t right_sensor;
    while (right_encoder < cur_encoder+2) {
        left_sensor = analog(LEFT_LINE_SENSOR);
        right_sensor = analog(RIGHT_LINE_SENSOR);
        line_follow(left_sensor, right_sensor);
    }
}

float box_muller_transform(void) {
    float u1 = ( (float) rand() ) / ( (float) RAND_MAX );
    float u2 = ( (float) rand() ) / ( (float) RAND_MAX );
    float sq = (float) sqrt(-2.0*log(u1));
    float co = (float) cos(2.0*M_PI*u2);
    float z = sq*co;
    return z;
}

/**
 * @todo: add in 5% random particles
*/
void resample2(struct particle particle_array[]) {
    struct particle resampled_particle_array[NUM_PARTICLES];
    float weight_bucket = 0.0;
    float counter_bucket = 0.0;
    uint8_t idx_current = 0;
    float counter_increment = 1.0 / NUM_PARTICLES;
    uint8_t idx_resample = 0;

    // Loop until resample array is filled with NUM_PARTICLES
    // for (uint8_t idx_resample = 0; idx_resample < NUM_PARTICLES; idx_resample++) {
    while (idx_resample < NUM_PARTICLES) {
        // if (idx_current >= NUM_PARTICLES) {
        //     exit(-1);
        // }
        // add current particle's weight to weight bucket
        struct particle current_particle = particle_array[idx_current];
        weight_bucket += current_particle.weight;
        idx_current++;

        // Add number of particles relative to its weight
        while (weight_bucket > counter_bucket) {
            struct particle copy = current_particle; // make a copy of the current particle 
            if (idx_resample >= NUM_PARTICLES) {                    
                break;                                          // break out of algorithm if resampled array has reached 100 particles
            }
            resampled_particle_array[idx_resample] = copy;      // add the copy to the resample array
            idx_resample++;
            counter_bucket += counter_increment;                // increment counter_bucket by counter
        }
    }

    // /* add 5% random particles */
    // for (uint8_t i = 0; i < 0.05 * NUM_PARTICLES; i++) {
    //     // generate a random angle between 0 & 360
    //     float random_angle = ( (float) rand() ) / ( (float) RAND_MAX ) * 360.0;
    //     struct particle p = 
    //     {
    //         .angle = random_angle,
    //         .classification = 0, // unimportant, will get updated in the next iteration of algorithm
    //         .weight = 0.0 // unimportant, will get updated in the next iteration of algorithm
    //     };
    //     resampled_particle_array[i] = p;
    // }

    memcpy(particle_array, resampled_particle_array, sizeof(resampled_particle_array));
    // particle_array = resampled_particle_array;
}

// define which particles are towers or not, based off of given inputs.
void classify_particle(uint8_t num_tower, uint16_t tower_loc[], struct particle* particle_ptr){
    for (uint8_t i = 0; i < num_tower; i++){
        if (min_diff_between_angles(particle_ptr->angle, (float) tower_loc[i]) < LOCATION_TOLERANCE){
            particle_ptr->classification = 1;
            return;
        }
        else {
            particle_ptr->classification = 0;
        }
    }
}

float min_diff_between_angles(float a1, float a2) {
    if (a1 > 360.0 || a1 < 0.0) {
        clear_screen();
        print_string("ERROR A1");
        return(-1);
    }

    if (a2 > 360.0 || a2 < 0.0) {
        clear_screen();
        print_string("ERROR A2");
        return(-1);
    }

    float large_angle;
    float small_angle;
    float min_diff;
    if (a2 > a1) {
        large_angle = a2;
        small_angle = a1;
    }
    else {
        large_angle = a1;
        small_angle = a2;
    }
    min_diff = large_angle - small_angle;

    if (min_diff > 180.0) {
        min_diff = (360 - large_angle) + small_angle;
    }

    return min_diff;
}

void update_particle_weight(struct particle* particle_ptr, uint8_t sensor_val) {
    if (particle_ptr->classification == 0) {     // free space particle
        particle_ptr->weight = probability_free_space(sensor_val);
    }
    else {                                          // tower particle
        particle_ptr->weight = probability_tower(sensor_val);
    }
}

// Compute probability of particle being a tower or free space
float probability_free_space(uint8_t sensor_val){
    float P;
    float u_free = 2.0/ (float) ((d_free + c_free - b_free - a_free));
    if ((sensor_val >= b_free) && (sensor_val < c_free)){
        P = u_free;
    }
    else if ((sensor_val >= a_free) && (sensor_val < b_free)){
        P = u_free * ((float) (sensor_val-a_free))/ ((float) (b_free-a_free));
    }
    else if ((sensor_val >= c_free) && (sensor_val < d_free)){
        P = u_free * ((float) (d_free-sensor_val)) / ((float) (d_free-c_free));
    }
    else{
        P = 0;
    }
    if (P > 1 || P < 0) {
        clear_screen();
        print_string("PROBLEM");
        lcd_cursor(0, 1);
        print_string("P FREE");
        return(-2);
    }
    return P;
}

float probability_tower(uint8_t sensor_val){
    float P;
    float u_tower = 2.0/ ((float) (d_tower + c_tower - b_tower - a_tower));
    if ( ((float) (sensor_val >= b_tower)) && ((float) (sensor_val < c_tower))){
        P = u_tower;
    }
    else if ((sensor_val >= a_tower) && (sensor_val < b_tower)){
        P = u_tower * ((float) (sensor_val-a_tower)) /( (float) (b_tower-a_tower));
    }
    else if ( (sensor_val >= c_tower) && (sensor_val < d_tower) ){
        P = u_tower * ((float) (d_tower-sensor_val)) / ( (float) (d_tower-c_tower));
    }
    else{
        P = 0;
    }
    if (P > 1 || P < 0) {
        clear_screen();
        print_string("PROBLEM");
        lcd_cursor(0, 1);
        print_string("P FREE");
        return(-2);
    }
    return P;
}

/**
 * @todo: ensure this is working correctly.
 * may require using pointers to directly access the structs
*/
void normalize_weights(struct particle particle_array[]) {
    float sum_weights = 0.0;

    // calculate total weights
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        sum_weights += particle_array[i].weight;
    }

    // normalize each weight
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        particle_array[i].weight = particle_array[i].weight / sum_weights;
    }
    
}

float calculate_std_dev(uint8_t size, struct particle particle_array[]) {
    float mean_x;
    float mean_y;
    float mean_degrees;
    float total_x = 0;
    float total_y = 0;
    float std_diff_squared = 0;
    float std_dev;

    float angle_rad;
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        angle_rad = particle_array[i].angle * M_PI / 180.0;
        total_x += cos(angle_rad);
        total_y += sin(angle_rad);
    }
    mean_x = total_x/size;
    mean_y = total_y/size;
    mean_degrees = atan2(mean_y, mean_x) * 180.0 / M_PI;
    mean_degrees += 360;
    while (mean_degrees > 360) {
        mean_degrees -= 360;
    }

    // loop through to calculate values for standard deviation
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        std_diff_squared += pow((min_diff_between_angles(mean_degrees, particle_array[i].angle)),2);
    }
    std_dev = (float) sqrt(std_diff_squared/size); // calculate standard deviation
    return std_dev;
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

    return motor_inst;
}

ISR(PCINT0_vect) {

   left_encoder++;  //increment left encoder

}


ISR(PCINT1_vect) {

   right_encoder++;  //increment right encoder

}
