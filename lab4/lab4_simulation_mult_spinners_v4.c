/**
 * v5 includes the resampling function using cos, sin, & atan2
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define NUM_PARTICLES 100
#define a_free  0 
#define b_free  20
#define c_free  50
#define d_free  70
#define a_tower 60
#define b_tower 90
#define c_tower 110
#define d_tower 120
#define LOCATION_TOLERANCE 8.67 // if a value is less than 8.67 degrees away
#define ANGLE_THRESHOLD 45
#define THRESHOLD_PERCENTAGE 0.25
#define SIMULATED_SENSOR_TOWER 100
#define SIMULATED_SENSOR_FREE 35

#define MEAN 3
#define STD_DEV 0.3

#define STD_DEV_THRESHOLD 5

struct particle {
    uint8_t classification; //0 if free space, 1 if tower
    float angle;
    float weight;
};

uint8_t get_simulated_sensor_value(float, uint8_t, uint16_t[]);
float probability_free_space(uint8_t);
float probability_tower(uint8_t);
void normalize_weights(struct particle[]);
void resample_particles(struct particle[]);
void print_particle_array(struct particle[], uint8_t);
void classify_particle(uint8_t, uint16_t[], struct particle*);
void update_particle_weight(struct particle*, uint8_t);
float min_diff_between_angles(float, float);
float calculate_std_dev(uint8_t, struct particle[]);
float box_muller_transform(void);
void print_particle_array_weights(struct particle[], uint8_t);
float calculate_std_dev(uint8_t, struct particle[]);

int main() {
    /* initialize tower & robot location info */
    uint8_t num_towers = 3;
    uint8_t target_tower = 2;
    uint16_t tower_loc[] = { 0, 90, 225 };
    float robot_position = 225.0;
    float std_dev;
    uint8_t sensor_val;

    /* initialize fake particles */
    // struct particle p0 = {0, 5, 0.1};
    // struct particle p1 = {0, 10, 0.0};
    // struct particle p2 = {0, 20, 0.2};
    // struct particle p3 = {0, 30, 0.3};
    // struct particle p4 = {0, 40, 0.4};
    // struct particle particle_array[NUM_PARTICLES] = {p0, p1, p2, p3, p4};
    struct particle particle_array[NUM_PARTICLES];
    for (uint8_t i = 0; i < NUM_PARTICLES; i++){
        particle_array[i].angle = (360.0/NUM_PARTICLES)*i;
    }

    /* localization algorithm */
    while(1) {
        srand(time(NULL));
        // move robot X ticks + (no simulated noise. assuming no motion noise)
        robot_position += MEAN;
        // restrict robot_position between 0 & 360 (inclusive)
        if (robot_position > 360.0) {
            robot_position -= 360.0;
        }
        sensor_val = get_simulated_sensor_value(robot_position, num_towers, tower_loc);

        /*
        printf("INITIAL\n");
        print_particle_array(particle_array, NUM_PARTICLES);
        */

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
        printf("ROBOT @ %.f DEGREES\n", robot_position);
        /*
        printf("AFTER MOVING, CLASSIFYING, & CALCULATING WEIGHTS OF PARTICLES\n");
        print_particle_array(particle_array, NUM_PARTICLES);
        */
       
        // /* classify particles */
        // for (uint8_t i = 0; i < NUM_PARTICLES; i ++) {
        //     classify_particle(num_towers, tower_loc, &(particle_array[i]));
        // }
        // printf("AFTER CLASSIFICATION\n");
        // print_particle_array(particle_array, NUM_PARTICLES);

        // /* calculate weights */
        // for (uint8_t i = 0; i < NUM_PARTICLES; i ++) {
        //     update_particle_weight(&(particle_array[i]), sensor_val);
        // }
        // printf("AFTER CALCULATING WEIGHTS\n");
        // print_particle_array(particle_array, NUM_PARTICLES);

        /* normalize weights */
        normalize_weights(particle_array);
        /*
        printf("AFTER NORMALIZING WEIGHTS\n");
        print_particle_array(particle_array, NUM_PARTICLES);
        */

        /* resample particles */
        resample_particles(particle_array);
        // printf("AFTER RESAMPLING\n");
        print_particle_array(particle_array, NUM_PARTICLES);
        /* calculate std dev */
        std_dev = calculate_std_dev(NUM_PARTICLES, particle_array);
        printf("STD DEV: %.4f\n", std_dev);

        if (std_dev < STD_DEV_THRESHOLD) {
            break;
        }
    }
    printf("ROBOT @ %.6f degrees", robot_position);
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
 * This function returns a simulated sensor value based on what the range sensor might read.
 * For simplicity, if the robot is within the tolerance of a tower location, the function returns the 
 * center value of the trapezoid PDF for the tower, defined by SIMULATED_SENSOR_TOWER.
 * Otherwise, the function returns the center value of the trapezoid PDF for free space, 
 * defined by SIMULATED_SENSOR_FREE.
 * @param position: angle in degrees representing the current position of the robot on the circle
 * @param num_tower: number of towers placed on course
 * @param tower_loc: array of tower locations given by degrees
 * 
*/
uint8_t get_simulated_sensor_value(float position, uint8_t num_tower, uint16_t tower_loc[]) {
    uint8_t simulated_sensor_value;
    for (uint8_t i = 0; i < num_tower; i++){
        if (min_diff_between_angles(position, (float) tower_loc[i]) < LOCATION_TOLERANCE) { // tower
            simulated_sensor_value = SIMULATED_SENSOR_TOWER;
        }
        else { // free space
            simulated_sensor_value = SIMULATED_SENSOR_FREE;
        }
    }

    return simulated_sensor_value;
}

void resample_particles(struct particle particle_array[]) {
    float cumulative_weight = 0.0;
    struct particle resampled_particle_array[NUM_PARTICLES];
    struct particle sum_weight_array[NUM_PARTICLES];
    uint8_t i = 0;
    
    for (uint8_t j = 0; j < NUM_PARTICLES; j++){
        cumulative_weight += particle_array[j].weight;
        sum_weight_array[j].weight = cumulative_weight;
        sum_weight_array[j].angle = particle_array[j].angle;
        sum_weight_array[j].classification = particle_array[j].classification;
    }
    // printf("Press ENTER key to Print sum_weights_array.\n");  
    // getchar();
    // print_particle_array_everything(sum_weight_array);

    // while(idx_arr )
    //     if particle_array[idx_array] > sum_weight_array[idx_arr]
    cumulative_weight = 0.0;
    for (uint8_t j = 0; j < NUM_PARTICLES; j++){
        uint8_t random = roundf(((float)rand()) / RAND_MAX * (NUM_PARTICLES - 1));
        // printf("%s", "Random: ");
        // printf("%d", random);
        // printf("%s", "\n");
        // printf("%s", "j: ");
        // printf("%d", j);
        // printf("%s", "\n");
        float U = (particle_array[random].weight) + j*1/NUM_PARTICLES;
        // printf("%.6f", U);
        while (U > sum_weight_array[i].weight){
            i++;
            // printf("%s", "hi");
        }
        resampled_particle_array[j].weight = particle_array[i].weight;
        resampled_particle_array[j].angle = particle_array[i].angle;
        resampled_particle_array[j].classification = particle_array[i].classification;
        
    //     weight = particle_array[i];
    }
    
    /* add 5% random particles */
    for (uint8_t i = 0; i < 0.05 * NUM_PARTICLES; i++) {
        // generate a random angle between 0 & 360
        float random_angle = ( (float) rand() ) / ( (float) RAND_MAX ) * 360.0;
        float random_weight = ( (float) rand() ) / ( (float) RAND_MAX );
        struct particle p = 
        {
            .angle = random_angle,
            .classification = 0, // unimportant, will get updated in the next iteration of algorithm
            .weight = random_weight // unimportant, will get updated in the next iteration of algorithm
        };
        resampled_particle_array[i] = p;
    }

    memcpy(particle_array, resampled_particle_array, sizeof(resampled_particle_array));
    // printf("Press ENTER key to Print resampled array.\n");  
    // getchar();
    // print_particle_array_everything(resampled_particle_array);
    }

// define which particles are towers or not, based off of given inputs.
void classify_particle(uint8_t num_tower, uint16_t tower_loc[], struct particle* particle_ptr){
    // printf("ENTERED classify_particle\n");
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
    // printf("ENTERED min_diff_between_angles\n");
    if (a1 > 360.0 || a1 < 0.0) {
        printf("ERROR: Angle a1 out of bounds. Ensure angle is between 0 and 360 degrees.");
    }

    if (a2 > 360.0 || a2 < 0.0) {
        printf("ERROR: Angle a2 out of bounds. Ensure angle is between 0 and 360 degrees.");
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
    // printf("ENTERED update_particle_weight\n");
    if (particle_ptr->classification == 0) {     // free space particle
        particle_ptr->weight = probability_free_space(sensor_val);
    }
    else {                                          // tower particle
        particle_ptr->weight = probability_tower(sensor_val);
    }
}

// Compute probability of particle being a tower or free space
float probability_free_space(uint8_t sensor_val){
    // printf("ENTERED probability_free_space\n");
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
        printf("PROBLEM %.6f\n", P);
    }
    return P;
}

float probability_tower(uint8_t sensor_val){
    // printf("ENTERED probability_tower\n");
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
        printf("PROBLEM %.6f\n", P);
    }
    return P;
}

/**
 * @todo: ensure this is working correctly.
 * may require using pointers to directly access the structs
*/
void normalize_weights(struct particle particle_array[]) {
    printf("ENTERED normalize_weights\n");
    print_particle_array(particle_array, NUM_PARTICLES);
    float sum_weights = 0.0;

    // calculate total weights
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        sum_weights += particle_array[i].weight;
    }

    printf("SUM %.4f\n", sum_weights);

    // normalize each weight
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        particle_array[i].weight = particle_array[i].weight / sum_weights;
    }
    
    print_particle_array_weights(particle_array, NUM_PARTICLES);
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

// void print_particle_array(struct particle particle_array[], uint8_t num_particles) 
// { 
//     printf("ENTERED print_particle_array\n");
//     uint8_t i; 
//     for (i = 0; i < num_particles; i++) 
//         // printf("Classification: %d Angle: %.4f, Weight: %.4f", particle_array[i].classification, particle_array[i].angle, particle_array[i].weight); 
//         printf("C: %d A: %.4f, W: %.4f ", particle_array[i].classification, particle_array[i].angle, particle_array[i].weight); 
//     printf("\n"); 
// } 

void print_particle_array(struct particle particle_array[], uint8_t num_particles) 
{ 
    // printf("ENTERED print_particle_array\n");
    uint8_t i; 
    printf("Classifications: ");
    for (i = 0; i < num_particles; i++) 
        printf("%d ", particle_array[i].classification); 
    printf("\n"); 
    printf("Weights: ");
    for (i = 0; i < num_particles; i++) 
        printf("%.4f ", particle_array[i].weight); 
    printf("\n"); 
    printf("Angles: ");
    for (i = 0; i < num_particles; i++) 
        printf("%.4f ", particle_array[i].angle); 
    printf("\n"); 
} 

void print_particle_array_weights(struct particle particle_array[], uint8_t num_particles) 
{ 
    uint8_t i; 
    for (i = 0; i < num_particles; i++) 
        printf("%.6f ", particle_array[i].weight); 
    printf("\n"); 
} 

