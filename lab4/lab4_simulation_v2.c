#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define NUM_PARTICLES 100
#define a_free  0 
#define b_free  20
#define c_free  50
#define d_free  65
#define a_tower 60
#define b_tower 90
#define c_tower 110
#define d_tower 125
#define LOCATION_TOLERANCE 8.67 // if a value is less than 8.67 degrees away
#define ANGLE_THRESHOLD 45
#define THRESHOLD_PERCENTAGE 0.25

// #define MEAN 31.023882
// #define STD_DEV 7.595287
#define MEAN 3.1023882
#define STD_DEV 0.7595287

#define SIMULATED_SENSOR_TOWER 100
#define SIMULATED_SENSOR_FREE 35 

/**
 * @todo: adjust this to stop the algorithm earlier/later
*/
#define STD_DEV_THRESHOLD 5

struct particle {
    uint8_t classification; //0 if free space, 1 if tower
    float angle;
    float weight;
};

float probability_free_space(uint16_t);
float probability_tower(uint16_t);
void classify_particle(uint8_t, uint16_t[], struct particle*);
void print_particle_array_weights(struct particle[], uint8_t);
void print_particle_array_angles(struct particle[], uint8_t);
void selection_sort(struct particle[], uint8_t); 
void swap(struct particle*, struct particle*); 
float calculate_std_dev(uint8_t, struct particle[]);
void normalize_weights(struct particle[]);
void resample_particles(struct particle[]);
float box_muller_transform(void);
uint8_t get_simulated_sensor_value(float, uint8_t, uint16_t[]);
float min_diff_between_angles(float, float);

int main(void) {
    /**
     * INITIALIZE PARTICLE & TOWER DATA
    */
    struct particle particle_array[NUM_PARTICLES];
    for (uint8_t i = 0; i < NUM_PARTICLES; i++){
        particle_array[i].angle = (360.0/NUM_PARTICLES)*i;
    }
    
    uint8_t num_tower;
    uint8_t target_tower;

    num_tower = 3; // subject to change, tester value for now.
    // uint16_t tower_loc[num_tower];
    // tower_loc[0] = 0;
    // tower_loc[1] = 90;
    // tower_loc[2] = 225;
    uint16_t tower_loc[] = { 0, 90, 225 };
    target_tower = 2; // subject to change, tester value for now.
    float robot_position = 50.0;

    float std_dev;

    /**
     * MONTE CARLO LOCALIZATION
    */
    while(1) {
        // move robot X ticks + (no simulated noise. assuming no motion noise)
        robot_position += MEAN;
        // restrict robot_position between 0 & 360 (inclusive)
        if (robot_position > 360.0) {
            robot_position -= 360.0;
        }

        uint8_t sensor_val = get_simulated_sensor_value(robot_position, num_tower, tower_loc);
        // for each particle
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

            classify_particle(num_tower, tower_loc, current_particle_ptr);
            if (current_particle_ptr->classification == 0) {     // free space particle
                current_particle_ptr->weight = probability_free_space(sensor_val);
            }
            else {                                          // tower particle
                current_particle_ptr->weight = probability_tower(sensor_val);
            }
            // printf("%.6f ", current_particle_ptr->weight; // for debug
        }
        normalize_weights(particle_array);
        // sort particle_array by weight in descending order
        // selection_sort(particle_array, NUM_PARTICLES);
        resample_particles(particle_array);
        // selection_sort(particle_array, NUM_PARTICLES);
        print_particle_array_angles(particle_array, NUM_PARTICLES);

        // check if standard deviation is close enough.
        /**
         * @todo
        */
        std_dev = calculate_std_dev(NUM_PARTICLES, particle_array);
        printf("STD DEV: %.4f\n", std_dev);

        if (std_dev < STD_DEV_THRESHOLD) {
            break;
        }
        
    }
    printf("ROBOT @ %.6f degrees", robot_position);
    // print_particle_array_weights(particle_array, NUM_PARTICLES);
    
}

void normalize_weights(struct particle particle_array[]) {
    float sum_weights = 0.0;

    // calculate total weights
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        sum_weights += particle_array[i].weight;
    }

    // normalize each weight
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        particle_array[i].weight /= sum_weights;
    }

}

// void resample_particles(struct particle particle_array[]) {

// }

void resample_particles(struct particle particle_array[]) {
    struct particle resampled_particle_array[NUM_PARTICLES];
    uint8_t idx_arr = 0;
    uint8_t idx_resampled = 0;

    // add 95% particles based on weights
    while (idx_resampled < 0.95*NUM_PARTICLES) {
        if (idx_arr >= NUM_PARTICLES) {
            // catch edge case where index of current array is greater than 100
            // reset idx_arr to 0 and continue to grab particles until idx_resampled 
            idx_arr = 0;
            // printf("ERROR: Particle array index out of bounds. Index: %d", idx_arr);
        }
        struct particle p = particle_array[idx_arr];
        idx_arr++;
        
        // multiply by 100 to account for particle weights normalized sum to 1
        uint8_t num_current_particles = (uint8_t) roundf(p.weight * 100);
        // while (num_current_particles > 0) {
        //     if (idx_resampled >= 0.95*NUM_PARTICLES) {
        //         break;
        //     }
        //     struct particle particle_to_add = {.angle=p.angle, .classification=p.classification, .weight=p.weight};
        //     resampled_particle_array[idx_resampled] = particle_to_add;
        //     idx_resampled++;
        //     num_current_particles--;
        // }
        for (uint8_t i = 0; i < num_current_particles; i++) {
            if (idx_resampled >= 0.95*NUM_PARTICLES) {
                break;
            }
            struct particle particle_to_add = {.angle=p.angle, .classification=p.classification, .weight=p.weight};
            resampled_particle_array[idx_resampled] = particle_to_add;
            // printf("%.2f \n", particle_to_add.angle);
            // printf("%.2f \n", resampled_particle_array[idx_resampled].angle);
            idx_resampled++;
        }
    }


    // add 5% random particles
    while (idx_resampled < NUM_PARTICLES) {
        // generate a random angle between 0 & 360
        float random_angle = ( (float) rand() ) / ( (float) RAND_MAX ) * 360.0;
        struct particle p = 
        {
            .angle = random_angle,
            .classification = 0, // unimportant, will get updated in the next iteration of algorithm
            .weight = 0.0 // unimportant, will get updated in the next iteration of algorithm
        };
        resampled_particle_array[idx_resampled] = p;
        idx_resampled++;
    }

    // particle_array = resampled_particle_array;
    memcpy(particle_array, resampled_particle_array, sizeof(resampled_particle_array));
}

float calculate_std_dev(uint8_t size, struct particle particle_array[]) {
    float mean;
    float total = 0;
    float std_diff_squared = 0;
    float std_dev;

    /**
     * determine if particles need to be recalculated for edge case where
     * particles are bunched around 0/360 line.
    */
    uint8_t particle_threshold = (uint8_t) (NUM_PARTICLES * THRESHOLD_PERCENTAGE);
    uint8_t num_particles_near_0 = 0;
    uint8_t num_particles_near_360 = 0;
    // count the number of particles near 0 and near 360
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        if (particle_array[i].angle <= 0 + ANGLE_THRESHOLD) {
            num_particles_near_0++;
        }
        if (particle_array[i].angle >= 360 - ANGLE_THRESHOLD) {
            num_particles_near_360++;
        }
    }

    // if the number of particles near 0 and near 360 is greater than the threshold
    if (num_particles_near_0 + num_particles_near_360 >= particle_threshold) {
        // if more particles near 0, move particles near 360 over the 0/360 gap
        if (num_particles_near_0 > num_particles_near_360) {
            for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
                if (particle_array[i].angle >= 360 - ANGLE_THRESHOLD) {
                    particle_array[i].angle = 360 - particle_array[i].angle;
                }
            }
        }
        // otherwise if more particles near 360, move particles near 0 over the 0/360 gap
        else {
            for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
                if (particle_array[i].angle <= 0 + ANGLE_THRESHOLD) {
                    particle_array[i].angle = 360 - particle_array[i].angle;
                }
            }
        }
    }

    // calculating total needed for mean
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        total += particle_array[i].angle;
    }
    mean = total/size; // calculate mean value
    // printf("Mean: %2f\n", mean);
    // loop through again to calculate values for standard deviation
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        std_diff_squared += pow((particle_array[i].angle - mean),2);
    }
    std_dev = (float) sqrt(std_diff_squared/size); // calculate standard deviation
    return std_dev;
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

/**
 * Function: calculates the minimum delta between two angles on a circle.
 * This function accounts for edge cases where 1 point @ one side of 0/360 and 1 point @ other side.
 * @param a1: (Float) angle #1 in degrees between 0 & 360
 * @param a2: (Float) angle #2 in degrees between 0 & 360
*/
float min_diff_between_angles(float a1, float a2) {
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

/**
 * @todo figure out natural log syntax
*/
float box_muller_transform() {
    float u1 = ( (float) rand() ) / ( (float) RAND_MAX );
    float u2 = ( (float) rand() ) / ( (float) RAND_MAX );
    float sq = (float) sqrt(-2.0*log(u1));
    float co = (float) cos(2.0*M_PI*u2);
    float z = sq*co;
    return z;
}

// Compute probability of particle being a tower or free space
float probability_free_space(uint16_t sensor_val){
    float P;
    float u_free = 2.0/(d_free + c_free - b_free - a_free);
    if ((sensor_val >= b_free) && (sensor_val < c_free)){
        P = u_free;
    }
    else if ((sensor_val >= a_free) && (sensor_val < b_free)){
        P = u_free *((sensor_val-a_free)/(b_free-a_free));
    }
    else if ((sensor_val >= c_free) && (sensor_val < d_free)){
        P = u_free *((d_free-sensor_val)/(d_free-c_free));
    }
    else{
        P = 0;
    }
    if (P > 1 || P < 0) {
        printf("PROBLEM %.6f\n", P);
    }
    return P;
}

float probability_tower(uint16_t sensor_val){
    float P;
    float u_tower = 2.0/(d_tower + c_tower - b_tower - a_tower);
    if ((sensor_val >= b_tower) && (sensor_val < c_tower)){
        P = u_tower;
    }
    else if ((sensor_val >= a_tower) && (sensor_val < b_tower)){
        P = u_tower *((sensor_val-a_tower)/(b_tower-a_tower));
    }
    else if ((sensor_val >= c_tower) && (sensor_val < d_tower)){
        P = u_tower *((d_tower-sensor_val)/(d_tower-c_tower));
    }
    else{
        P = 0;
    }
    if (P > 1 || P < 0) {
        printf("PROBLEM %.6f\n", P);
    }
    return P;
}

void swap(struct particle* xp, struct particle* yp) 
{ 
    struct particle temp = *xp; 
    *xp = *yp; 
    *yp = temp; 
} 
  
// Function to perform Selection Sort 
void selection_sort(struct particle particle_array[], uint8_t num_particles) 
{ 
    uint8_t i, j, max_idx; 
  
    // One by one move boundary of 
    // unsorted subarray 
    for (i = 0; i < num_particles - 1; i++) { 
        // Find the maximum element in 
        // unsorted array 
        max_idx = i; 
        for (j = i + 1; j < num_particles; j++) 
            if (particle_array[j].weight > particle_array[max_idx].weight) 
                max_idx = j; 
  
        // Swap the found maximum element 
        // with the first element 
        swap(&particle_array[max_idx], &particle_array[i]); 
    } 
} 

void print_particle_array_weights(struct particle particle_array[], uint8_t num_particles) 
{ 
    uint8_t i; 
    for (i = 0; i < num_particles; i++) 
        printf("%.6f ", particle_array[i].weight); 
    printf("\n"); 
} 

void print_particle_array_angles(struct particle particle_array[], uint8_t num_particles) 
{ 
    uint8_t i; 
    for (i = 0; i < num_particles; i++) 
        printf("%.6f ", particle_array[i].angle); 
    printf("\n"); 
} 

