#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// #include <time.h>

#define NUM_PARTICLES 100
#define a_free  0 
#define b_free  20
#define c_free  50
#define d_free  70
#define a_tower 60
//#define a_tower 68
#define b_tower 90
//#define b_tower 80
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

float probability_free_space(uint8_t);
float probability_tower(uint8_t);

int main() {
    uint8_t test_value = 100;
    float probability_fs;
    float probability_t;
    probability_fs = probability_free_space(test_value);
    probability_t = probability_tower(test_value);
    printf("Probability Free Space: ");
    printf("%.6f",probability_fs);
    printf(", Probability Tower: ");
    printf("%.6f", probability_t);
}

float probability_free_space(uint8_t sensor_val){
    // printf("ENTERED probability_free_space\n");
    float P;
    float u_free = (float) (2.0/(d_free + c_free - b_free - a_free));
    if ((sensor_val >= b_free) && (sensor_val < c_free)){
        P = u_free;
    }
    else if ((sensor_val >= a_free) && (sensor_val < b_free)){
        P = (float) (u_free *((float)(sensor_val-a_free)/(b_free-a_free)));
    }
    else if ((sensor_val >= c_free) && (sensor_val < d_free)){
        P = (float) (u_free *((float)(d_free-sensor_val)/(d_free-c_free)));
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
    float u_tower = (float) (2.0/(d_tower + c_tower - b_tower - a_tower));
    if ((sensor_val >= b_tower) && (sensor_val < c_tower)){
        P = u_tower;
    }
    else if ((sensor_val >= a_tower) && (sensor_val < b_tower)){
        P = (float) (u_tower *((float)(sensor_val-a_tower)/(b_tower-a_tower)));
    }
    else if ((sensor_val >= c_tower) && (sensor_val < d_tower)){
        P = (float) (u_tower *((float)(d_tower-sensor_val)/(d_tower-c_tower)));
    }
    else{
        P = 0;
    }
    if (P > 1 || P < 0) {
        printf("PROBLEM %.6f\n", P);
    }
    return P;
}