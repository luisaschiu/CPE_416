#include <stdint.h>
#include <stdio.h>

#define a_free  0 
#define b_free  20
#define c_free  50
#define d_free  70
#define a_tower 60
#define b_tower 90
#define c_tower 110
#define d_tower 120

float probability_free_space(uint8_t);
float probability_tower(uint8_t);

int main() {
    // float p = probability_tower(65);
    printf("pt(65) %.4f\n", probability_tower(65));
    printf("pt(90) %.4f\n", probability_tower(90));
    printf("pt(100) %.4f\n", probability_tower(100));

    return 0;
}


// Compute probability of particle being a tower or free space
float probability_free_space(uint8_t sensor_val){
    // printf("ENTERED probability_free_space\n");
    float P;
    float u_free = 2.0/ ( (float) (d_free + c_free - b_free - a_free) );
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