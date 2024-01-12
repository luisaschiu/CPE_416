#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define NUM_PARTICLES 20
#define a_free  0 
#define b_free  20
#define c_free  50
#define d_free  65
#define a_tower 60
#define b_tower 90
#define c_tower 110
#define d_tower 125
#define LOCATION_TOLERANCE 8.67 // if a value is less than 8.67 degrees away
//#define LOCATION_TOLERANCE 8
#define ANGLE_THRESHOLD 45
#define THRESHOLD_PERCENTAGE 0.25

// #define MEAN 31.023882
#define MEAN 3.1023882
// #define STD_DEV 7.595287
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

void normalize_weights(struct particle[]);
void resample_particles(struct particle[]);
void print_particle_array_everything(struct particle[]) ;

struct particle particle_array[NUM_PARTICLES];
struct particle sum_weight_array[NUM_PARTICLES];
struct particle resampled_particle_array[NUM_PARTICLES];

int main(void) {
    // Assign angles for first initial pass
    for (uint8_t i = 0; i < NUM_PARTICLES; i++){
        particle_array[i].angle = (360.0/NUM_PARTICLES)*i;
    }

        // initialize random weight
    particle_array[0].weight = 0.01;
    particle_array[1].weight = 0.01;
    particle_array[2].weight = 0.1;
    particle_array[3].weight = 0.2;
    particle_array[4].weight = 0.2;
    particle_array[5].weight = 0.1;
    particle_array[6].weight = 0.1;
    particle_array[7].weight = 0.1;
    particle_array[8].weight = 0.1;
    particle_array[9].weight = 0.1;
    particle_array[10].weight = 0.1;
    particle_array[11].weight = 0.05;
    particle_array[12].weight = 0.05;
    particle_array[13].weight = 0.2;
    particle_array[14].weight = 0.01;
    particle_array[15].weight = 0.01;
    particle_array[16].weight = 0.001;
    particle_array[17].weight = 0.001;
    particle_array[18].weight = 0.001;
    particle_array[19].weight = 0.001;

    while(1){
        printf("Press ENTER key to Print Values before normalizing.\n");  
        getchar();
        print_particle_array_everything(particle_array);

        normalize_weights(particle_array);

        printf("Press ENTER key to Print Values after normalizing.\n");  
        getchar();
        print_particle_array_everything(particle_array);
        float cumulative_weight = 0.0;
        uint8_t i = 0;
        
        for (uint8_t j = 0; j < NUM_PARTICLES; j++){
            cumulative_weight += particle_array[j].weight;
            sum_weight_array[j].weight = cumulative_weight;
            sum_weight_array[j].angle = particle_array[j].angle;
        }
        printf("Press ENTER key to Print sum_weights_array.\n");  
        getchar();
        print_particle_array_everything(sum_weight_array);

        // while(idx_arr )
        //     if particle_array[idx_array] > sum_weight_array[idx_arr]
        cumulative_weight = 0.0;
        for (uint8_t j = 0; j < NUM_PARTICLES; j++){
            uint8_t random = roundf(((float)rand()) / RAND_MAX * (NUM_PARTICLES - 1));
            printf("%s", "Random: ");
            printf("%d", random);
            printf("%s", "\n");
            printf("%s", "j: ");
            printf("%d", j);
            printf("%s", "\n");
            float U = (particle_array[random].weight) + j*1/NUM_PARTICLES;
            printf("%.6f", U);
            while (U > sum_weight_array[i].weight){
                i++;
                printf("%s", "hi");
            }
            resampled_particle_array[j].weight = particle_array[i].weight;
            resampled_particle_array[j].angle = particle_array[i].angle;
            
        //     weight = particle_array[i];
        }
        
        memcpy(particle_array, resampled_particle_array, sizeof(resampled_particle_array));
        printf("Press ENTER key to Print resampled array.\n");  
        getchar();
        print_particle_array_everything(resampled_particle_array);
    }
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

void print_particle_array_everything(struct particle particle_array[]) 
{ 
    for (uint8_t i = 0; i < NUM_PARTICLES; i++) {
        printf("%s ", "Particle: "); 
        printf("%d", i);
        printf("%s ", ", "); 
        printf("%s ", "Angle: "); 
        printf("%.6f ", particle_array[i].angle);
        printf("%s ", "Weight: "); 
        printf("%.6f ", particle_array[i].weight); 
        // printf("%s ", "Classification: "); 
        // printf("%d ", particle_array[i].classification);
        printf("\n"); 
    }
} 