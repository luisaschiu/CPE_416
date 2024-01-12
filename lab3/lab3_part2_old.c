/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment: Lab 3 part 2
 * Description: 
*/
#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#define LEFT_SENSOR_PIN 0
#define RIGHT_SENSOR_PIN 1
#define LEARNING_RATE 0.005
#define NUM_DATA_ARRAY 500
#define LEFT_TARGET 20 //Arbitrary value, fix later
#define RIGHT_TARGET 20 //Arbitrary value, fix later

void motor(uint8_t, int8_t);
struct compute_neural_network(uint8_t, uint8_t);

int main(void) {
    init();  //initialize board hardware
    for (i = 0; i < NUM_DATA_ARRAY; i++) {
        compute_neural_network(left_sensor_array[i], right_sensor_array[i]);
    }
    // pseudo code for data capture
    
    int8_t data_counter = 0;
    u08 left_sensor;
    u08 right_sensor;
    // create an array of size NUM_DATA_ARRAY
    u08 left_sensor_array[NUM_DATA_ARRAY];
    u08 right_sensor_array[NUM_DATA_ARRAY];
    
    // Data Capture
    while (1){
        left_sensor = analog(LEFT_SENSOR_PIN);
        right_sensor = analog(RIGHT_SENSOR_PIN);
        // store sensor data in respective arrays
        left_sensor_array[data_counter] = left_sensor;
        right_sensor_array[data_counter] = right_sensor;
        // print "Data" in first row
        lcd_cursor(0,0);
        print_string("Data");
        data_counter ++;
        // print counter value to show number of readings captured
        lcd_cursor(5,0);
        print_num(data_counter);
        // print left and right sensor readings
        lcd_cursor(1,0);
        print_num(left_sensor);
        lcd_cursor(1,5);
        print_num(right_sensor);
        _delay_ms(200);  // NOTE: Might have to adjust so we don't capture sensor readings too quickly or too slowly.
        // Break while loop once button is pressed. NOTE: Might have to adjust to using case statements.
        if get_btn(){
            break;
        }


    }
}

struct compute_neural_network(uint8_t left_data, uint8_t right_data){
    // Training Mode.
    // 3 hidden layer neurons, 2 output layer neurons.
    float old_w1;
    float old_w2;
    float old_w3;
    float old_w4;
    float old_w5;
    float old_w6;
    float old_w7;
    float old_w8;
    float old_w9;
    float old_w10;
    float old_w11;
    float old_w12;
    float old_b1;
    float old_b2;
    float old_b3;
    float old_b4;
    float old_b5;
    // initialize weights and bias to random numbers between 0 and 1
    old_w1 = (float) rand()/RAND_MAX;
    old_w2 = (float) rand()/RAND_MAX;
    old_w3 = (float) rand()/RAND_MAX;
    old_w4 = (float) rand()/RAND_MAX;
    old_w5 = (float) rand()/RAND_MAX;
    old_w6 = (float) rand()/RAND_MAX;
    old_w7 = (float) rand()/RAND_MAX;
    old_w8 = (float) rand()/RAND_MAX;
    old_w9 = (float) rand()/RAND_MAX;
    old_w10 = (float) rand()/RAND_MAX;
    old_w11 = (float) rand()/RAND_MAX;
    old_w12 = (float) rand()/RAND_MAX;
    old_b1 = (float) rand()/RAND_MAX;
    old_b2 = (float) rand()/RAND_MAX;
    old_b3 = (float) rand()/RAND_MAX;
    old_b4 = (float) rand()/RAND_MAX;
    old_b5 = (float) rand()/RAND_MAX;
    new_w1 = (float) rand()/RAND_MAX;
    new_w2 = (float) rand()/RAND_MAX;
    new_w3 = (float) rand()/RAND_MAX;
    new_w4 = (float) rand()/RAND_MAX;
    new_w5 = (float) rand()/RAND_MAX;
    new_w6 = (float) rand()/RAND_MAX;
    new_w7 = (float) rand()/RAND_MAX;
    new_w8 = (float) rand()/RAND_MAX;
    new_w9 = (float) rand()/RAND_MAX;
    new_w10 = (float) rand()/RAND_MAX;
    new_w11 = (float) rand()/RAND_MAX;
    new_w12 = (float) rand()/RAND_MAX;
    new_b1 = (float) rand()/RAND_MAX;
    new_b2 = (float) rand()/RAND_MAX;
    new_b3 = (float) rand()/RAND_MAX;
    new_b4 = (float) rand()/RAND_MAX;
    new_b5 = (float) rand()/RAND_MAX;
    // Start of equations. Note: x1 = input 1, x2 = input 2.
    // Net functions on h values
    net_h1 = x1*old_w1 + x2*old_w2 + old_b1;
    net_h2 = x1*old_w3 + x2*old_w4 + old_b2;
    net_h3 = x1*old_w5 + x2*old_w6 + old_b3;
    // Activation function on h values
    out_h1 = 1/(1+ exp (-net_h1));
    out_h2 = 1/(1+ e^(-net_h2));
    out_h3 = 1/(1+ e^(-net_h3));
    // Net functions on o values
    net_o1 = out_h1*old_w7 + out_h2*old_w8 + out_h3*old_w9 + old_b4;
    net_o2 = out_h1*old_w10 + out_h2*old_w11 + out_h3*old_w12 + old_b5;
    // Activation function on o values
    out_o1 = 1/(1+ e^(-net_o1));
    out_o2 = 1/(1+ e^(-net_o2));

    // Equations for output layer weights
    // Partial differentiation equations: Neuron o1
    dE_o1_dout_o1 = (out_o1-target_o1);  // NOTE: DEFINE TARGET_O1
    dout_o1_dnet_o1 = (out_o1*(1-out_o1));
    dE_net_o1_d_w7 = out_h1;
    dE_net_o1_d_w8 = out_h2;
    dE_net_o1_d_w9 = out_h3;
    dE_total_d_w7 = dE_o1_dout_o1 * dout_o1_dnet_o1 * dE_net_o1_d_w7;
    dE_total_d_w8 = dE_o1_dout_o1 * dout_o1_dnet_o1 * dE_net_o1_d_w8;
    dE_total_d_w9 = dE_o1_dout_o1 * dout_o1_dnet_o1 * dE_net_o1_d_w9;
    // Partial differentiation equations: Neuron o2
    dE_o2_dout_o2 = (out_o2-target_o2);  // NOTE: DEFINE TARGET_O2
    dout_o2_dnet_o2 = (out_o2*(1-out_o2));
    dE_net_o2_d_w10 = out_h1;
    dE_net_o2_d_w11 = out_h2;
    dE_net_o2_d_w12 = out_h3;
    dE_total_d_w10 = dE_o2_dout_o2 * dout_o2_dnet_o2 * dE_net_o2_d_w10;
    dE_total_d_w11 = dE_o2_dout_o2 * dout_o2_dnet_o2 * dE_net_o2_d_w11;
    dE_total_d_w12 = dE_o2_dout_o2 * dout_o2_dnet_o2 * dE_net_o2_d_w12;
    // Back Propogate: Update output layer weights
    new_w7 = old_w7 - LEARNING_RATE*dE_total_d_w7;
    new_w8 = old_w8 - LEARNING_RATE*dE_total_d_w8;
    new_w9 = old_w9 - LEARNING_RATE*dE_total_d_w9;
    new_w10 = old_w10 - LEARNING_RATE*dE_total_d_w10;
    new_w11 = old_w11 - LEARNING_RATE*dE_total_d_w11;
    new_w12 = old_w12 - LEARNING_RATE*dE_total_d_w12;

    // Equations for hidden layer weights
    // Partial differentiation equations: Neuron h1
    dE_total_dout_h1 = dE_o1_dout_o1*dout_o1_dnet_o1*old_w7 + dE_o2_dout_o2*dout_o2_dnet_o2*old_w10;
    dout_h1_dnet_h1 = out_h1*(1-out_h1);
    // w1
    dnet_h1_d_w1 = x1;
    dE_total_d_w1 = dE_total_dout_h1*dout_h1_dnet_h1*dnet_h1_d_w1;
    // w2
    dnet_h1_d_w2 = x2;
    dE_total_d_w2 = dE_total_dout_h1*dout_h1_dnet_h1*dnet_h1_d_w2;
    
    // Partial differentiation equations: Neuron h2
    dE_total_dout_h2 = dE_o1_dout_o1*dout_o1_dnet_o1*old_w8 + dE_o2_dout_o2*dout_o2_dnet_o2*old_w11;
    dout_h2_dnet_h2 = out_h2*(1-out_h2);
    // w3
    dnet_h2_d_w3 = x1;
    dE_total_d_w3 = dE_total_dout_h2*dout_h2_dnet_h2*dnet_h2_d_w3;
    // w4
    dnet_h2_d_w4 = x2;
    dE_total_d_w4 = dE_total_dout_h2*dout_h2_dnet_h2*dnet_h2_d_w4;

    // Partial differentiation equations: Neuron h3
    dE_total_dout_h3 = dE_o1_dout_o1*dout_o1_dnet_o1*old_w9 + dE_o2_dout_o2*dout_o2_dnet_o2*old_w12;
    dout_h3_dnet_h3 = out_h3*(1-out_h3);
    // w5
    dnet_h2_d_w5 = x1;
    dE_total_d_w5 = dE_total_dout_h3*dout_h3_dnet_h3*dnet_h2_d_w5;
    // w6
    dnet_h2_d_w6 = x2;
    dE_total_d_w6 = dE_total_dout_h3*dout_h3_dnet_h3*dnet_h2_d_w6;

    // Back Propogate: Update hidden layer weights
    new_w1 = old_w1 - LEARNING_RATE*dE_total_d_w1;
    new_w2 = old_w2 - LEARNING_RATE*dE_total_d_w2;
    new_w3 = old_w3 - LEARNING_RATE*dE_total_d_w3;
    new_w4 = old_w4 - LEARNING_RATE*dE_total_d_w4;
    new_w5 = old_w5 - LEARNING_RATE*dE_total_d_w5;
    new_w6 = old_w6 - LEARNING_RATE*dE_total_d_w6;
}
