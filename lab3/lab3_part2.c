/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment Lab 3 part 2
 * Description: Uses a Neural Network to train the robot to follow a line. 
 * Training targets are gathered using a the proportional controller from Lab3 Part1.
 * Network consists of 3 hidden layer neurons and 2 output layer neurons.
 * Network trains correctly given 200 Data Points, 400 Epochs, and Learning Rate of 0.5
*/

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
#define LEARNING_RATE 0.5
#define NUM_DATA_ARRAY 200
//#define NUM_EPOCHS 400
#define Kp 0.05
#define SERVO_0_CAL 1.25
#define INIT_SPEED 5
#define MIN_SPEED 2
#define MAX_SPEED 100
#define MAX_DELTA 255
#define SENSOR_READ_DELAY 100

#define MIN_EPOCH 1
#define MAX_EPOCH 1000
#define INIT_EPOCHS 20

struct motor_command {
    uint8_t left;
    uint8_t right;
};

struct data_point {
    uint8_t left_sensor;
    uint8_t right_sensor;
    uint8_t left_motor;
    uint8_t right_motor;
};

void motor(uint8_t, int8_t);
float activation_function(float);

struct motor_command compute_proportional(uint8_t, uint8_t);
struct motor_command compute_neural_network(uint8_t, uint8_t);

struct data_point data[NUM_DATA_ARRAY];

float calculate_hidden_net(uint8_t, uint8_t, float, float, float);
float calculate_output_net(float, float, float, float, float, float, float);
void train_neural_network(uint16_t);

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



int main(void) {
    init();  //initialize board hardware

    typedef enum {
        PROPORTIONAL,
        DATA,
        TRAINING,
        LINE_FOLLOWING
    } state_var_type;
    state_var_type state = PROPORTIONAL;

    motor(LEFT_MOTOR, 0);
    motor(RIGHT_MOTOR, 0);

    old_w1 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w2 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w3 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w4 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w5 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w6 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w7 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w8 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w9 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w10 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w11 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_w12 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_b1 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_b2 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_b3 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_b4 = ( (float) rand() ) / ( (float) RAND_MAX );
    old_b5 = ( (float) rand() ) / ( (float) RAND_MAX );

    u08 left_sensor;
    u08 right_sensor;
    u16 data_counter = 0;
    struct motor_command motor_inst;
    
    while(1){
        switch(state) {
            case PROPORTIONAL:
                _delay_ms(SENSOR_READ_DELAY);
                left_sensor = analog(LEFT_SENSOR_PIN); // left wheel
                right_sensor = analog(RIGHT_SENSOR_PIN); // right wheel

                motor_inst = compute_proportional(left_sensor, right_sensor);

                motor(LEFT_MOTOR, motor_inst.left);
                motor(RIGHT_MOTOR, motor_inst.right);

                /* testing */
                clear_screen();
                lcd_cursor(0,0);
                print_string("Proportional");
                if (get_btn()){
                    state = DATA;
                    _delay_ms(250);
                }
                else {
                    state = PROPORTIONAL;
                }
                break;
            case DATA:
                // Data Capture
                motor(LEFT_MOTOR, 0);
                motor(RIGHT_MOTOR, 0);
                data_counter++;
//                for (uint16_t data_counter = 0; data_counter < NUM_DATA_ARRAY; data_counter++){
                left_sensor = analog(LEFT_SENSOR_PIN);
                right_sensor = analog(RIGHT_SENSOR_PIN);
                struct motor_command motor_data = compute_proportional(left_sensor, right_sensor);
                struct data_point cur_data;
                cur_data.left_sensor = left_sensor;
                cur_data.right_sensor = right_sensor;
                cur_data.left_motor = motor_data.left;
                cur_data.right_motor = motor_data.right;

                // store current data point in data array
                data[data_counter] = cur_data;
                // print "Data" in first row

                clear_screen();
                lcd_cursor(0,0);
                print_string("Data");
                // print counter value to show number of readings captured
                lcd_cursor(5,0);
                print_num(data_counter);
                // print left and right sensor readings
                lcd_cursor(0,1);
                print_num(left_sensor);
                lcd_cursor(5,1);
                print_num(right_sensor);
                _delay_ms(100);
                if (data_counter > NUM_DATA_ARRAY){
                    data_counter = 0;
                }
                if (get_btn()){
                    state = TRAINING;
                    _delay_ms(250);
                }
                else {
                    state = DATA;
                }
                break;
            case TRAINING:
            // Training
                motor(LEFT_MOTOR, 0);
                motor(RIGHT_MOTOR, 0);

                u08 accel;
                u08 epoch_delta;
                u16 num_epochs = INIT_EPOCHS;
                int16_t new_epochs;
            
                while(!get_btn()) {
                    accel = get_accel_y();
                    if (accel > 127) {
                        // if accel is hard right, increase num_epochs by 10 rather than 1
                        if (accel < 210) {
                            epoch_delta = 10;
                        }
                        else {
                            epoch_delta = 1;
                        }

                        // make sure num_epochs doesn't go over limit
                        if (num_epochs + epoch_delta <= MAX_EPOCH) {
                            num_epochs+= epoch_delta;
                        }
                        else {
                            num_epochs = MAX_EPOCH;
                        }
                    }
                    else if (accel < 127) {
                        // if accel is hard left, decrease num_epochs by 10 rather than 1
                        if (accel > 44) {
                            epoch_delta = 10;
                        }
                        else {
                            epoch_delta = 1;
                        }

                        new_epochs = num_epochs - epoch_delta;
                        if ( new_epochs >= MIN_EPOCH) {
                            num_epochs-= epoch_delta;
                        }
                        else {
                            num_epochs = MIN_EPOCH;
                        }
                    }

                    clear_screen();
                    lcd_cursor(0, 0);
                    print_string("EPOCHS");
                    lcd_cursor(0, 1);
                    print_num(num_epochs);
                    _delay_ms(200);
                    
                }
                train_neural_network(num_epochs);
                state = LINE_FOLLOWING;
                break;
            case LINE_FOLLOWING:
            // Neural network line following
                clear_screen();
                lcd_cursor(0, 0);
                print_string("NN Line");
                lcd_cursor(0, 1);
                print_string("Following");
                // _delay_ms(1000);
                _delay_ms(SENSOR_READ_DELAY);
                left_sensor = analog(LEFT_SENSOR_PIN); // left wheel
                right_sensor = analog(RIGHT_SENSOR_PIN); // right wheel
                
                motor_inst = compute_neural_network(left_sensor, right_sensor);

                motor(LEFT_MOTOR, motor_inst.left);
                motor(RIGHT_MOTOR, motor_inst.right);

                if (get_btn()){
                    state = TRAINING;
                }
                else {
                    state = LINE_FOLLOWING;
                }
                break;
            }
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

struct motor_command compute_proportional(uint8_t left_sensor, uint8_t right_sensor) {
    int16_t sensor_delta;
    int8_t speed_delta;

    sensor_delta = left_sensor - right_sensor;

    speed_delta = sensor_delta * MAX_SPEED / MAX_DELTA * Kp;

    struct motor_command motor_inst;

    // Limit Verification
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

float calculate_hidden_net(uint8_t in1, uint8_t in2, float w1, float w2, float b) {
    return ( ((in1/255.0) * w1) + ((in2/255.0) * w2) + b );
}

float calculate_output_net(float in1, float in2, float in3, float w1, float w2, float w3, float b) {
    return ( (in1 * w1) + (in2 * w2) + (in3 * w3) + b );
}

float activation_function(float x) {
    return (1 / (1 + expf(-x)));
}

struct motor_command compute_neural_network(uint8_t left_sensor, uint8_t right_sensor){

    float h1_net = calculate_hidden_net(left_sensor, right_sensor, old_w1, old_w2, old_b1);
    float h1_out = activation_function(h1_net);

    float h2_net = calculate_hidden_net(left_sensor, right_sensor, old_w3, old_w4, old_b2);
    float h2_out = activation_function(h2_net);

    float h3_net = calculate_hidden_net(left_sensor, right_sensor, old_w5, old_w6, old_b3);
    float h3_out = activation_function(h3_net);

    float o1_net = calculate_output_net(h1_out, h2_out, h3_out, old_w7, old_w8, old_w9, old_b4);
    float o1_out = activation_function(o1_net);

    float o2_net = calculate_output_net(h1_out, h2_out, h3_out, old_w10, old_w11, old_w12, old_b5);
    float o2_out = activation_function(o2_net);


    struct motor_command motor_inst;
    motor_inst.left = (uint8_t) (o1_out*100);
    motor_inst.right = (uint8_t) (o2_out*100);

    return motor_inst;
}

void train_neural_network(uint16_t num_epoch) {
    float new_w1;
    float new_w2;
    float new_w3;
    float new_w4;
    float new_w5;
    float new_w6;
    float new_w7;
    float new_w8;
    float new_w9;
    float new_w10;
    float new_w11;
    float new_w12;
    float new_b1;
    float new_b2;
    float new_b3;
    float new_b4;
    float new_b5;

    clear_screen();
    lcd_cursor(0, 0);
    print_string("Training");
    lcd_cursor(0, 1);
    print_string("EPOCH");

    for (uint16_t e = 0; e < num_epoch; e++) {
        for (uint16_t d = 0; d < NUM_DATA_ARRAY; d++) {
            // Print current epoch number
            lcd_cursor(5, 1);
            print_num(e);

            struct data_point cur_data = data[d];
            float x1 = cur_data.left_sensor / 255.0;
            float x2 = cur_data.right_sensor / 255.0;
            float target_o1 = cur_data.left_motor / 100.0;
            float target_o2 = cur_data.right_motor / 100.0;

            // Training Mode.
            // 3 hidden layer neurons, 2 output layer neurons.
            
            // initialize weights and bias to random numbers between 0 and 1
            
            // Start of equations. Note: x1 = input 1, x2 = input 2.
            // Net functions on h values
            float net_h1 = x1*old_w1 + x2*old_w2 + old_b1;
            float net_h2 = x1*old_w3 + x2*old_w4 + old_b2;
            float net_h3 = x1*old_w5 + x2*old_w6 + old_b3;
            // Activation function on h values
            float out_h1 = 1/(1 + expf(-net_h1));
            float out_h2 = 1/(1+ expf(-net_h2));
            float out_h3 = 1/(1+ expf(-net_h3));
            // Net functions on o values
            float net_o1 = out_h1*old_w7 + out_h2*old_w8 + out_h3*old_w9 + old_b4;
            float net_o2 = out_h1*old_w10 + out_h2*old_w11 + out_h3*old_w12 + old_b5;
            // Activation function on o values
            float out_o1 = 1/(1+ expf(-net_o1));
            float out_o2 = 1/(1+ expf(-net_o2));

//            uint16_t error_o1 = (u16) (abs((out_o1-target_o1) * 100));
//            uint16_t error_o2 = (u16) (abs((out_o2-target_o2) * 100));

            // Equations for output layer weights
            // Partial differentiation equations: Neuron o1
            float dE_o1_dout_o1 = (out_o1-target_o1);
            float dout_o1_dnet_o1 = (out_o1*(1-out_o1));
            float dE_net_o1_d_w7 = out_h1;
            float dE_net_o1_d_w8 = out_h2;
            float dE_net_o1_d_w9 = out_h3;
            float dE_net_o1_d_b4 = 1.0;
            float dE_total_d_w7 = dE_o1_dout_o1 * dout_o1_dnet_o1 * dE_net_o1_d_w7;
            float dE_total_d_w8 = dE_o1_dout_o1 * dout_o1_dnet_o1 * dE_net_o1_d_w8;
            float dE_total_d_w9 = dE_o1_dout_o1 * dout_o1_dnet_o1 * dE_net_o1_d_w9;
            float dE_total_d_b4 = dE_o1_dout_o1 * dout_o1_dnet_o1 * dE_net_o1_d_b4;
            // Partial differentiation equations: Neuron o2
            float dE_o2_dout_o2 = (out_o2-target_o2);  // NOTE: DEFINE TARGET_O2
            float dout_o2_dnet_o2 = (out_o2*(1-out_o2));
            float dE_net_o2_d_w10 = out_h1;
            float dE_net_o2_d_w11 = out_h2;
            float dE_net_o2_d_w12 = out_h3;
            float dE_net_o2_d_b5 = 1.0;
            float dE_total_d_w10 = dE_o2_dout_o2 * dout_o2_dnet_o2 * dE_net_o2_d_w10;
            float dE_total_d_w11 = dE_o2_dout_o2 * dout_o2_dnet_o2 * dE_net_o2_d_w11;
            float dE_total_d_w12 = dE_o2_dout_o2 * dout_o2_dnet_o2 * dE_net_o2_d_w12;
            float dE_total_d_b5 = dE_o2_dout_o2 * dout_o2_dnet_o2 * dE_net_o2_d_b5;
            // Back Propogate: Update output layer weights
            new_w7 = old_w7 - LEARNING_RATE*dE_total_d_w7;
            new_w8 = old_w8 - LEARNING_RATE*dE_total_d_w8;
            new_w9 = old_w9 - LEARNING_RATE*dE_total_d_w9;
            new_b4 = old_b4 - LEARNING_RATE*dE_total_d_b4;
            new_w10 = old_w10 - LEARNING_RATE*dE_total_d_w10;
            new_w11 = old_w11 - LEARNING_RATE*dE_total_d_w11;
            new_w12 = old_w12 - LEARNING_RATE*dE_total_d_w12;
            new_b5 = old_b5 - LEARNING_RATE*dE_total_d_b5;

            // Equations for hidden layer weights
            // Partial differentiation equations: Neuron h1
            float dE_total_dout_h1 = dE_o1_dout_o1*dout_o1_dnet_o1*old_w7 + dE_o2_dout_o2*dout_o2_dnet_o2*old_w10;
            float dout_h1_dnet_h1 = out_h1*(1-out_h1);
            // w1
            float dnet_h1_d_w1 = x1;
            float dE_total_d_w1 = dE_total_dout_h1*dout_h1_dnet_h1*dnet_h1_d_w1;
            // w2
            float dnet_h1_d_w2 = x2;
            float dE_total_d_w2 = dE_total_dout_h1*dout_h1_dnet_h1*dnet_h1_d_w2;
            // b1
            float dnet_h1_d_b1 = 1.0;
            float dE_total_d_b1 = dE_total_dout_h1*dout_h1_dnet_h1*dnet_h1_d_b1;
            
            // Partial differentiation equations: Neuron h2
            float dE_total_dout_h2 = dE_o1_dout_o1*dout_o1_dnet_o1*old_w8 + dE_o2_dout_o2*dout_o2_dnet_o2*old_w11;
            float dout_h2_dnet_h2 = out_h2*(1-out_h2);
            // w3
            float dnet_h2_d_w3 = x1;
            float dE_total_d_w3 = dE_total_dout_h2*dout_h2_dnet_h2*dnet_h2_d_w3;
            // w4
            float dnet_h2_d_w4 = x2;
            float dE_total_d_w4 = dE_total_dout_h2*dout_h2_dnet_h2*dnet_h2_d_w4;
            // b2
            float dnet_h2_d_b2 = 1.0;
            float dE_total_d_b2 = dE_total_dout_h2*dout_h2_dnet_h2*dnet_h2_d_b2;

            // Partial differentiation equations: Neuron h3
            float dE_total_dout_h3 = dE_o1_dout_o1*dout_o1_dnet_o1*old_w9 + dE_o2_dout_o2*dout_o2_dnet_o2*old_w12;
            float dout_h3_dnet_h3 = out_h3*(1-out_h3);
            // w5
            float dnet_h2_d_w5 = x1;
            float dE_total_d_w5 = dE_total_dout_h3*dout_h3_dnet_h3*dnet_h2_d_w5;
            // w6
            float dnet_h2_d_w6 = x2;
            float dE_total_d_w6 = dE_total_dout_h3*dout_h3_dnet_h3*dnet_h2_d_w6;
            // b3
            float dnet_h3_d_b3 = 1.0;
            float dE_total_d_b3 = dE_total_dout_h3*dout_h3_dnet_h3*dnet_h3_d_b3;

            // Back Propogate: Update hidden layer weights
            new_w1 = old_w1 - LEARNING_RATE*dE_total_d_w1;
            new_w2 = old_w2 - LEARNING_RATE*dE_total_d_w2;
            new_b1 = old_b1 - LEARNING_RATE*dE_total_d_b1;
            new_w3 = old_w3 - LEARNING_RATE*dE_total_d_w3;
            new_w4 = old_w4 - LEARNING_RATE*dE_total_d_w4;
            new_b2 = old_b2 - LEARNING_RATE*dE_total_d_b2;
            new_w5 = old_w5 - LEARNING_RATE*dE_total_d_w5;
            new_w6 = old_w6 - LEARNING_RATE*dE_total_d_w6;
            new_b3 = old_b3 - LEARNING_RATE*dE_total_d_b3;

            // Update old weights
            old_w1 = new_w1;
            old_w2 = new_w2;
            old_w3 = new_w3;
            old_w4 = new_w4;
            old_w5 = new_w5;
            old_w6 = new_w6;
            old_w7 = new_w7;
            old_w8 = new_w8;
            old_w9 = new_w9;
            old_w10 = new_w10;
            old_w11 = new_w11;
            old_w12 = new_w12;
            old_b1 = new_b1;
            old_b2 = new_b2;
            old_b3 = new_b3;
            old_b4 = new_b4;
            old_b5 = new_b5;
        }
    }
}

