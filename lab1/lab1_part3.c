/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment Lab 1 part 3
 * Description: Simple pong game in which the user must click the button
 * when the LED on either end is lit. When the player misses a click or clicks
 * when neither end LED is lit, the program displays the final delay in ms 
 * on the screen.
*/


#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/**
 * time in milliseconds of initial delay
 * in order to achieve longer delay times, a single (ms) delay is looped a variable number of times
*/
#define INITIAL_DELAY 1000
#define LED_1_PIN 12
#define LED_2_PIN 10
#define LED_3_PIN 8
#define LED_4_PIN 6
#define LED_5_PIN 4
#define BTN_PIN 13
#define PIN_INPUT 0
#define PIN_OUTPUT 1
#define LED_INSIDE 0
#define LED_OUTSIDE 1

#define CONTINUE 1
#define LOSE 0

// u08 delay_and_check(u16, u08);
u08 delay_check_click(u16);
u08 delay_check_no_click(u16);
u16 reduce_delay(u16);

int main(void) {

    init();  //initialize board hardware
    
    // Initialize pin I/O
    digital_dir(LED_1_PIN, PIN_OUTPUT);
    digital_dir(LED_2_PIN, PIN_OUTPUT);
    digital_dir(LED_3_PIN, PIN_OUTPUT);
    digital_dir(LED_4_PIN, PIN_OUTPUT);
    digital_dir(LED_5_PIN, PIN_OUTPUT);
    digital_dir(BTN_PIN, PIN_INPUT);

    /**
     * direction of 1 causes the LEDs to increase in number, i.e. LED3 -> LED4
     * direction of 0 causes the LEDs to decrease in number, i.e. LED4 -> LED3
    */
    u08 dir = 1;

    // Score tracker
    // u08 score = 0;

    typedef enum {
        START_GAME,
        LED1,
        LED2,
        LED3,
        LED4,
        LED5,
        PRINT,
        GAME_OVER
    } state_var_type;

    state_var_type state = START_GAME;
    u16 delay = INITIAL_DELAY;

    while(1) {
        switch(state) {
            case START_GAME:
                state = LED3;
                break;
            case LED1:
                // turn off LED2, turn on LED1
                digital_out(LED_2_PIN, 0);
                digital_out(LED_1_PIN, 1);


                if (delay_check_click(delay)) {
                    // set new delay
                    delay = reduce_delay(delay);
                    // score++;

                    state = LED2;
                    dir = 1; // reverse direction
                        
                }
                else {
                    state = PRINT;
                }
                break;
            case LED2:
                // turn off LED1/3, turn on LED2
                if (dir) {
                    digital_out(LED_1_PIN, 0);
                }
                else {
                    digital_out(LED_3_PIN, 0);
                }
                digital_out(LED_2_PIN, 1);

                if (delay_check_no_click(delay)) {
                    // // set new delay
                    // delay = reduce_delay(delay);

                    if (dir) {
                        state = LED3;
                    }
                    else {
                        state = LED1;
                    }
                }
                else {
                    state = PRINT;
                }
                break;
            case LED3:
                // turn off LED2/4, turn on LED3
                if (dir) {
                    digital_out(LED_2_PIN, 0);
                }
                else {
                    digital_out(LED_4_PIN, 0);
                }
                digital_out(LED_3_PIN, 1);

                if (delay_check_no_click(delay)) {
                    // // set new delay
                    // delay = reduce_delay(delay);
                    
                    if (dir) {
                        state = LED4;
                    }
                    else {
                        state = LED2;
                    }
                }
                else {
                    state = PRINT;
                }
                break;
            case LED4:
                // turn off LED3/5, turn on LED4
                if (dir) {
                    digital_out(LED_3_PIN, 0);
                }
                else {
                    digital_out(LED_5_PIN, 0);
                }
                digital_out(LED_4_PIN, 1);

                if (delay_check_no_click(delay)) {
                    // // set new delay
                    // delay = reduce_delay(delay);

                    if (dir) {
                        state = LED5;
                    }
                    else {
                        state = LED3;
                    }
                }
                else {
                    state = PRINT;
                }
                break;
            case LED5:
                // turn off LED4, turn on LED5
                digital_out(LED_4_PIN, 0);
                digital_out(LED_5_PIN, 1);

                if (delay_check_click(delay)) {
                    // set new delay
                    delay = reduce_delay(delay);
                    // score++;
                    state = LED4;
                    dir = 0; // reverse direction
                }
                else {
                    state = PRINT;
                }
                break;
            case PRINT:
                // print_num(delay);
                clear_screen();
                print_string("GameOver");
                lcd_cursor(0, 1);
                print_num(delay);
                lcd_cursor(4, 1);
                print_string("ms");
                // print_string("Score:");
                // lcd_cursor(6, 1);
                // print_num(score);
                state = GAME_OVER;
                break;
            case GAME_OVER:
                state = GAME_OVER;
                break;
            default:
                state = START_GAME;
                break;
        }
    }

    return 0;
}

/**
 * Function:  delay_check_click
 * --------------------
 * Delays for a variable number of ms. During this delay, each ms, checks if
 * the button has been clicked. 
 * 
 * delay: number of ms to delay
 * 
 * returns: If the button is clicked, return 1 for success.
 *          Otherwise, return 0 for failure.
*/
u08 delay_check_click(u16 delay) {
    u08 ret = 0;
    for (u16 i = 0; i < delay; i++) {
        if (!digital(BTN_PIN)) { // if button is clicked
            ret = 1;    // game continues
        }
        _delay_ms(1);
    }
    return ret;
}

/**
 * Function:  delay_check_no_click
 * --------------------
 * Delays for a variable number of ms. During this delay, each ms, checks if
 * the button has been clicked. 
 * 
 * delay: number of ms to delay
 * 
 * returns: If the button is NOT clicked, return 1 for success.
 *          Otherwise, return 0 for failure.
*/
u08 delay_check_no_click(u16 delay) {
    u08 ret = 1;
    for (u16 i = 0; i < delay; i++) {
        if (!digital(BTN_PIN)) { // if button is clicked
            ret = 0;    // game over
        }
        _delay_ms(1);
    }
    return ret;
}

/**
 * Function:  reduce_delay
 * --------------------
 * Reduces delay using an exponential reducton function: 
 *      newDelay = delay * 0.9
 * 
 * delay: current delay
 * 
 * returns: New delay
*/
u16 reduce_delay(u16 delay) {
    return (u16) delay * 0.9;
}
