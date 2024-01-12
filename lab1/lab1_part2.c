/* 
 * Name: Luisa Chiu and Asa Grote
 * Assignment: Lab 1 part 2
 * Description: This program scrolls our full name across the LCD display left-to-right, such that holding down the user button
 * switches the name to that of the lab partner, and letting go switches the name back to the other person.
*/ 

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

int main(void){
    init();  //initialize board hardware
    uint8_t col = 0;
    while (1){
        // if button is held down
        if (get_btn()){
            // clear screen to start new name scrolling
            clear_screen();
            _delay_us(200000);
            // start scroll to start at the far left of the screen
            col = 0;
            while (1){
                // start printing first name in the first row, first column
                lcd_cursor(col, 0);
                print_string("Asa");
                // start printing last name in the second row, first column
                lcd_cursor(col, 1);
                print_string("Grote");
                _delay_us(200000);
                // increase number of columns to scroll name
                col++;
                // if column reaches past screen, reset scrolling to the far left of the screen
                if (col >= 9){
                    col = 0;
                }
                _delay_us(200000);
                clear_screen();
                // if button is released, break out of while loop and go to the else if statement
                if (!get_btn()){
                    break;
                }
            }
        }
        // if button is not held down
        else if (!get_btn()){
            // clear screen to start new name scrolling
            clear_screen();
            _delay_us(200000);
            // start scroll to start at the far left of the screen
            col = 0;
            while (1){
                // start printing first name in the first row, first column
                lcd_cursor(col, 0);
                print_string("Luisa");
                // start printing last name in the second row, first column
                lcd_cursor(col, 1);
                print_string("Chiu");
                _delay_us(200000);
                // increase number of columns to scroll name
                col++;
                // if column reaches past screen, reset scrolling to the far left of the screen
                if (col >= 9){
                    col = 0;
                }
                _delay_us(200000);
                clear_screen();
                // if button is released, break out of while loop and go to the else if statement
                if (get_btn()){
                    break;
                }
            }
        }
    }
}