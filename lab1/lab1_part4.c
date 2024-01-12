/* 
 * Name: Luisa Chiu and Asa Grote
 * Assignment: Lab 1 part 4
 * Description: This program scrolls the string, "416" across the LCD display according to the tilt of the board; x-direction corresponds to the columns
 * and y-direction corresponds to the rows.
*/ 

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

int main(void){
    init();  //initialize board hardware
    u08 row = 0;
    u08 col = 0;
    u08 y_accel;
    u08 x_accel;
    while(1){
        lcd_cursor(col, row);
        print_string("416");
        x_accel = get_accel_x();
        y_accel = get_accel_y();
        _delay_us(80000);
        // threshold is 127, since u08 data types ranges from 0-255
        if (x_accel > 127){
            // as long as the row is less than 1 and tilt is in positive x-dir, increase row by 1
            if (row < 1){
                row++;        
                clear_screen();
            }
        }
        else if (x_accel < 127){
            // as long as the row is greater than 0 and tilt is in negative x-dir, decrease row by 1
            if (row > 0){
                row--;        
                clear_screen();
            }
        }
        // if statement to also check for y acceleration
        if (y_accel < 127){
            // as long as the column is greater than 1 and tilt is in negative y-dir, decrease row by 1
            if (col >= 1){
                col--;        
                clear_screen();
            }
        }
        else if (y_accel > 127){
            // as long as the column is less than 5 and tilt is in positive y-dir, increment row by 1
            if (col < 5){
                col++;    
                clear_screen();
            }
        }
    }
}