/**
 * Name: Asa Grote and Luisa Chiu
 * Assignment Lab 1 part 1
 * Description: Gradually fades LED0 on and then off. Repeats this fade indefinitely.
 * Does not fade LED1, since we do not have LED1 on our Bumblebee board.
*/

#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define COUNT 5

void delay_us_variable(u16);

int main(void) {
   init();  //initialize board hardware
   u08 count = 0;
   while(1) {
      // turn LED0 off for 10ms
      led(LED0_PIN, 0);
      _delay_us(100000);

      /**
       * fade LED ON
       * -----------
       * increment from 0 to 10000 (10ms) in increments of 500 (0.5ms).
       * turn LED ON for a period of 0ms initially, up to 10ms (in increments of 0.5ms)
       * at the same time, 
       * turn LED OFF for a period of 10ms initially, down to 0ms (in increments of 0.5ms)
       * This will create the effect of the LED fading ON, because of the variable duty cycles
       * of the LED being ON/OFF
       * 
       * -----------
       * NOTE: this cycle is repeated COUNT times in order to lengthen the 
       * duration of the fade ON
      */
      for (u16 i=0; i<=10000; i += 500) {
         while (count < COUNT) {
            led(LED0_PIN, 1);
            delay_us_variable(i);
            led(LED0_PIN, 0);
            delay_us_variable(10000-i);
            count++;
         }
         count = 0;
      }

      /**
       * fade LED OFF
       * -----------
       * increment from 0 to 10000 (10ms) in increments of 500 (0.5ms).
       * turn LED ON for a period of 10ms initially, down to 0ms (in increments of 0.5ms)
       * at the same time, 
       * turn LED OFF for a period of 0ms initially, up to 10ms (in increments of 0.5ms)
       * This will create the effect of the LED fading OFF, because of the variable duty cycles
       * of the LED being ON/OFF
       * -----------
       * NOTE: this cycle is repeated COUNT times in order to lengthen the 
       * duration of the fade OFF
      */
      for (u16 i=0; i<=10000; i += 500) {
         while (count < COUNT) {
            led(LED0_PIN, 1);
            delay_us_variable(10000-i);
            led(LED0_PIN, 0);
            delay_us_variable(i);
            count++;
         }
         count = 0;
      }
   }
}

/**
 * Function:  delay_us_variable
 * --------------------
 * Delays for a variable number of us.
 * 
 * us: number of us to delay
 * 
 * returns: None
*/
void delay_us_variable(u16 us) {
   for (u16 i = 0; i < us; i++) {
      _delay_us(1);
   }
}