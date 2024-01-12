#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define MIN_EPOCH 1
#define MAX_EPOCH 1000

void delay_us_variable(u16);

int main(void) {
    init();  //initialize board hardware

    set_servo(0, 127);
    set_servo(1, 127);

    u08 accel;
    u16 num_epochs = 400;
    u16 epoch_delta;
    int16_t new_epochs;
   
    while(!get_btn()) {
        accel = get_accel_y();

        // epoch_delta = (u16) (accel * MAX_EPOCH / 127);

        // epoch_delta = 1;
        
        if (accel > 127) {
            if (num_epochs + 1 <= MAX_EPOCH) {
                num_epochs+= 1;
            }
            else {
                num_epochs = MAX_EPOCH;
            }
        }
        else if (accel < 127) {
            new_epochs = num_epochs - 1;
            if ( new_epochs >= MIN_EPOCH) {
                num_epochs-= 1;
            }
            else {
                num_epochs = MIN_EPOCH;
            }
        }

        clear_screen();
        print_num((u16)accel);
        lcd_cursor(0, 1);
        print_num(num_epochs);
        _delay_ms(25);
        
    }
      
    return 0;
}
