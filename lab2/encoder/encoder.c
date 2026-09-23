/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * Simple GPIO interrupt demo.
 * 
 * Wire GPIO 2 to GPIO 3 (thru a resistor).
 * The code toggles GPIO 3, triggering an ISR
 * at every rising edge. The ISR blinks the LED.
 * 
 * Note that the onboard LED is on GPIO 25.
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define ENCODER_A 3
#define ENCODER_B 4
#define SW 5 //

// GPIO ISR on encoder pin A
void gpio_callback() {
    // Check encoder pin B
    gpio_get(ENCODER_B);
    if ()
}

int main() {
    // Initialize stdio
    stdio_init_all();
    printf("GPIO interrupt\n");

    // Configure GPIO interrupt on encoder pin A
    gpio_init(ENCODER_A);
    gpio_set_dir(ENCODER_A, GPIO_IN);
    gpio_pull_up(ENCODER_A);
    gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback,);

    // Configure GPIO input on one of the button switch pins
    gpio_init(SW);
    gpio_set_dir(SW, GPIO_IN);
    gpio_pull_up(SW);

    while (1) {
        // Display number on VGA that rotate
    }

}
