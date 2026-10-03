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

#define ENCODER_A 26
#define ENCODER_B 27
#define SW 28

volatile bool b_value;
volatile int count = 0; // counter for measuring orientation - +1 for clockwise, -1 for counter-clockwise
volatile int bounciness = 0.3;
volatile int sw_state = 0;
// GPIO ISR on encoder pin As
void gpio_callback(uint gpio, uint32_t events)
{
    // Check encoder pin B
    b_value = gpio_get(ENCODER_B);
    // sw_value = gpio_callback
    if (gpio == ENCODER_A)
    {
        if (b_value)
        { // clockwise
            gpio_put(25, !gpio_get(25));
            if (sw_state == 1)
            {
                if (bounciness < 1)
                {
                    bounciness += 0.01;
                }
            }
            else
            {
                count += 1;
            }
        }
        else
        { // counter-clockwise
            if (bounciness > 0)
            {
                bounciness -= 0.01;
            }
            else
                count -= 1;
        }
    }
    else if (gpio == SW)
    {
        sw_state = !sw_state;
    }
}

int main()
{
    // Initialize stdio
    stdio_init_all();
    printf("GPIO interrupt\n");

    // Configure GPIO interrupt on encoder pin A
    gpio_init(ENCODER_A);
    gpio_set_dir(ENCODER_A, GPIO_IN);
    gpio_pull_up(ENCODER_A);
    gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Configure GPIO interrupt on encoder pin B
    gpio_init(ENCODER_B);
    gpio_set_dir(ENCODER_B, GPIO_IN);
    gpio_pull_up(ENCODER_B);

    // Configure GPIO input on one of the button switch pins
    gpio_init(SW);
    gpio_set_dir(SW, GPIO_IN);
    gpio_pull_up(SW);
    gpio_set_irq_enabled_with_callback(SW, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    while (1)
    {
        // Display number on VGA that increments when rotating clockwise and decrements when rotating counterclockwise
        printf("count: %d\n", count);
        printf("bounciness: %d\n", bounciness);
        // sleep_ms(500);
    }
}
