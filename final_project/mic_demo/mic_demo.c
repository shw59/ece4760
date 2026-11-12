/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * Final Code for bird song synchronization implementation
 * Camille Yap (cy474), Selena Wang (shw59)
 * ECE 4760 Fall 2026
 *
 * KEYPAD CONNECTIONS
 *  - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
 *  - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
 *  - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
 *  - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
 *  - GPIO 13  -->     Pin 5 (button col 1)
 *  - GPIO 14  -->     Pin 6 (button col 2)
 *  - GPIO 15  -->     Pin 7 (button col 3)
 *
 * SERIAL CONNECTIONS
 *  - GPIO 0        -->     UART RX (white)
 *  - GPIO 1        -->     UART TX (green)
 *  - RP2040 GND    -->     UART GND
 */

#include "VGA/vga16_graphics_v3.h"

#include "stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

// ==========================================
// === protothreads
// ==========================================
// protothreads header
#include "pt_cornell_rp2040_v1_4.h"

// Initializing pins
#define LED_PIN 25
#define ADC_PIN 28
#define ADC_MUX 2

// This thread runs on core 0
static PT_THREAD(protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);

    static unsigned int adc_val; // ADC value

    // scan()
    while (1)
    {
        // toggle gpio 25
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        adc_val = adc_read();
        printf("%u\n", adc_val);

        PT_YIELD_usec(10000); // 100Hz polling rate
    }
    // Indicate thread end
    PT_END(pt);
}

int main()
{
    // Overclock
    set_sys_clock_khz(150000, true);

    // Initialize stdio
    stdio_init_all();

    // Initialize the VGA screen
    initVGA() ;

    // Setup the ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    // Map LED to GPIO port, make it low
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // Add core 0 threads
    pt_add_thread(protothread_core_0);

    // Start scheduling core 0 threads
    pt_schedule_start;
}