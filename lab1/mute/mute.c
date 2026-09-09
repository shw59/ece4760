/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * mute implementatioin
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

#include "pt_cornell_rp2040_v1_4.h"
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


// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[NUMKEYS] = {      0x57, 0x6E, 0x5E, 0x3E, 0x6D,
                                        0x5D, 0x3D, 0x6B, 0x5B, 0x3B,
                                        0x67, 0x37} ;
unsigned int scancodes[KEYROWS] = {   0xE, 0xD, 0xB, 0x7} ;
unsigned int button = 0x70 ;


char keytext[40];
int prev_key = 0;

// ==========================================
// === protothreads ADC
// ==========================================
// protothreads header
#include "pt_cornell_rp2040_v1_4.h"

#define LED_PIN 25
#define ADC_PIN 26
#define ADC_MUX 0

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

//DDS parameters
#define two32 4294967296.0 // 2^32 
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)
// the DDS units:
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (800.0*two32)/Fs ;

// SPI data
uint16_t DAC_data ; // output value

//DAC parameters
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

//GPIO for timing the ISR
#define ISR_GPIO 2

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size] ;

// Alarm ISR
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

	// DDS phase and sine table lookup
	phase_accum_main += phase_incr_main  ;
    DAC_data = (DAC_config_chan_A | ((sin_table[phase_accum_main>>24] + 2048) & 0xffff))  ;

    // Perform an SPI transaction
    spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);

}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ;
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if ((~keypad) & button) break ;
        }
        // If we found a button . . .
        if ((~keypad) & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;

        // Print key to terminal
        printf("\n%d", i) ;

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}


int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    gpio_set_dir((BASE_KEYPAD_PIN+4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+6), GPIO_IN);
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_up((BASE_KEYPAD_PIN+4)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+5)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+6)) ;

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}
