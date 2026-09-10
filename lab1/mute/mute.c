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
#define KEYROWS 4
#define NUMKEYS 12
#define NUMKEYS_RECORD 9
#define MAX_SAMPLE_COUNT 3000 // 3000 samples to cover 30 seconds at 100Hz sampling rate of ADC

// States for keypad debouncing
typedef enum {
    NOT_PRESSED,
    MAYBE_PRESSED,
    PRESSED,
    MAYBE_NOT_PRESSED
} key_state_t;

// States for recording 
typedef enum {
    RECORD_NONE,
    RECORD_EN,
    RECORDING,
    PLAYBACK
} record_state_t;

unsigned int keycodes[NUMKEYS] = {0x57, 0x6E, 0x5E, 0x3E, 0x6D,
                                  0x5D, 0x3D, 0x6B, 0x5B, 0x3B,
                                  0x67, 0x37};
unsigned int scancodes[KEYROWS] = {0xE, 0xD, 0xB, 0x7};
unsigned int button = 0x70;

char keytext[40];
int prev_key = 0;

// ==========================================
// === protothreads
// ==========================================
// protothreads header
#include "pt_cornell_rp2040_v1_4.h"

#define LED_PIN 25
#define ADC_PIN 26
#define ADC_MUX 0

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// DDS parameters
#define two32 4294967296.0 // 2^32
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)
// the DDS units:
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (800.0 * two32) / Fs;

// SPI data
uint16_t DAC_data; // output value

// DAC parameters
//  A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI configurations
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// GPIO for timing the ISR
#define ISR_GPIO 2

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];
volatile int freq_table[NUMKEYS_RECORD][MAX_SAMPLE_COUNT]; 

// Alarm ISR
static void alarm_irq(void)
{
    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // DDS phase and sine table lookup
    phase_accum_main += phase_incr_main;
    DAC_data = (DAC_config_chan_A | ((sin_table[phase_accum_main >> 24] + 2048) & 0xffff));

    // Perform an SPI transaction
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);
}

// This thread runs on core 0
static PT_THREAD(protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);

    // Some variables
    static int i;
    static uint32_t keypad;
    static key_state_t curr_state = NOT_PRESSED;
    static record_state_t curr_record_state = RECORD_NONE;
    // static unsigned int count = 0;
    static uint32_t possible;
    static bool mute = true;
    // static bool record_mode = false; // record_mode false implies playback mode
    static unsigned int adc_val;
    static unsigned int frequency;
    static unsigned int rec_sample_count; // active sample counter during recording or playback
    static unsigned int recs_num_samples[NUMKEYS_RECORD]; // stores number of samples recorded per key (1-9)
    static int recording_key = -1; // key (1-9) currently being recorded or played back
    // static unsigned int playback_idx = 0;

    // scan()
    while (1)
    {
        // toggle gpio 25
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

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

        // Read the ADC
        adc_val = adc_read();

        // Convert ADC reading (0-4095) to frequency (0-10 kHz)
        frequency = ((uint32_t)adc_val * 10000) / 4095;

        if (mute) {
            frequency = 0;
        } else if (curr_record_state == PLAYBACK) {
            if (rec_sample_count < rec_sample_counts[recording_key-1]) {
                frequency = freq_table[recording_key-1][rec_sample_count];
                rec_sample_count += 1;
            } else {
                curr_record_state = RECORD_NONE;
                rec_sample_count = 0;
            }
            
        }

        // Update DDS phase increment
        phase_incr_main = (frequency * two32) / Fs;

        printf("DDS Frequency: %u \n", frequency);

        // Enter state machine debouncing
        switch (curr_state) {
            case NOT_PRESSED:
                printf("NOT PRESSED\n");
                if (i != -1) {
                    curr_state = MAYBE_PRESSED;
                    possible = i;
                }

                break;
            
            case MAYBE_PRESSED:
                printf("MAYBE PRESSED\n");
                if (i == possible) {
                    curr_state = PRESSED;
                    if (i == 0) (mute = !mute); // toggle mute/unmute when 0 is pressed
                    else if (i == 10) (curr_record_state = RECORD_EN); // * = record mode on
                    else if (i != 11) { // a key (1-9) is being pressed, so we either want to record if in record mode or playback if a recording exists at that key
                        if (curr_record_state == RECORD_EN) {
                            // start recording
                            curr_record_state = RECORDING;
                        } else if (recs_num_samples[i-1] > 0) { // if there's a recording for this key i, play it back
                            curr_record_state = PLAYBACK;
                            rec_sample_count = 0;
                        }
                        recording_key = i; // save the key currently being recorded/played back
                    }
                } else {
                    curr_state = NOT_PRESSED;
                }

                break;

            case PRESSED:
                printf("PRESSED\n");
                if (i != possible) {
                    curr_state = MAYBE_NOT_PRESSED;
                }

                // currently recording on key i
                if (curr_record_state == RECORDING) {
                    if (rec_sample_count < MAX_SAMPLE_COUNT) {
                        freq_table[recording_key - 1][rec_sample_count] = frequency;
                        rec_sample_count += 1;
                    } else { // reached recording time limit, stop recording
                        curr_record_state = RECORD_NONE;
                        rec_sample_counts[recording_key-1] = rec_sample_count
                        rec_sample_count = 0;
                    }
                    
                }

                break;

            case MAYBE_NOT_PRESSED:
                printf("MAYBE NOT PRESSED\n");
                if (i == possible) {
                    curr_state = PRESSED;
                } else {
                    curr_state = NOT_PRESSED;
                    if (curr_record_state == RECORDING) {
                        // user prematurely unpressed key, stop recording
                        curr_record_state = RECORD_NONE;
                        recs_num_samples[recording_key - 1] = rec_sample_count; // save the recording length
                        rec_sample_count = 0; // reset sample counter
                    }
                }

                break;
        }

        // if (not_mute)
        // {

        //     printf("\n playing the tone");
        //     // Read the ADC
        //     adc_val = adc_read();

        //     // if playing recording
        //     if (recording != -1)
        //     {
        //         printf("\n taking %d frequency", freq_table[recording - 1]);
        //         frequency = freq_table[recording - 1];
        //     }
        //     else
        //     {
        //         printf("\n current frequency");
        //         frequency = ((uint32_t)adc_val * 10000) / 4095;
        //     }

        //     // Update DDS phase increment
        //     phase_incr_main = (frequency * two32) / Fs;

        //     // Print the value
        //     printf("\nADC value: %d\n", adc_val);

        //     // Yield
        //     PT_YIELD_usec(100000);
        // }
        // printf("\nstart scanning the keypad");
        // // Scan the keypad!
        // for (i = 0; i < KEYROWS; i++)
        // {
        //     // Set a row high
        //     gpio_put_masked((0xF << BASE_KEYPAD_PIN),
        //                     (scancodes[i] << BASE_KEYPAD_PIN));
        //     // Small delay required
        //     sleep_us(1);
        //     // Read the keycode
        //     keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);
        //     // Break if button(s) are pressed
        //     if ((~keypad) & button)
        //     {
        //         printf("\nkeypad is pressed");
        //         break;
        //     }
        // }
        // // If we found a button . . . MAYBE PRESSED
        // if ((~keypad) & button)
        // {
        //     // Look for a valid keycode.
        //     for (i = 0; i < NUMKEYS; i++)
        //     {
        //         printf("\n%d", i);
        //         if (keypad == keycodes[i])
        //         { // pressing on i keypad
        //             if (count > 3) // key must be held for this many iterations to not count as a fluke
        //             {
        //                 if (i != possible)
        //                 {
        //                     printf("\npressing different key");
        //                     possible = i;
        //                     i = -1;
        //                     count = 0;
        //                 }
        //                 printf("\npressing on %d", i);
        //                 break;
        //             }
        //             else
        //             {
        //                 printf("\ni: %d and possible: %d", i, possible);
        //                 if (i != possible)
        //                 {
        //                     printf("\npressing different key");
        //                     possible = i;
        //                     i = -1;
        //                     count = 0;
        //                 }
        //                 else
        //                 {
        //                     printf("\ndebouncing");
        //                     count += 1;
        //                     i = -1;
        //                 }
        //             }
        //             break;
        //         }
        //     }
        //     // If we don't find one, report invalid keycode
        //     if (i == NUMKEYS)
        //     {
        //         printf("invalid keycode");
        //         // possible = -1;
        //         (i = -1);
        //         count = 0;
        //     }
        // }
        // // Otherwise, indicate invalid/non-pressed buttons
        // else
        //     (i = -1);

        // // Print key to terminal
        // // PRESSED STATE
        // switch (i)
        // {
        // case 0: // mute toggle
        //     not_mute = !not_mute;
        //     count = 0;
        //     printf("\n0 pressed, not muted: %d", not_mute);
        //     break;

        // case 10: // asterisk
        //     record_mode = !record_mode;
        //     printf(record_mode ? "\nentering record mode" : "\nleaving record mode");
        //     break;
        // case 11: // stop key
        // case -1: // no key currently settled/pressed
        //     if (recording != -1)
        //     { // stop playing recording
        //         printf("\n stop playback of key %d", recording);
        //         recording = -1;
        //     }
        //     break;

        // default:
        //     if (i < 1 || i > 9)
        //         break;
        //     if (record_mode)
        //     {
        //         adc_val = adc_read();
        //         frequency = ((uint32_t)adc_val * 10000) / 4095;
        //         freq_table[i - 1] = frequency;
        //         printf("\nrecorded freq for key %d = %d", i, frequency);
        //     }
        //     else
        //     {
        //         recording = (recording == i) ? -1 : i;
        //         printf("\ntoggle for %d, recording=%d", i, recording);
        //     }
        //     break;
        // }
        // printf("\n%d", i);

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

    // Setup the ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    // Map LED to GPIO port, make it low
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    // === build the sine lookup table =======
    // scaled to produce values between 0 and 4096
    int ii;
    for (ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    gpio_set_dir((BASE_KEYPAD_PIN + 4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 6), GPIO_IN);
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN));
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_up((BASE_KEYPAD_PIN + 4));
    gpio_pull_up((BASE_KEYPAD_PIN + 5));
    gpio_pull_up((BASE_KEYPAD_PIN + 6));

    // Add core 0 threads
    pt_add_thread(protothread_core_0);

    // Start scheduling core 0 threads
    pt_schedule_start;
}
