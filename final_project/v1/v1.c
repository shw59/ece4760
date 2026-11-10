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
#define COMPOSE_SIZE 10

volatile int sin_table[sine_table_size]; // sine table of 256 elements for all accumulator / phasor angle values

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
    while (1)
    {
        // toggle gpio 25
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        //=======Sound generation========

        // Read the ADC
        adc_val = adc_read();

        // Convert ADC reading (0-4095) to frequency (0-10,000 Hz)
        frequency = ((uint32_t)adc_val * 10000) / 4095;

        // Update DDS phase increment
        phase_incr_main = (frequency * two32) / Fs;

        // Print the value
        printf("ADC value: %d\n", adc_val);
    }
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
    // Format     (channel,  data bits per transfer, polarity, phase, order)
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
    /*
    Calculates phase angles in one cycle of a sine wave and stores in a 256-entry table.
    Scaled to produce values between 0 and 4096
    */
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

    // Add core 0 threads
    pt_add_thread(protothread_core_0);

    // Start scheduling core 0 threads
    pt_schedule_start;
}