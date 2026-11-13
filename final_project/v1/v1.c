// include FFT library
#include "f_Audio_FFT/pt_cornell_rp2040_v1_4.h"
// include standard libraries
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

// Place ADC samples from DMA here
uint8_t sample_array[NUM_SAMPLES];
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

typedef struct
{
    int16_t sample_size;   // size of the audio buffer
    int16_t W;             // integration window size
    float *diff;           // d_t difference function
    float *cmndf_diff;     // d'_t difference function
    float abs_threshold, ; // absolute threshold (step 4 in paper)
    float periodicity;     //
    int16_t tau_max;       // max lag size
} Yin;

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

// Difference Function (Step 2 of the paper)
void yin_diff(Yin *yin, int16_t *x)
{
    float sum;
    // equation 6
    for (int tau = 0; tau < yin->tau_max; tau++) // searching through values of lag (tau)
    {
        for (int j = 0; j < yin->W; j++)
        {
            sum = x[j] - x[j + tau];
            yin->diff[tau] += sum * sum;
            yin->cmndf_diff[tau] += sum * sum;
        }
    }
}

// Cumulative Mean Normalized Difference Function (Step 3)
void yin_cmndf(Yin *yin)
{
    // replacing diff function with CMD function d'
    float sum;
    yin->cmndf_diff[0] = 1;

    for (int tau = 1; tau < yin->tau_max; tau++)
    {
        sum += yin->cmndf_diff[tau];
        yin->cmndf_diff[tau] *= (tau / sum);
    }
}

// Absolute Threshold (step 4) set an abs threshold + choose smallest tau that gives min of d' deeper than threshold
int16_t yin_threshold(Yin *yin)
{
    int tau;
    for (tau = 0; tau < yin->tau_max; tau++)
    {
        if (yin->cmndf_diff[tau] < yin->abs_threshold)
        { // smaller tau than abs threshold
            while ((tau + 1 < yin->tau_max) && (yin->tau > yin->cmndf_diff[tau + 1]))
            {
                // keep going to find the local minimum of d' (will stop as soon as it starts increasing)
                tau += 1;
            }

            // we break here as we found the
            // "smallest value of tau that gives a minimum of d' deeper than"
            // the abs threshold

            /*
                according to the paper:
                "d'(T) is proportional to the aperiodic/total power ratio.
                A candidate T is accepted in the set if this ratio
                is below threshold."

                this could be helpful for tuning as the threshold dictates
                what is accepted or not.
                low periodicity indicates not very stable note
                high periodicity indicates stable note
            */
            yin->periodicity = 1 - yin->cmndf_diff[tau];
            break;
        }
    }

    // if none is found, we pick global minimum
    if (tau == yin->tau_max || yin->cmndf_diff[tau] >= yin->abs_threshold)
    {
        yin->periodicity = 0;
        // for now instead of finding global min (may keep this b/c maybe it's best to reject it)
        tau = -1;
    }
}

// Parabolic Interpolation: Step 5
// Solves possible incremental error issue when period is not amultiple of the sampling period
float yin_parabolic(Yin *yin, int16_t tau_est)
{
    // immediate neighors of tau_est
    int16_t left_n = tau_est < 1 ? tau_est : tau_est - 1;
    int16_t right_n = (tau_est + 1 > yin->tau_max) ? tau_est : tau_est + 1;
    int16_t interpolated_tau;

    // parabolical interpolation to find a better estimate of tau
    if (left_n == tau_est) // not parabola
        // pick the smallest raw d function value
        interpolated_tau = (yin->diff[tau_est] <= yin->diff[right_n]) ? tau_est : right_n;

    else if (right_n == tauEstimate) // not parabola
        // pick the smallest raw d function value
        interpolated_tau = (yin->diff[tau_est] <= yin->diff[left_n]) ? tau_est : left_n;
    else // parabola
    {
        float l = yin->cmndf_diff[left_n];
        float m = yin->yin_cmndf[tau_est];
        float r = yin->yin_cmndf[right_n];

        /*
        say l = -1; m = 0; r = 1. find a, b, and c from y = ax^2 + bx + c
        using x in {left, mid, right}.
        m = c
        l = a - b + c
        r = a + b + c

        => l + r = 2a + 2c
        => (l + r - m) / 2 = a
        => (r - m - (l + r - m) / 2) = b
        => (r - l) / 2 = b

        find minimum so -b/2a
        => (r - l) / (2 * (2 * m - r - l))
        */
        interpolated_tau = (r - l) / (4 * m - 2 * r - 2 * l) + tau_est;
    }
    return interpolated_tau;
}

// This thread runs on core 0
static PT_THREAD(protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);

    while (1)
    {
    }
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
    sample_chan = dma_claim_unused_channel(true);
    control_chan = dma_claim_unused_channel(true);

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