/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * Takes in microphone input and displays waveform on VGA
 *
 * VGA CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * MICROPHONE
 *  - GPIO 28 ---> Mic output [0-3.3V] (ADC channel 2)
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

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "pt_cornell_rp2040_v1_4.h"

// ==========================================
// === pins
// ==========================================
#define LED_PIN 25
#define ADC_PIN 28
#define ADC_MUX 2

// ==========================================
// === sampling / plot parameters
// ==========================================
#define SAMPLE_RATE_HZ 20000
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)

// Only every DECIMATION-th sample is stored for display
// (like "threshold" in the PID code). Time shown on screen =
// PLOT_WIDTH * DECIMATION / SAMPLE_RATE_HZ seconds.
#define DECIMATION 30

// We will start drawing at column 81
#define LEFT_EDGE 81
// Place the plot
#define PLOT_WIDTH 530
#define PLOT_TOP 80
#define PLOT_BOTTOM 430
#define PLOT_HEIGHT (PLOT_BOTTOM - PLOT_TOP)

// ==========================================
// === shared data
// ==========================================
// Circular display buffer (raw 12-bit ADC values)
uint16_t mic_buf[PLOT_WIDTH];

// Pointer to head of buffer (oldest sample)
volatile int head = 0;

// Decimation counter ("throttle" in the PID code)
volatile int throttle = 0;

// Most recent sample
volatile uint16_t latest_sample = 0;

// character array
char screentext[40];

static struct repeating_timer sample_timer;

// Display gain: how many times to magnify the signal around the DC bias
#define DISPLAY_GAIN 8
#define ADC_MID 2048

static inline int adc_to_y(uint16_t v)
{
  int centered = ((int)v - ADC_MID) * DISPLAY_GAIN;  // zero-centered, magnified
  int y = (PLOT_TOP + PLOT_HEIGHT / 2) - ((centered * PLOT_HEIGHT) >> 12);
  if (y < PLOT_TOP) y = PLOT_TOP;                    // clamp to the plot area
  if (y > PLOT_BOTTOM) y = PLOT_BOTTOM;
  return y;
}

// ==========================================
// === draw the static aspects of the display
// ==========================================
void drawAxes()
{
  setTextSize(1);
  setTextColor(WHITE);

  // Vertical axis and ticks (0, 2048, 4095)
  drawVLine(LEFT_EDGE - 1, PLOT_TOP, PLOT_HEIGHT, CYAN);
  drawHLine(LEFT_EDGE - 6, PLOT_BOTTOM, 5, CYAN);
  drawHLine(LEFT_EDGE - 6, PLOT_TOP + PLOT_HEIGHT / 2, 5, CYAN);
  drawHLine(LEFT_EDGE - 6, PLOT_TOP, 5, CYAN);

  sprintf(screentext, "0");
  setCursor(LEFT_EDGE - 30, PLOT_BOTTOM - 4);
  writeString(screentext);
  sprintf(screentext, "2048");
  setCursor(LEFT_EDGE - 50, PLOT_TOP + PLOT_HEIGHT / 2 - 4);
  writeString(screentext);
  sprintf(screentext, "4095");
  setCursor(LEFT_EDGE - 50, PLOT_TOP - 4);
  writeString(screentext);
}

// ==========================================
// === sampling ISR (runs on core 0)
// ==========================================
bool sample_callback(struct repeating_timer *t)
{
  uint16_t val = adc_read();
  latest_sample = val;

  // Increment drawspeed controller
  throttle += 1;
  // Maintain a circular buffer for display
  if (throttle >= DECIMATION)
  {
    // Zero drawspeed controller
    throttle = 0;

    // Replace oldest data in array with new sample
    mic_buf[head] = val;

    // Move the pointer to the oldest data
    head = ((head + 1) == PLOT_WIDTH) ? 0 : (head + 1);
  }
  return true; // keep repeating
}

// ==========================================
// === draw thread (runs on core 1)
// ==========================================
static PT_THREAD(protothread_draw(struct pt *pt))
{
  PT_BEGIN(pt);

  static int start;
  static int i;
  char info_str[64];
  char title_str[64];

  // Draw axes
  drawAxes();
  // Copy to other buffer
  copy_buffer_to_other();

  while (1)
  {
    // Wait for a buffer swap
    PT_YIELD_UNTIL(pt, draw_start_signal());

    // Clear the buffer, draw the axes
    clearLowFrame(0, BLACK);
    drawAxes();

    // Draw text
    sprintf(title_str, "Mic signal: %d Hz, %d ms window",
            SAMPLE_RATE_HZ / DECIMATION,
            (int)((1000L * PLOT_WIDTH * DECIMATION) / SAMPLE_RATE_HZ));
    sprintf(info_str, "Latest ADC: %d", (int)latest_sample);
    printf("%u\n", latest_sample);
    drawTextAscii(10, 10, title_str, WHITE, BLACK);
    drawTextAscii(10, 22, info_str, WHITE, BLACK);

    // Plot data oldest-->newest
    start = head;
    for (i = 0; i < PLOT_WIDTH; i++)
    {
      drawPixel(LEFT_EDGE + i, adc_to_y(mic_buf[start]), GREEN);
      start = ((start + 1) == PLOT_WIDTH) ? 0 : (start + 1); // next value in the data array
    }
  }

  PT_END(pt);
}

// ==========================================
// === core 0 thread: LED heartbeat
// ==========================================
static PT_THREAD(protothread_core_0(struct pt *pt))
{
  PT_BEGIN(pt);

  while (1)
  {
    // toggle gpio 25
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    PT_YIELD_usec(500000);
  }

  PT_END(pt);
}

// Entry point for core 1
void core1_entry()
{
  pt_add_thread(protothread_draw);
  pt_schedule_start;
}

int main()
{
  // Overclock
  set_sys_clock_khz(150000, true);

  // Initialize stdio
  stdio_init_all();

  // Initialize the VGA screen
  initVGA();

  // Setup the ADC
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(ADC_MUX);

  // Map LED to GPIO port, make it low
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 0);

  // Start the display buffer at mid-scale (flat line)
  for (int k = 0; k < PLOT_WIDTH; k++)
  {
    mic_buf[k] = 2048;
  }

  // Start the sampling ISR on core 0
  add_repeating_timer_us(-SAMPLE_PERIOD_US, sample_callback, NULL, &sample_timer);

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  // Add core 0 threads
  pt_add_thread(protothread_core_0);

  // Start scheduling core 0 threads
  pt_schedule_start;
}