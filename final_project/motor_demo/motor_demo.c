/**
 * Encoder knob -> stepper motor, with encoder count and motor status shown on VGA
 *
 * VGA CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - GND ---> VGA GND
 *
 * STEPPER DRIVER
 *  - GPIO 4 ---> PUL
 *  - GPIO 3 ---> DIR
 *  - GPIO 2 ---> ENA
 *
 * ENCODER
 *  - GPIO 6 ---> Encoder A
 *  - GPIO 7 ---> Encoder B
 *  - GPIO 8 ---> Encoder push button
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/clocks.h"
#include "hardware/gpio.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "pt_cornell_rp2040_v1_4.h"

// ==========================================
// === pins
// ==========================================
#define PUL_PIN 4
#define DIR_PIN 3
#define ENA_PIN 2

#define ENCODER_A 6
#define ENCODER_B 7
#define SW 8

// ==========================================
// === constants
// ==========================================
#define STEP_PHASE_DELAY 50 // 10kHz pulse frequency (us)
#define DIR_SETUP_US 10 // delay after changing DIR before stepping
#define PULSES_PER_CLICK 16 // microsteps per encoder click (tune this)

// Software debouncing: edges closer together than this are ignored
#define ENC_DEBOUNCE_US 2000 // encoder A (increase if you still see double counts)
#define SW_DEBOUNCE_US 50000 // push button (50 ms)

// ==========================================
// === startup motor test (temporary)
// ==========================================
#define RUN_MOTOR_TEST 1 // set to 0 to skip the test
#define TEST_PULSES_PER_REV 3200 // set to match pulse/rev dipswitch setting
#define TEST_HALF_PERIOD_US 1000 // delay of each high/low toggle of the pulse line

// ==========================================
// === shared data
// ==========================================
volatile int32_t count = 0; // encoder clicks (+ clockwise, - counter-clockwise)
volatile int32_t dir = 0; // tracks current dirction motor is set to rotate
volatile int32_t target = 0; // desired position in microsteps
volatile bool motor_enabled = true; // toggled by the button
static int32_t position = 0; // tracks microsteps taken since program start

// ==========================================
// === motor helper functions
// ==========================================
static void motor_set_enabled(bool on)
{
  gpio_put(ENA_PIN, (int)(!on));
}

static void motor_init(void)
{
  gpio_init(PUL_PIN);
  gpio_set_dir(PUL_PIN, GPIO_OUT);
  gpio_put(PUL_PIN, 0);

  gpio_init(DIR_PIN);
  gpio_set_dir(DIR_PIN, GPIO_OUT);
  gpio_put(DIR_PIN, dir);

  gpio_init(ENA_PIN);
  gpio_set_dir(ENA_PIN, GPIO_OUT);
  motor_set_enabled(true);
}

// One microstep in the given direction
static void step_once(bool desired_dir)
{
  if ((int)desired_dir != dir)
  {
    gpio_put(DIR_PIN, desired_dir);
    dir = desired_dir;
    sleep_us(DIR_SETUP_US);
  }
  gpio_put(PUL_PIN, 1);
  sleep_us(STEP_PHASE_DELAY);
  gpio_put(PUL_PIN, 0);
  sleep_us(STEP_PHASE_DELAY);
}

// quick test: one revolution forward, pause, one revolution back
static void motor_test(void)
{
  printf("Motor test: starting in 2 s\n");
  sleep_ms(2000);

  motor_set_enabled(true);
  sleep_ms(100); // give the driver time to enable

  for (int d = 1; d >= 0; d--)
  {
    gpio_put(DIR_PIN, d);
    dir = d;
    sleep_us(DIR_SETUP_US);

    for (int i = 0; i < TEST_PULSES_PER_REV; i++)
    {
      gpio_put(PUL_PIN, 1);
      sleep_us(TEST_HALF_PERIOD_US);
      gpio_put(PUL_PIN, 0);
      sleep_us(TEST_HALF_PERIOD_US);
    }
    sleep_ms(1000);
  }
  printf("Motor test: done\n");
}

// ==========================================
// === GPIO ISR (encoder A and button)
// ==========================================
void gpio_callback(uint gpio, uint32_t events)
{
  static uint32_t last_a_us = 0;
  static uint32_t last_sw_us = 0;
  uint32_t now = time_us_32();

  if (gpio == ENCODER_A)
  {
    // debouncing: ignore edges that arrive too soon after the last one
    if (now - last_a_us < ENC_DEBOUNCE_US)
      return;
    last_a_us = now;

    if (gpio_get(ENCODER_B))
    { // clockwise
      count += 1;
      target += PULSES_PER_CLICK;
    }
    else
    { // counter-clockwise
      count -= 1;
      target -= PULSES_PER_CLICK;
    }
  }
  else if (gpio == SW)
  {
    if (now - last_sw_us < SW_DEBOUNCE_US)
      return;
    last_sw_us = now;

    motor_enabled = !motor_enabled;
    motor_set_enabled(motor_enabled);
  }
}

// ==========================================
// === draw thread (runs on core 1)
// ==========================================
static PT_THREAD(protothread_draw(struct pt *pt))
{
  PT_BEGIN(pt);

  static char title_str[32];
  static char status_str[32];
  static char count_str[16];

  copy_buffer_to_other();

  while (1)
  {
    // Wait for a buffer swap
    PT_YIELD_UNTIL(pt, draw_start_signal());

    // Clear the buffer
    clearLowFrame(0, BLACK);

    // Title
    sprintf(title_str, "Encoder count");
    drawTextAscii(10, 10, title_str, WHITE, BLACK);

    // Motor enable status
    if (motor_enabled)
    {
      sprintf(status_str, "Motor: ENABLED");
      drawTextAscii(10, 22, status_str, GREEN, BLACK);
    }
    else
    {
      sprintf(status_str, "Motor: DISABLED");
      drawTextAscii(10, 22, status_str, RED, BLACK);
    }

    // Big count
    sprintf(count_str, "%ld", (long)count);
    setTextSize(6);
    setTextColor(WHITE);
    setCursor(10, 100);
    writeString(count_str);
    setTextSize(1);
  }

  PT_END(pt);
}

// ==========================================
// === motor thread (runs on core 0)
// ==========================================
static PT_THREAD(protothread_motor(struct pt *pt))
{
  PT_BEGIN(pt);

  while (1)
  {
    // Wait until the motor is behind the target
    PT_YIELD_UNTIL(pt, position != target);

    if (!motor_enabled)
    {
      // Driver is released, so drop any pending steps
      position = target;
    }
    else
    {
      bool desired_dir = target > position;
      step_once(desired_dir);
      position += desired_dir ? 1 : -1;
    }
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

  // Setup the motor pins
  motor_init();

#if RUN_MOTOR_TEST
  motor_test();
#endif

  // Encoder pins (A is the interrupt, B is just read inside the ISR)
  gpio_init(ENCODER_A);
  gpio_set_dir(ENCODER_A, GPIO_IN);
  gpio_pull_up(ENCODER_A);

  gpio_init(ENCODER_B);
  gpio_set_dir(ENCODER_B, GPIO_IN);
  gpio_pull_up(ENCODER_B);

  // Button
  gpio_init(SW);
  gpio_set_dir(SW, GPIO_IN);
  gpio_pull_up(SW);

  // One shared callback for both interrupts
  gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled(SW, GPIO_IRQ_EDGE_FALL, true);

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  // Add core 0 threads
  pt_add_thread(protothread_motor);

  // Start scheduling core 0 threads
  pt_schedule_start;
}