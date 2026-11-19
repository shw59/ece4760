/**
 * Encoder knob -> stepper motor, with encoder count shown on VGA
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
 *  - GPIO 2 ---> PUL
 *  - GPIO 3 ---> DIR
 *  - GPIO 4 ---> ENA
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
#define PUL_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4

#define ENCODER_A 6
#define ENCODER_B 7
#define SW 8

// ==========================================
// === constants
// ==========================================
#define STEP_PHASE_DELAY 50 // 10kHz pulse frequency (us)
#define DIR_SETUP_US 10 // delay after changing DIR before stepping
#define PULSES_PER_CLICK 16 // microsteps per encoder click (tune this)

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

// ==========================================
// === GPIO ISR (encoder A and button)
// ==========================================
void gpio_callback(uint gpio, uint32_t events)
{
  if (gpio == ENCODER_A)
  {
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