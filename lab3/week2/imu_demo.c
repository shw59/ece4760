/**
 * V. Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
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
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// We will start drawing at column 81
#define LEFT_EDGE 81

// Rescale the measurements for display
#define OLD_RANGE 500. // (+/- 250)
#define NEW_RANGE 150. // (looks nice on VGA)
#define OLD_MIN -250.
#define OLD_MAX 250.
#define ACCEL_SCALE 120.0
#define CTRL_SCALE 0.0833
#define ANGLE_MIN -90.
#define ANGLE_RANGE 180.

// Place the plots
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_BOTTOM 430
#define PLOT_WIDTH 530

// Button
#define BUTTON 26

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Arrays for circular display buffer
fix15 beam_angle[PLOT_WIDTH];
fix15 motor_sig[PLOT_WIDTH];

// Pointer to head of buffer
volatile int head = 0;

// character array
char screentext[40];

// draw speed
volatile int threshold = 10;

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// semaphore
static struct pt_sem vga_semaphore;

// Some paramters for PWM
#define WRAPVAL 6000
#define CLKDIV 25.0

// Clamp parameter for Integrator Windup
#define INT_WIND_UP 500

void drawAxes()
{
  // Draw the static aspects of the display
  setTextSize(1);
  setTextColor(WHITE);

  // Draw bottom plot
  drawHLine(75, 430, 5, CYAN);
  drawHLine(75, 355, 5, CYAN);
  drawHLine(75, 280, 5, CYAN);
  drawVLine(80, 280, 150, CYAN);
  sprintf(screentext, "0");
  setCursor(50, 350);
  writeString(screentext);
  sprintf(screentext, "+90");
  setCursor(45, 280);
  writeString(screentext);
  sprintf(screentext, "-90");
  setCursor(45, 425);
  writeString(screentext);

  // Draw top plot
  drawHLine(75, 230, 5, CYAN);
  drawHLine(75, 155, 5, CYAN);
  drawHLine(75, 80, 5, CYAN);
  drawVLine(80, 80, 150, CYAN);
  sprintf(screentext, "3000");
  setCursor(50, 150);
  writeString(screentext);
  sprintf(screentext, "6000");
  setCursor(45, 75);
  writeString(screentext);
  sprintf(screentext, "0");
  setCursor(45, 225);
  writeString(screentext);
}

// PWM duty cycle
volatile int control;
volatile int old_control;
volatile int Kp;
volatile int Kd;
volatile int Ki;
volatile float error;
volatile float cumulative_error = 0;
volatile float desired_angle = 0;
volatile float prev_error = 0;
uint slice_num;
volatile fix15 filtered_control = 0;

// complementary filter and VGA plotting variables
volatile int throttle = 10;
volatile fix15 accel_angle;
volatile fix15 gyro_angle;
volatile fix15 filtered_ax = 0;
volatile fix15 filtered_ay = 0;
volatile fix15 filtered_az = 0;
volatile fix15 complementary_angle;
volatile fix15 gyro_angle_delta;

volatile uint64_t start_us;
volatile bool run_sequence = false;
void gpio_callback(uint gpio, uint32_t events)
{
  run_sequence = true;
  start_us = time_us_64();
}

// Interrupt service routine
void on_pwm_wrap()
{

  // Clear the interrupt flag that brought us here
  pwm_clear_irq(pwm_gpio_to_slice_num(5));
  gpio_put(14, !gpio_get(14));

  // Read the IMU
  // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
  // If you want these values in floating point, call fix2float15() on
  // the raw measurements.
  mpu6050_read_raw(acceleration, gyro);

  filtered_ax = filtered_ax + ((acceleration[0] - filtered_ax) >> 6); // ax
  filtered_ay = filtered_ay + ((acceleration[1] - filtered_ay) >> 6); // ay
  filtered_az = filtered_az + ((acceleration[2] - filtered_az) >> 6); // az

  // Accelerometer angle (degrees - 15.16 fixed point)
  // SMALL ANGLE APPROXIMATION
  // accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), oneeightyoverpi);
  // NO SMALL ANGLE APPROXIMATION
  accel_angle = multfix15(float2fix15(atan2(-fix2float15(filtered_ax), fix2float15(filtered_az))), oneeightyoverpi);

  // // Gyro angle delta (measurement times timestep) (15.16 fixed point)
  gyro_angle_delta = multfix15(gyro[1], zeropt001);

  // // Complementary angle (degrees - floating point)
  complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);
  if (run_sequence)
  {
    uint64_t elapsed_time = (time_us_64() - start_us) / 1000000;
    if (elapsed_time < 5)
    {
      desired_angle = 0.0;
    }
    else if (elapsed_time < 10)
    {
      desired_angle = 30.0;
    }
    else if (elapsed_time < 15)
    {
      desired_angle = -30.0;
    }
    else if (elapsed_time > 15)
    {
      desired_angle = 0;
      run_sequence = false;
    }
  }
  // PID
  // calculate beam angle error
  float curr_angle = fix2float15(complementary_angle);
  error = desired_angle - curr_angle;

  // calculate cumulative error
  cumulative_error += error * zeropt001; /////////// check if delta t is zeropt001

  // calculate motor signal via PID formula
  control = (Kp * error) + (Ki * cumulative_error) + (Kd * (error - prev_error) / zeropt001); // (Kd * (error ))
  prev_error = error;

  // clamp signal from 0-6000
  float u = Kp * error;
  if (u > WRAPVAL)
  {
    u = WRAPVAL;
  }
  else if (u < 0)
  {
    u = 0;
  }
  control = (int)u;

  // lowpass filter for VGA
  filtered_control = filtered_control + ((int2fix15(control) - filtered_control) >> 6);

  // Update duty cycle
  if (control != old_control)
  {
    old_control = control;
    pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
  }

  // Increment drawspeed controller
  throttle += 1;
  // Maintain a circular buffer for display
  if ((throttle >= threshold))
  {
    // Zero drawspeed controller
    throttle = 0;

    // Replace oldest data in arrays with new sample
    beam_angle[head] = complementary_angle;
    motor_sig[head] = filtered_control;

    // Move the pointer to the oldest data
    head = ((head + 1) == PLOT_WIDTH) ? 0 : (head + 1);
  }
}

static PT_THREAD(protothread_draw(struct pt *pt))
{
  PT_BEGIN(pt);

  static int start;
  static int i;

  // Draw axes
  drawAxes();
  // Copy to other buffer
  copy_buffer_to_other();

  while (1)
  {
    // Wait for a buffer swap
    PT_YIELD_UNTIL(pt, draw_start_signal());
    gpio_put(15, !gpio_get(15));

    // Clear the buffer, draw the axes
    clearLowFrame(0, BLACK);
    drawAxes();

    // Plot data oldest-->newest
    start = head;
    for (i = 0; i < PLOT_WIDTH; i++)
    {
      // up   - negative x acceleration
      // down - positive x acceleration
      // up   - positive y gyro
      // down - negative y gyro
      printf("Desired Angle: %d/n", desired_angle);
      printf("Complementary Angle: %d/n", complementary_angle);
      printf("Kp, Ki, Kd: %d, %d, %d/n", Kp, Ki, Kd);
      drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM - (int)(NEW_RANGE * ((float)((fix2float15(beam_angle[start])) - ANGLE_MIN) / ANGLE_RANGE))), BLUE);
      drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM - (int)(NEW_RANGE * ((float)(fix2float15(motor_sig[start]) * CTRL_SCALE) / OLD_RANGE))), RED); // drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM - (int)(NEW_RANGE * ((float)((fix2float15(ay[start]) * ACCEL_SCALE) - OLD_MIN) / OLD_RANGE))), RED);
      // drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM - (int)(NEW_RANGE * ((float)((fix2float15(az[start]) * ACCEL_SCALE) - OLD_MIN) / OLD_RANGE))), GREEN);
      // drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM - (int)(NEW_RANGE * ((float)((fix2float15(gx[start])) - OLD_MIN) / OLD_RANGE))), WHITE);
      // drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM - (int)(NEW_RANGE * ((float)((fix2float15(gy[start])) - OLD_MIN) / OLD_RANGE))), RED);
      // drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM - (int)(NEW_RANGE * ((float)((fix2float15(gz[start])) - OLD_MIN) / OLD_RANGE))), GREEN);
      start = ((start + 1) == PLOT_WIDTH) ? 0 : (start + 1);
    }
  }

  PT_END(pt);
}

// User input thread. User can change draw speed lol do we need this
static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  static char classifier;
  static int test_in;
  static float float_in;
  while (1)
  {
    sprintf(pt_serial_out_buffer, "input a command (t, a, p, d, i): ");
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;
    // convert input string to number
    sscanf(pt_serial_in_buffer, "%c", &classifier);

    // num_independents = test_in ;
    if (classifier == 't')
    {
      sprintf(pt_serial_out_buffer, "timestep: ");
      serial_write;
      serial_read;
      // convert input string to number
      sscanf(pt_serial_in_buffer, "%d", &test_in);
      if (test_in > 0)
      {
        threshold = test_in;
      }
    }
    else if (classifier == 'a')
    {
      sprintf(pt_serial_out_buffer, "desired beam angle: ");
      serial_write;
      serial_read;
      sscanf(pt_serial_in_buffer, "%d", &test_in);
      desired_angle = test_in;
    }
    else if (classifier == 'p')
    {
      sprintf(pt_serial_out_buffer, "input PID proportional gain: ");
      serial_write;
      // spawn a thread to do the non-blocking serial read
      serial_read;
      // convert input string to number
      sscanf(pt_serial_in_buffer, "%d", &test_in);
      Kp = test_in;
    }
    else if (classifier == 'd')
    {
      sprinf(pt_serial_out_buffer, "input PID differential gain: ");
      serial_write;
      // spawn a thread to do the non-blocking serial read
      serial_read;
      // convert input string to number
      sscanf(pt_serial_in_buffer, "%d", &test_in);
      Kd = test_in;
    }
    else if (classifier == 'i')
    {
      sprinf(pt_serial_out_buffer, "input PID integral gain: ");
      serial_write;
      // spawn a thread to do the non-blocking serial read
      serial_read;
      // convert input string to number
      sscanf(pt_serial_in_buffer, "%d", &test_in);
      Ki = test_in;
    }
    else
    {
      sprinf(pt_serial_out_buffer, "invalid input, try again");
    }
  }
  PT_END(pt);
}

// Entry point for core 1
void core1_entry()
{
  gpio_init(15);
  gpio_set_dir(15, GPIO_OUT);
  gpio_put(15, 0);
  pt_add_thread(protothread_draw);
  pt_schedule_start;
}

int main()
{

  // Overclock
  set_sys_clock_khz(150000, true);

  // Initialize stdio
  stdio_init_all();

  // Initialize VGA
  initVGA();

  ////////////////////////////////////////////////////////////////////////
  ///////////////////////// I2C CONFIGURATION ////////////////////////////
  i2c_init(I2C_CHAN, I2C_BAUD_RATE);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

  // MPU6050 initialization
  // printf("before reset\n");
  mpu6050_reset();
  // uint8_t buf;
  // int ret68 = i2c_read_timeout_us(I2C_CHAN, 0x68, &buf, 1, false, 10000);
  // int ret69 = i2c_read_timeout_us(I2C_CHAN, 0x69, &buf, 1, false, 10000);
  // printf("probe 0x68: %d, probe 0x69: %d\n", ret68, ret69);
  // printf("after reset\n");
  mpu6050_read_raw(acceleration, gyro);
  // printf("after read\n");

  ////////////////////////////////////////////////////////////////////////
  ///////////////////////// PWM CONFIGURATION ////////////////////////////
  ////////////////////////////////////////////////////////////////////////
  // Tell GPIO's 4,5 that they allocated to the PWM
  gpio_set_function(5, GPIO_FUNC_PWM);
  gpio_set_function(4, GPIO_FUNC_PWM);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  // Configure GPIO interrupt on encoder pin A
  gpio_init(BUTTON);
  gpio_set_dir(BUTTON, GPIO_IN);
  gpio_pull_up(BUTTON);
  gpio_set_irq_enabled_with_callback(BUTTON, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
  slice_num = pwm_gpio_to_slice_num(5);

  // Mask our slice's IRQ output into the PWM block's single interrupt line,
  // and register our interrupt handler
  pwm_clear_irq(slice_num);
  pwm_set_irq_enabled(slice_num, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  // This section configures the period of the PWM signals
  pwm_set_wrap(slice_num, WRAPVAL);
  pwm_set_clkdiv(slice_num, CLKDIV);

  // This sets duty cycle
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

  // Start the channel
  pwm_set_mask_enabled((1u << slice_num));

  ////////////////////////////////////////////////////////////////////////
  ///////////////////////////// ROCK AND ROLL ////////////////////////////
  ////////////////////////////////////////////////////////////////////////
  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  // start core 0
  gpio_init(14);
  gpio_set_dir(14, GPIO_OUT);
  gpio_put(14, 0);
  pt_add_thread(protothread_serial);
  pt_schedule_start;
}
