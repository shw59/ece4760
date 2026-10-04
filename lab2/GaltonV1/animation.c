
/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "VGA/vga16_graphics_v3.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/sync.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short *address_pointer = &DAC_data[0];

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

// SPI configurations
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Encoder
#define ENCODER_A 26
#define ENCODER_B 27
#define SW 28

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0f)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0f)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b > int2fix15(480))
#define hitTop(b) (b < int2fix15(0))
#define hitLeft(a) (a < int2fix15(0))
#define hitRight(a) (a > int2fix15(640))
#define sqrtfix(a) (float2fix15(sqrtf(fix2float15(a))))
// uS per frame
#define FRAME_RATE 33000
#define GRAVITY float2fix15(0.37)
#define BALL_RADIUS int2fix15(3)
#define PEG_RADIUS int2fix15(6)
// #define BOUNCINESS float2fix15(0.3)
#define MAX_NUM_BALLS 8000 // max number of balls that can be spawned
#define HORIZONTAL_SEP int2fix15(38)
#define VERTICAL_SEP int2fix15(19)
#define NUM_LEVELS 16
#define NUM_PEGS (NUM_LEVELS * (NUM_LEVELS + 1) / 2)
#define SPAWN_X int2fix15(320)
#define SPAWN_Y int2fix15(0)
#define VY_INITIAL int2fix15(0)
#define BALL_RADIUS_INT 3
#define PEG_RADIUS_INT 6
#define SEPARATION_DIST (BALL_RADIUS + PEG_RADIUS)
#define HORIZONTAL_SEP_INT 38 // spacing between pegs in the same row
#define VERTICAL_SEP_INT 19   // spacing between rows
#define BIN_LINE_Y_FIX int2fix15(BIN_LINE_Y)
// histogram stuff
#define NUM_BINS (NUM_LEVELS + 1)
#define NUM_GAPS (NUM_BINS - 2)                                                                   // 15 bins between the bottom-row pegs
#define BIN_LEFT_X 35                                                                             // x of leftmost bottom-row peg
#define BIN_RIGHT_X (BIN_LEFT_X + NUM_GAPS * BIN_WIDTH)                                           // 605, x of rightmost bottom-row peg
#define BIN_WIDTH HORIZONTAL_SEP_INT                                                              // 38px
#define BIN_LINE_Y (100 + (NUM_LEVELS - 1) * VERTICAL_SEP_INT + PEG_RADIUS_INT + BALL_RADIUS_INT) // ball finish line below last row of pegs (395)
#define HIST_BASE_Y 480                                                                           // bars grow upward from the bottom of the screen
#define HIST_MAX_H 80                                                                             // tallest a bar can be

// one copy per core so the two cores never increment the same counter at once
int bins[2][NUM_BINS];              // histogram bins (summed when drawn)
volatile int total_fallen[2] = {0}; // total balls fallen through the board

// float Q_rsqrt(fix15 number)
// {
//   fix15 i;
//   fix15 x2, y;
//   const fix15 threehalfs = 1.5F;

//   x2 = multfix15(number, 0.5F);
//   y = number;
//   i = *(fix15 *)&y;
//   i = 0x5f3759df - (i >> 1);
//   y = *(fix15 *)&i;
//   y = multfix15(y, (threehalfs - multfix15(multfix15(x2, y), y)));

//   return y;
// }
// float Q_rsqrt(float number)
// {
//   long i;
//   float x2, y;
//   const float threehalfs = 1.5F;

//   x2 = number * 0.5F;
//   y = number;
//   i = *(long *)&y;
//   i = 0x5f3759df - (i >> 1);
//   y = *(float *)&i;
//   y = y * (threehalfs - (x2 * y * y));

//   return y;
// }
static inline float Q_rsqrt(float x)
{
  float xhalf = 0.5f * x;
  int32_t i;
  memcpy(&i, &x, sizeof i); // safe type-pun, same cost as the cast
  i = 0x5f3759df - (i >> 1);
  memcpy(&x, &i, sizeof x);
  x = x * (1.5f - xhalf * x * x); // one Newton step
  return x;
}

// the color of the Ball
char color = WHITE;

typedef struct
{
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
  int16_t prev_peg; // -1 = no peg yet
} Ball;

Ball balls[MAX_NUM_BALLS];

typedef struct
{
  fix15 x;
  fix15 y;
} Peg;

Peg pegs[NUM_PEGS];

// Create a semaphore
semaphore_t enc_semaphore;
semaphore_t phys_sem; // core 0 -> core 1: frame cleared, go
semaphore_t done_sem; // core 1 -> core 0: my half of the balls is drawn
volatile int frame_n; // ball count core 0 picked for this frame, so both cores split the same n
int data_chan;
int ctrl_chan;

// time elapsed
uint64_t start_us;

volatile int mode = 0; // 0 for ball count, 1 for bounciness

volatile bool b_value;
volatile int count = 5000;                // counter for measuring orientation - +1 for clockwise, -1 for counter-clockwise
volatile float bounciness = 0.3;          // stores the current bounciness value
volatile fix15 bounce = float2fix15(0.3); // could make a new global temp var for fix15
volatile bool reset_hist = false;         // set by the encoder ISR, cleared by core 0 once it zeroes the histogram

// GPIO ISR on encoder pin As
void gpio_callback(uint gpio, uint32_t events)
{
  // Check encoder pin B
  b_value = gpio_get(ENCODER_B);
  if (gpio == ENCODER_A)
  {
    reset_hist = true; // any parameter change resets the histogram + fallen count
    if (b_value)
    { // clockwise
      if (mode == 1)
      {
        if (bounciness < 1)
        {
          bounciness += 0.01;
          bounce = float2fix15(bounciness);
        }
      }
      else
      {
        if (count < MAX_NUM_BALLS)
        {
          count += 1;
        }
      }
    }
    else
    { // counter-clockwise
      if (mode == 1)
      {
        if (bounciness > 0.0)
        {
          bounciness -= 0.01;
          bounce = float2fix15(bounciness);
        }
      }
      else
      {
        if (count > 0)
        {
          count -= 1;
        }
      }
    }
  }
  else if (gpio == SW)
  {
    mode = !mode;
  }
}

// Create a Ball
void spawnBall(Ball *ball)
{
  // Start from top of the screen
  ball->x = SPAWN_X;
  ball->y = SPAWN_Y;
  // ball->vx = ((fix15)(rand() & 0xffff) >> 15) + ((fix15)(rand() & 0xffff) >> 16) - 49152; // 1.5 * 32768 = 49152
  float random_vx = ((float)rand() / (float)RAND_MAX) - 0.5f;
  ball->vx = float2fix15(random_vx);
  ball->vy = VY_INITIAL;
}

void trigger_sound()
{
  dma_start_channel_mask(1u << ctrl_chan);
}

void initBalls()
{
  for (int i = 0; i < MAX_NUM_BALLS; i++)
  {
    spawnBall(&balls[i]);

    balls[i].prev_peg = -1;
  }
}

void initPegs()
{
  int peg_index = 0;

  for (int level = 0; level < NUM_LEVELS; level++)
  {
    for (int j = 0; j <= level; j++)
    {
      // Center the row at x = 320; adjacent pegs are HORIZONTAL_SEP apart
      int x = 320 + (2 * j - level) * (HORIZONTAL_SEP_INT / 2);
      int y = 100 + level * VERTICAL_SEP_INT;

      pegs[peg_index].x = int2fix15(x);
      pegs[peg_index].y = int2fix15(y);
      // pegs[peg_index].rad = PEG_RADIUS; // remove for memory
      peg_index++;
    }
  }
}
#define SEP_SQ_FIX multfix15(SEPARATION_DIST, SEPARATION_DIST)
static inline void collide(Ball *ball, int peg_index)
{
  Peg *peg = &pegs[peg_index];

  fix15 dx = ball->x - peg->x;
  fix15 dy = ball->y - peg->y;

  if (abs(dx) < (SEPARATION_DIST) && (abs(dy) < (SEPARATION_DIST)))
  {
    fix15 d2 = multfix15(dx, dx) + multfix15(dy, dy);
    if (d2 < SEP_SQ_FIX && d2 > 0) // no sqrt needed for the hit test
    {
      fix15 inv = float2fix15(Q_rsqrt(fix2float15(d2))); // 1/dist
      fix15 normal_x = multfix15(dx, inv);
      fix15 normal_y = multfix15(dy, inv);
      // fix15 dist = sqrtfix(multfix15(dx, dx) + multfix15(dy, dy));
      // if (dist < (SEPARATION_DIST))
      // {
      // fix15 inv = float2fix15(Q_rsqrt(fix2float15(dist)));
      // // fix15 normal_x = multfix15(dx, float2fix15(Q_rsqrt(dist)));
      // // fix15 normal_y = multfix15(dy, float2fix15(Q_rsqrt(dist)));
      // fix15 normal_x = divfix(dx, inv);
      // fix15 normal_y = divfix(dy, inv);

      fix15 dot = multfix15(normal_x, ball->vx) + multfix15(normal_y, ball->vy);
      fix15 intermediate_term = multfix15(float2fix15(-2), dot);

      ball->x = peg->x + multfix15(normal_x, (SEPARATION_DIST + int2fix15(1)));
      ball->y = peg->y + multfix15(normal_y, (SEPARATION_DIST + int2fix15(1)));
      ball->vx = ball->vx + (multfix15(normal_x, intermediate_term));
      ball->vy = ball->vy + (multfix15(normal_y, intermediate_term));

      if (peg_index != ball->prev_peg) // if we hit a new peg
      {
        trigger_sound();
        // lose energy from bounciness
        ball->vx = multfix15(bounce, ball->vx);
        ball->vy = multfix15(bounce, ball->vy);
        ball->prev_peg = peg_index;
      }
    }
  }
}

// update a ball position/status
void updateBall(Ball *ball, int core) // core picks which bins/total_fallen to use
// void updateBall(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, int *prev) // remove rad again
{
  // did ball cross histogram finish line right below last peg row?
  if (ball->y > BIN_LINE_Y_FIX)
  {
    int x_int = fix2int15(ball->x);
    int b_idx; // bin index

    if (x_int < BIN_LEFT_X)
    { // left of all pegs
      b_idx = 0;
    }
    else if (x_int >= BIN_RIGHT_X)
    { // right of all pegs
      b_idx = NUM_BINS - 1;
    }
    else
    { // between 2 pegs
      b_idx = 1 + (x_int - BIN_LEFT_X) / BIN_WIDTH;
    }

    bins[core][b_idx]++;
    total_fallen[core]++;

    // respawn since the ball is counted in the histogram now
    spawnBall(ball);
    // spawnBall(x, y, vx, vy);

    ball->prev_peg = -1;
  }

  // Update position using velocity
  ball->x = ball->x + ball->vx;
  ball->y = ball->y + ball->vy;

  int bx = fix2int15(ball->x);
  int by = fix2int15(ball->y);
  int r0 = (by - 100) / VERTICAL_SEP_INT;
  for (int r = r0; r <= r0 + 1; r++)
  {
    if (r < 0 || r >= NUM_LEVELS)
      continue;
    int u = bx - 320 + (r + 1) * (HORIZONTAL_SEP_INT / 2);
    if (u < 0)
      continue;
    int j = u / HORIZONTAL_SEP_INT; // nearest peg in this row
    if (j > r)
      continue;
    collide(ball, (r * (r + 1)) / 2 + j);
  }
  // for (int peg_index = 0; peg_index < NUM_PEGS; peg_index += 1)
  // {
  //   collide(ball, peg_index);
  // }

  if (hitBottom(ball->y))
  {
    spawnBall(ball);
    // spawnBall(x, y, vx, vy);
    ball->prev_peg = -1;
  }

  // Reverse direction if we've hit a wall
  if (hitTop(ball->y))
  {
    ball->vy = (-ball->vy);
    ball->y = (ball->y + int2fix15(5));
  }
  if (hitRight(ball->x))
  {
    ball->vx = (-ball->vx);
    ball->x = (ball->x - int2fix15(5));
  }
  if (hitLeft(ball->x))
  {
    ball->vx = (-ball->vx);
    ball->x = (ball->x + int2fix15(5));
  }
  ball->vy = ball->vy + GRAVITY;
}

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);
  static char count_str[64];
  static char fallen_str[64];
  static char time_str[64];
  static char bounce_str[64];

  while (1)
  {
    // Wait for the signal that the buffer's changed
    PT_YIELD_UNTIL(pt, draw_start_signal());
    static uint32_t t0; // static: locals are lost across PT_YIELD
    t0 = time_us_64();  // TIMEEEE

    // reset upon change
    if (reset_hist)
    {
      reset_hist = false;
      memset(bins, 0, sizeof(bins));
      total_fallen[0] = 0;
      total_fallen[1] = 0;
    }

    // Clear the buffer, then let core 1 start on its half of the balls
    clearLowFrame(0, BLACK);
    int n = count;
    frame_n = n;
    sem_release(&phys_sem);

    // update + draw the first half of the balls (core 1 does the second half)
    for (int i = 0; i < n / 2; i++)
    {
      updateBall(&balls[i], 0);
      drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS_INT, color);
    }

    // Draw count of fallen balls
    sprintf(fallen_str, "Total particles dropped: %d", total_fallen[0] + total_fallen[1]);
    drawTextAscii(10, 10, fallen_str, WHITE, BLACK);

    // Draw count of balls
    sprintf(count_str, "Active particles: %d", count);
    drawTextAscii(10, 22, count_str, WHITE, BLACK);

    // Draw bounciness
    sprintf(bounce_str, "Bounciness: %f", bounciness);
    drawTextAscii(10, 34, bounce_str, WHITE, BLACK);

    // Draw time
    uint64_t elapsed_time = (time_us_64() - start_us) / 1000000;
    sprintf(time_str, "Time elapsed: %d", elapsed_time);
    drawTextAscii(10, 46, time_str, WHITE, BLACK);

    // Draw histogram
    int bin_sum[NUM_BINS];
    int max_bin = 1; // avoid dividing by 0 if no balls have fallen yet
    // combine both cores' bins and find the tallest one
    for (int k = 0; k < NUM_BINS; k++)
    {
      bin_sum[k] = bins[0][k] + bins[1][k];
      if (bin_sum[k] > max_bin)
        max_bin = bin_sum[k];
    }

    // normalize everything else to the tallest bin and draw
    for (int k = 0; k < NUM_BINS; k++)
    {
      int h = (bin_sum[k] * HIST_MAX_H) / max_bin; // normalize # of balls in each bar to a pixel fraction of the tallest one

      if (h > 0) // don't draw bins with 0 balls
      {
        // find coordinates of bin k that we are drawing in
        int left, width;
        if (k == 0)
        {
          left = 0;
          width = BIN_LEFT_X;
        }
        else if (k == NUM_BINS - 1)
        {
          left = BIN_RIGHT_X;
          width = 640 - BIN_RIGHT_X;
        }
        else
        {
          left = BIN_LEFT_X + (k - 1) * BIN_WIDTH;
          width = BIN_WIDTH;
        }

        fillRect(left + 1, HIST_BASE_Y - h, width - 2, h, GREEN);
      }
    }

    for (int i = 0; i < NUM_PEGS; i++)
    {
      drawCircle(fix2int15(pegs[i].x), fix2int15(pegs[i].y), PEG_RADIUS_INT, BLUE);
    }

    // don't start the next frame (and its clear) until core 1 is done drawing
    PT_YIELD_UNTIL(pt, sem_try_acquire(&done_sem));
    gpio_put(25, time_us_64() - t0 > 16667); // LED FLASHING

  } // END WHILE(1)
  PT_END(pt);
} // animation thread
void core1_main()
{
  while (1)
  {
    sem_acquire_blocking(&phys_sem); // wait until core 0 has cleared the frame
    int n = frame_n;
    for (int i = n / 2; i < n; i++)
    {
      updateBall(&balls[i], 1);
      drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS_INT, color);
    }
    sem_release(&done_sem);
  }
}
void init_audio()
{
  // Initialize SPI channel (channel, baud rate set to 20MHz)
  spi_init(SPI_PORT, 20000000);

  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Build sine table and DAC data table
  int i;
  for (i = 0; i < (sine_table_size); i++)
  {
    raw_sin[i] = (int)(2047 * sin((float)i * 6.283 / (float)sine_table_size) + 2047); // 12 bit
    DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff);
  }

  // Select DMA channels
  data_chan = dma_claim_unused_channel(true);

  ctrl_chan = dma_claim_unused_channel(true);

  // Setup the control channel
  dma_channel_config c = dma_channel_get_default_config(ctrl_chan); // default configs
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);           // 32-bit txfers
  channel_config_set_read_increment(&c, false);                     // no read incrementing
  channel_config_set_write_increment(&c, false);                    // no write incrementing
  channel_config_set_chain_to(&c, data_chan);                       // chain to data channel

  dma_channel_configure(
      ctrl_chan,                        // Channel to be configured
      &c,                               // The configuration we just created
      &dma_hw->ch[data_chan].read_addr, // Write address (data channel read address)
      &address_pointer,                 // Read address (POINTER TO AN ADDRESS)
      1,                                // Number of transfers
      false                             // Don't start immediately
  );

  // Setup the data channel
  dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
  channel_config_set_read_increment(&c2, true);                      // yes read incrementing
  channel_config_set_write_increment(&c2, false);                    // no write incrementing
  // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
  // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
  dma_timer_set_fraction(0, 0x0017, 0xffff);
  // 0x3b means timer0 (see SDK manual)
  channel_config_set_dreq(&c2, 0x3b); // DREQ paced by timer 0
  // chain to the controller DMA channel
  // channel_config_set_chain_to(&c2, ctrl_chan); // Chain to control channel

  dma_channel_configure(
      data_chan,                 // Channel to be configured
      &c2,                       // The configuration we just created
      &spi_get_hw(SPI_PORT)->dr, // write address (SPI data register)
      DAC_data,                  // The initial read address
      sine_table_size,           // Number of transfers
      false                      // Don't start immediately.
  );

  // start the control channel
  // dma_start_channel_mask(1u << ctrl_chan);
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
  set_sys_clock_khz(150000, true);
  // initialize stdio
  stdio_init_all();

  // initialize LED
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);

  // Configure GPIO interrupt on encoder pin A
  gpio_init(ENCODER_A);
  gpio_set_dir(ENCODER_A, GPIO_IN);
  gpio_pull_up(ENCODER_A);
  gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // Configure pin B
  gpio_init(ENCODER_B);
  gpio_set_dir(ENCODER_B, GPIO_IN);
  gpio_pull_up(ENCODER_B);

  // Configure GPIO input on one of the button switch pins
  gpio_init(SW);
  gpio_set_dir(SW, GPIO_IN);
  gpio_pull_up(SW);
  gpio_set_irq_enabled_with_callback(SW, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // initialize VGA
  initVGA();

  // initialize audio
  init_audio();
  initBalls();
  initPegs();

  // Initialize the semaphore
  // Arguments: pointer to sem, initial count, max count
  // sem_init(&enc_semaphore, 0, 1);

  sem_init(&phys_sem, 0, 1);
  sem_init(&done_sem, 0, 1);
  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // // add threads
  start_us = time_us_64();
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start;
}