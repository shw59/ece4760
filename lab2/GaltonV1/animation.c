
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
#include <math.h>
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
// #define SW 

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
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
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))
// uS per frame
#define FRAME_RATE 33000
#define GRAVITY float2fix15(0.37)
#define BALL_RADIUS int2fix15(4)
#define PEG_RADIUS int2fix15(6)
#define BOUNCINESS float2fix15(0.5)
#define NUM_BALLS 10
#define HORIZONTAL_SEP int2fix15(38)
#define VERTICAL_SEP int2fix15(19)
#define NUM_LEVELS 16
#define NUM_PEGS 136
// int NUM_PEGS = NUM_LEVELS * (NUM_LEVELS + 1) / 2;

#define BALL_RADIUS_INT 4
#define PEG_RADIUS_INT 6
fix15 SEPARATION_DIST = BALL_RADIUS + PEG_RADIUS;

// the color of the boid
char color = WHITE;

typedef struct
{
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
  fix15 rad;
  int current_peg;
  int prev_peg;
} Ball;

Ball balls[NUM_BALLS];

typedef struct
{
  fix15 x;
  fix15 y;
  fix15 rad;
} Peg;

Peg pegs[NUM_PEGS];

// Create a semaphore
semaphore_t draw_semaphore;
int data_chan;
int ctrl_chan;

volatile bool b_value;
volatile int count = 0; // counter for measuring orientation - +1 for clockwise, -1 for counter-clockwise
// GPIO ISR on encoder pin As
void gpio_callback(uint gpio, uint32_t events)
{

    // Check encoder pin B
    b_value = gpio_get(ENCODER_B);
    if (gpio == ENCODER_A)
    {
        if (b_value)
        { // clockwise
            gpio_put(25, !gpio_get(25));
            count += 1;
        }
        else
        { // counter-clockwise
            count -= 1;
        }
    }
}
// Create a boid
void spawnBoid(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, fix15 *rad)
{
  // Start from top of the screen
  *x = int2fix15(320);
  *y = int2fix15(0);
  float random_vx = ((float)rand() / (float)RAND_MAX) - 0.5f;
  *vx = float2fix15(random_vx);
  *vy = int2fix15(0);
  *rad = BALL_RADIUS;
}
// volatile current_peg;
void trigger_sound()
{
  dma_start_channel_mask(1u << ctrl_chan);
}

void initBoids()
{
  for (int i = 0; i < NUM_BALLS; i++)
  {
    spawnBoid(
        &balls[i].x,
        &balls[i].y,
        &balls[i].vx,
        &balls[i].vy,
        &balls[i].rad);

    balls[i].prev_peg = -1;
    balls[i].current_peg = -1;
  }
}

void initPegs()
{
    int peg_index = 0;

    for (int level = 0; level < NUM_LEVELS; level++)
    {
        int num_pegs_level = level + 1;

        for (int j = 0; j < num_pegs_level; j++)
        {
            // Horizontal position:
            // center the row around x = 320
            int x = 320 + (2 * j - level) * 38;

            // Vertical position
            int y = 100 + level * 19;

            pegs[peg_index].x = int2fix15(x);
            pegs[peg_index].y = int2fix15(y);
            pegs[peg_index].rad = PEG_RADIUS;

            peg_index++;
        }
    }
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, fix15 *rad, int *curr, int *prev)
{
  // Update position using velocity
  *x = *x + *vx;
  *y = *y + *vy;
  for (int peg_index = 0; peg_index < NUM_PEGS; peg_index += 1)
  {
    Peg *peg = &pegs[peg_index];

    fix15 dx = *x - peg->x;
    fix15 dy = *y - peg->y;

    if (abs(dx) < (SEPARATION_DIST) && (abs(dy) < (SEPARATION_DIST)))
    {
      fix15 dist = sqrtfix(multfix15(dx, dx) + multfix15(dy, dy));
      if (dist < (SEPARATION_DIST))
      {
        fix15 normal_x = divfix(dx, dist);
        fix15 normal_y = divfix(dy, dist);

        fix15 dot = multfix15(normal_x, *vx) + multfix15(normal_y, *vy);
        fix15 intermediate_term = multfix15(float2fix15(-2), dot);

        *x = peg->x + multfix15(normal_x, (SEPARATION_DIST + int2fix15(1)));
        *y = peg->y + multfix15(normal_y, (SEPARATION_DIST + int2fix15(1)));
        *vx = *vx + (multfix15(normal_x, intermediate_term));
        *vy = *vy + (multfix15(normal_y, intermediate_term));

        if (*curr != *prev)
        {
          trigger_sound();
          // lose energy from bounciness
          *vx = multfix15(BOUNCINESS, *vx);
          *vy = multfix15(BOUNCINESS, *vy);
        }
      }
    }
  }
  if (hitBottom(*y))
  {
    spawnBoid(x, y, vx, vy, rad);
  }

  // Reverse direction if we've hit a wall
  if (hitTop(*y))
  {
    *vy = (-*vy);
    *y = (*y + int2fix15(5));
  }
  if (hitRight(*x))
  {
    *vx = (-*vx);
    *x = (*x - int2fix15(5));
  }
  if (hitLeft(*x))
  {
    *vx = (-*vx);
    *x = (*x + int2fix15(5));
  }
  *vy = *vy + GRAVITY;
}

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);
  static char count_str[20];

  while (1)
  {
    // Wait for the signal that the buffer's changed
    PT_YIELD_UNTIL(pt, draw_start_signal());
    // Clear the buffer
    clearLowFrame(0, BLACK);
    // Signal core 1 that it can start drawing
    PT_SEM_SDK_SIGNAL(pt, &draw_semaphore);

    for (int i = 0; i < NUM_BALLS; i++)
    {
      fillCircle(
          fix2int15(balls[i].x),
          fix2int15(balls[i].y),
          fix2int15(balls[i].rad),
          color);
      wallsAndEdges(&balls[i].x, &balls[i].y, &balls[i].vx, &balls[i].vy, &balls[i].rad, &balls[i].current_peg, &balls[i].prev_peg);

      // Draw encoder count
      sprintf(count_str, "Count: %d", count);
      drawTextAscii(10, 10, count_str, WHITE, BLACK);
      
    }

    for (int i = 0; i < NUM_PEGS; i++)
    {
      fillCircle(fix2int15(pegs[i].x), fix2int15(pegs[i].y), PEG_RADIUS_INT, BLUE);
    }
    
    // // update boid's position and velocity
    // // wallsAndEdges(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy);

    // // draw the boid at its new position
    // fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), BALL_RADIUS, color);
    // // draw the boundaries
    // drawArena();
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

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
  // initialize stio
  stdio_init_all();

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
  // gpio_init(SW);
  // gpio_set_dir(SW, GPIO_IN);
  // gpio_pull_up(SW);

  // initialize VGA
  initVGA();

  // initialize audio
  init_audio();
  initBoids();
  initPegs();

  // Initialize the semaphore
  // Arguments: pointer to sem, initial count, max count
  sem_init(&draw_semaphore, 0, 1);

  // start core 1
  // multicore_reset_core1();
  // multicore_launch_core1(&core1_main);

  // // add threads
  // pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start;
}
