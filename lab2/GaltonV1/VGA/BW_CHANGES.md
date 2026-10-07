# Black/white (1 bit per pixel) VGA driver

This driver is a 1-bit-per-pixel version of the 16-color driver. It frees RAM so we can spawn more balls.

| | 16-color (original) | Black/white (new) |
|---|---|---|
| Driver | `vga16_graphics_v3.c` | `vga1_graphics_v3.c` |
| PIO program | `rgb.pio` | `rgb1.pio` |
| Bits per pixel | 4 | 1 |
| Bytes per frame buffer | 153,600 | 38,400 |
| Both buffers (double buffered) | 307,200 | 76,800 |

That saves **230,400 bytes**. A ball struct is 20 bytes, so that's room for about 11,000 more balls.

The header (`vga16_graphics_v3.h`) is unchanged. Every function and color name still exists, so `animation.c` compiles without edits. **Any nonzero color draws white**, and `BLACK` (0) draws black.

Changing only `drawPixel` would not have been enough. The DMA channel and the PIO program both assume 4 bits per pixel, and several drawing functions write to the buffer directly instead of calling `drawPixel`. All of those had to change together.

---

## Switching to the black/white driver

Change two lines in `CMakeLists.txt`:

```cmake
pico_generate_pio_header(VGA_Animation_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb1.pio)
target_sources(VGA_Animation_Demo PRIVATE animation.c VGA/vga1_graphics_v3.c)
```

To go back, change them back to `rgb.pio` and `vga16_graphics_v3.c`.

---

## Buffer layout

Each line is 640 pixels, which is 80 bytes (20 words of 32 bits). Pixel `(x, y)` lives at:

```
byte = (640*y + x) >> 3      // 8 pixels per byte
bit  = x & 7                 // bit 0 = leftmost pixel of the byte
```

The bit order matches how the PIO reads the data. The DMA sends 32-bit words, and the RP2350 stores them little-endian, so byte 0 sits in bits 0–7 of the word. The PIO shifts out the least significant bit first. As a result, bit `x & 7` of byte `x >> 3` is the `x`-th pixel sent to the screen.

---

## `rgb1.pio`

### Original program (`rgb.pio`)

```
pull block                ; pull 1 byte (2 pixels)
out pins, 4 [9]           ; pixel 1 -> 4 color pins
out pins, 4 [7]           ; pixel 2 -> 4 color pins
jmp x-- colorout          ; 320 iterations per line
```

Each 4-bit value went straight to the 4 color pins (GPIO 18–21).

### New program

```
pull block                ; (once) pull the pixel count (639)
out isr, 32               ; (once) store the count in ISR, leave OSR empty
.wrap_target
set pins, 0               ; blank the pins between lines
mov y, isr                ; reload the pixel counter
wait 1 irq 1 [3]          ; wait for vsync's "active line" signal
colorout:
  out x, 1                ; next pixel bit -> x (0 or 1)
  jmp x-- invert          ; x = x - 1   (1 -> 0, 0 -> 0xFFFFFFFF)
invert:
  mov pins, ~x [6]        ; ~x = 1111 (white) or 0000 (black) on all 4 pins
  jmp y-- colorout        ; 640 iterations per line
.wrap
```

### What changed and why

**1. One bit is copied to all 4 color pins.** `out pins, 1` would only drive one pin (GPIO 18, the low green bit), giving a dim green instead of white. PIO has no instruction that repeats one bit across several pins, so the program does it in three steps:

- `out x, 1` puts the pixel bit in `x`, so `x` is `0` or `1`.
- `jmp x-- invert` decrements `x`. `jmp x--` always decrements, and its target is the next instruction, so it never actually skips anything. `1` becomes `0`, and `0` wraps around to `0xFFFFFFFF`.
- `mov pins, ~x` inverts the result, so `x` is all ones (white) when the bit was 1 and all zeros (black) when it was 0. Only the low 4 bits reach the pins.

**2. Autopull loads data.** The original ran `pull block` for every byte. The new program enables autopull with a 32-bit threshold in `rgb1_program_init`:

```c
sm_config_set_out_shift(&c, true /*shift right*/, true /*autopull*/, 32);
```

Each `out x, 1` takes one bit, and after 32 bits the OSR refills from the FIFO automatically. As long as the FIFO has data, the refill costs no extra cycles, so the pixel timing stays even. A line is 640 bits, exactly 20 words, so every line starts on a fresh word.

**3. The line counter moved from `x` to `y`, and its saved copy moved into ISR.** The original kept the counter in `x` and a saved copy in `y`. The new program needs `x` for the pixel bit, so the counter is now in `y`. The saved copy is in ISR, which this program doesn't otherwise use, and `mov y, isr` reloads it each line.

**4. Startup uses `out isr, 32` instead of `mov y, osr`.** This one instruction saves the count in ISR and also empties the OSR. The first `out x, 1` therefore autopulls real pixel data instead of reading leftover bits of the count.

**5. Pixel timing.** The system clock is 250 MHz and the pixel clock is 25 MHz, so each pixel must last 10 cycles:

| Instruction | Cycles |
|---|---|
| `out x, 1` | 1 |
| `jmp x-- invert` | 1 |
| `mov pins, ~x [6]` | 1 + 6 |
| `jmp y-- colorout` | 1 |
| **Total** | **10** |

The delay is set by `.define pixelhold 6`. **At 150 MHz, change it to 2** (6 cycles per pixel).

**6. Counter value.** `RGB_ACTIVE` in the C file changed from 319 (two pixels per loop) to 639 (one pixel per loop).

### Instruction budget

All three VGA programs share pio0's 32-instruction memory:

| Program | Instructions |
|---|---|
| `hsync.pio` | 9 |
| `vsync.pio` | 14 |
| `rgb1.pio` | 9 |
| **Total** | **32 / 32** |

pio0 is now full. Adding even one instruction to `rgb1.pio` makes `pio_add_program` fail when the program loads, not at compile time. If more instructions are needed, move the RGB program to pio1.

---

## `vga1_graphics_v3.c`

The file is a copy of `vga16_graphics_v3.c` with the changes below. Everything else is unchanged: lines, circles, rectangles, triangles, `drawChar` and the other deprecated text functions all draw through `drawPixel` or `drawHLine`.

### Constants and buffers

```c
#define RGB_ACTIVE 639                  // was 319
#define VGA_BYTES_PER_LINE 80           // new
#define VGA_BUFFER_COUNT 38400          // was 153600 (bytes)
#define VGA_DMA_COUNT (VGA_BUFFER_COUNT/4)   // new: 9600 32-bit transfers
#define FILL_BYTE(c) ((c) ? 0xff : 0x00)     // replaces TOPMASK/BOTTOMMASK
```

Both buffers are declared `__attribute__ ((aligned (4)))` because the DMA now reads them as 32-bit words.

### `initVGA`

- Includes `rgb1.pio.h`, loads `rgb1_program`, and calls `rgb1_program_init`.
- The pixel DMA channel uses `DMA_SIZE_32` and `VGA_DMA_COUNT` transfers. It had to switch from 8-bit transfers. When the DMA writes a single byte to the PIO FIFO, the byte is copied into all 4 bytes of the FIFO word. The PIO would then see each byte four times.
- The buffer-swapping DMA chain (display, draw, and start-flag channels) is unchanged.

### `drawPixel`

```c
unsigned char *draw_loc = (unsigned char *)current_draw_buffer + ((640*y + x) >> 3);
unsigned char mask = 1 << (x & 7);
if (color) *draw_loc |= mask;
else       *draw_loc &= ~mask;
```

The address is now `>> 3` (8 pixels per byte) instead of `>> 1`. The code sets or clears one bit instead of replacing a 4-bit half of the byte.

### `readPixel`

It reads bit `x & 7` and returns `WHITE` (15) or `BLACK` (0) rather than raw 0/1, so callers like `isAlive` that check `readPixel(...) & 1` still work.

### `drawHLine`

- Draws single pixels until `x` reaches a multiple of 8, and also draws the leftover pixels at the right end.
- Fills the middle with `memset` at 8 pixels per byte, using `0xFF` or `0x00`.
- **New clipping:** returns early if `y < 0`, and trims the line if `x < 0`. The original only checked the right and bottom edges, so a negative coordinate could make `memset` write outside the frame buffer.

### Text functions

`drawTextGLCD`, `drawTextAscii`, `drawTextTiny8`, `drawTextVGA437`, `drawTextArial24` and `drawTextGrotesk32` used to build 2-pixel bytes and write them straight into the buffer. They now use a shared helper:

```c
static inline void drawFontRow(short x, short y, unsigned short bits, int n,
                               char color, char bgcolor);
```

It draws `n` pixels of one font row, from the most significant bit (leftmost) down, by calling `drawPixel`. Each function passes its font's rows in the same bit order as before, so the text looks the same:

| Function | Pixels per row | Rows |
|---|---|---|
| `drawTextGLCD` | 6 (5 font columns + 1 space; column-major font, transposed) | 8 |
| `drawTextAscii` | 6 (top 6 bits of the byte) | 7 |
| `drawTextTiny8` | 8 | 8 |
| `drawTextVGA437` | 8 | 16 |
| `drawTextArial24` | 16 (two bytes, left byte first) | 24 |
| `drawTextGrotesk32` | 16 (two bytes, left byte first) | 31 |

Side effect: **x no longer has to be even** for text.

### Clear and copy functions

- `clearLowFrame` and `clearRegion` use `memset` with `VGA_BYTES_PER_LINE` (80) and `FILL_BYTE(c)`. Clearing a full frame is now 38 KB instead of 154 KB, so each frame should also draw a bit faster.
- `clearRect` now calls `drawHLine` for each row, so `x1` and `x2` no longer need to be even.
- `copy_buffer0to1`, `copy_buffer1to0` and `copy_buffer_to_other` are unchanged. They copy `VGA_BUFFER_COUNT` bytes, which is now the smaller size.

### Small cleanup

`checkNeighbors` declared a `draw_loc` it never used, and its address math assumed 4-bit pixels. That line was removed.

---

## Testing so far

- The project compiles with the new files swapped in through `CMakeLists.txt`.
- The generated PIO headers confirm hsync = 9, vsync = 14 and rgb1 = 9 instructions (32 total).
- With 10,000 balls, the build uses about 284 KB of RAM (`.bss`), and the two frame buffers are 0x9600 = 38,400 bytes each.
- **It has not been run on hardware yet.** Things to check on the monitor: white looks white rather than green, and the image isn't shifted or wrapped horizontally. A shift or wrap would point to the timing (`pixelhold`) or the bit order.
