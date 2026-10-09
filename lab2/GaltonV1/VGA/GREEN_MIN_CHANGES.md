# Green/black driver: minimal changes to the original

This is the same green/black, 1-bit-per-pixel display as `GREEN_CHANGES.md`. The difference is that it's built from the **original 16-color files** with as few edits as possible, so the diff against the original is short and easy to review.

| | Original | Minimal green (new) |
|---|---|---|
| Driver | `vga16_graphics_v3.c` | `vga16_green_graphics_v3.c` |
| PIO program | `rgb.pio` (7 instructions) | `rgb_green_min.pio` (7 instructions) |
| Bits per pixel | 4 | 1 |
| Bytes per frame buffer | 153,600 | 38,400 |
| Pins driven | GPIO 18–21 | GPIO 19 only (green high bit) |
| Lit pixel color | any of 16 | `MED_GREEN` (any nonzero color draws green) |

Like the other 1-bit versions, it frees **230,400 bytes** of RAM (double buffered).

## Switching to this driver

Change two lines in `CMakeLists.txt`:

```cmake
pico_generate_pio_header(VGA_Animation_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb_green_min.pio)
target_sources(VGA_Animation_Demo PRIVATE animation.c VGA/vga16_green_graphics_v3.c)
```

`animation.c` doesn't need changes.

## Two choices that kept the diff small

1. **The DMA is unchanged: still 8-bit transfers.** When the DMA writes one byte to the PIO FIFO, the hardware copies that byte into all four bytes of the 32-bit FIFO word. The original driver already relied on this. With autopull set to refill every **8** bits, the PIO uses one byte (8 pixels) and then pulls the next. The buffer declarations and DMA channel setup stay exactly the same. Only the `VGA_BUFFER_COUNT` value changes.

2. **The PIO program is still named `rgb`.** The new file is `rgb_green_min.pio`, so the generated header is `rgb_green_min.pio.h`, but the functions are still `rgb_program` and `rgb_program_init`. The C file only needs a new `#include`.

---

## `rgb_green_min.pio` (diff vs `rgb.pio`)

```diff
-.define pixel1hold 9
-.define pixel2hold 7
+.define pixelhold 8

 pull block
-mov y, osr
+out y, 32
 .wrap_target
 set pins, 0
 mov x, y
 wait 1 irq 1 [3]
 colorout:
-	pull block
-	out pins, 4	[pixel1hold]
-	out pins, 4	[pixel2hold]
+	out pins, 1	[pixelhold]
 	jmp x-- colorout
 .wrap
```

| Change | Why |
|---|---|
| `out pins, 4` ×2 → `out pins, 1` ×1 | One bit per pixel, sent to one pin. The loop now handles 1 pixel instead of 2. |
| Removed `pull block` in the loop | Autopull now loads data: every 8 pixels it pulls the next byte from the FIFO automatically. With FIFO data waiting, this costs no cycles. |
| `mov y, osr` → `out y, 32` | Saves the pixel count in `y` **and** empties the OSR. With `mov`, the OSR would still hold the count, and the first `out pins, 1` would send count bits to the screen. |
| `pixel1hold`/`pixel2hold` → `pixelhold 8` | `out` (1 + 8) + `jmp` (1) = 10 cycles per pixel at 250 MHz. **At 150 MHz use 4.** |

Changes in `rgb_program_init`:

```diff
-    sm_config_set_set_pins(&c, pin, 4);
-    sm_config_set_out_pins(&c, pin, 4);
+    sm_config_set_set_pins(&c, pin, 1);
+    sm_config_set_out_pins(&c, pin, 1);
+    sm_config_set_out_shift(&c, true, true, 8);   // shift right, autopull every 8 bits
 ...
     pio_gpio_init(pio, pin);
-    pio_gpio_init(pio, pin+1);
-    pio_gpio_init(pio, pin+2);
-    pio_gpio_init(pio, pin+3);
-    pio_sm_set_consecutive_pindirs(pio, sm, pin, 4, true);
+    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
```

The program is the same length as the original (7 instructions). pio0 uses hsync 9 + vsync 14 + rgb 7 = **30 / 32**.

GPIO 18, 20 and 21 are no longer connected to the PIO. Their reset pull-downs and the monitor's 75 Ω termination keep red, blue and the low green bit dark.

---

## `vga16_green_graphics_v3.c` (diff vs `vga16_graphics_v3.c`)

### Constants and setup

| Line | Before | After |
|---|---|---|
| PIO header | `#include "rgb.pio.h"` | `#include "rgb_green_min.pio.h"` |
| `RGB_ACTIVE` | `319` (2 pixels per loop) | `639` (1 pixel per loop) |
| `VGA_BUFFER_COUNT` | `153600` (2 pixels/byte) | `38400` (8 pixels/byte) |
| `initVGA` | `rgb_program_init(..., LO_GRN)` | `rgb_program_init(..., HI_GRN)` |

`VGA_BUFFER_COUNT` is used for both the buffer size and the DMA transfer count, so changing it updates both. `HI_GRN` (GPIO 19, 330 Ω) is the brighter of the two green pins.

### `drawPixel`

```diff
-    char * draw_loc = (current_draw_buffer + ((640 * y + x) >> 1)) ;
-    if (x & 1) {
-        *(draw_loc) = (*(draw_loc) & TOPMASK) | (color << 4) ;
-    }
-    else {
-        *(draw_loc) = (*(draw_loc) & BOTTOMMASK) | (color) ;
-    }
+    char * draw_loc = (current_draw_buffer + ((640 * y + x) >> 3)) ;
+    if (color) {
+        *(draw_loc) |= (1 << (x & 7)) ;
+    }
+    else {
+        *(draw_loc) &= ~(1 << (x & 7)) ;
+    }
```

Pixel `x` is bit `x & 7` of byte `(640*y + x) >> 3`. The PIO shifts out the least significant bit first, so bit 0 is the leftmost pixel of each byte.

### `readPixel`

It uses the same `>> 3` / `x & 7` addressing and returns `GREEN` (3) or `BLACK` (0). `GREEN` is odd, so `readPixel(...) & 1` (used by `isAlive`) still works.

### `drawHLine`

```diff
-  short both_color = color | (color<<4) ;
-  if((x & 1)) { ...one pixel... }
-  if((w & 1)) { ...one pixel... }
-  int len = (w>>1) ;
-  memset(current_draw_buffer+(320*y+(x>>1)), both_color, len) ;
+  short both_color = color ? 0xff : 0x00 ; // 8 pixels per byte
+  while((x & 7) && w > 0) { ...one pixel... }
+  while((w & 7) && w > 0) { ...one pixel... }
+  int len = (w>>3) ;
+  memset(current_draw_buffer+(80*y+(x>>3)), both_color, len) ;
```

Up to 7 pixels can sit on each side of the full bytes now, so the `if`s became `while` loops.

### Text functions

`drawTextGLCD`, `drawTextAscii`, `drawTextTiny8`, `drawTextVGA437`, `drawTextArial24` and `drawTextGrotesk32` wrote a 2-pixel byte at a time from a `pix_value[4]` lookup table. The table and the `draw_loc` pointer are removed. Each write is replaced one-for-one with a call to a new helper that keeps the same index meaning:

```c
// bit 1 of v is the left pixel (x), bit 0 is the right pixel (x+1)
static inline void drawPixelPair(short x, short y, int v, char color, char bgcolor);
```

```diff
-        *(draw_loc+i*320+1) = pix_value[(line>>4) & 0x03] ;
+        drawPixelPair(x+2, y+i, (line>>4) & 0x03, color, bgcolor) ;
```

The bit-extraction expressions are untouched, so every font draws the same as before. Byte offset `+k` becomes pixel offset `x+2k`. The `draw_loc += n` lines are gone because `x += 2n` already advances each character.

### Clear functions

| Function | Change |
|---|---|
| `clearLowFrame` | `320` → `80` bytes per line, fill `c \| (c<<4)` → `c ? 0xff : 0x00` |
| `clearRegion` | same as `clearLowFrame` |
| `clearRect` | `memset` per row → `drawHLine(x1, i, x2-x1, c)` per row. A per-row `memset` would require `x1` and `x2` to be multiples of 8. `drawHLine` handles any x. |

### Left unchanged on purpose

To keep the diff small, these stay as they are. They are harmless:

- The `TOPMASK`/`BOTTOMMASK` defines are no longer used.
- `checkNeighbors` still calculates a `draw_loc` it never uses (its math assumes 4-bit pixels, but it never reads or writes through it).
- The buffer copy functions need no change because they use `VGA_BUFFER_COUNT`.
- **Existing bug, not fixed:** `drawHLine` doesn't check for negative `x` or `y`, so a line that starts off the left or top edge can make `memset` write outside the frame buffer. The original driver has the same issue. `vga1_graphics_v3.c` fixes it if you need that.

---

## Testing so far

- A copy of the project builds with the two files swapped in through `CMakeLists.txt`.
- The generated PIO headers confirm hsync = 9, vsync = 14 and rgb = 7 instructions (30 total).
- The two frame buffers are 0x9600 = 38,400 bytes each.
- **It has not been run on hardware yet.** Things to check on the monitor:
  - Lit pixels are green, with no red or blue.
  - The image isn't shifted or wrapped horizontally. A shift or wrap points to `pixelhold`.
  - Text isn't mirrored within each 8-pixel group. Mirroring would mean the bit order is reversed.
