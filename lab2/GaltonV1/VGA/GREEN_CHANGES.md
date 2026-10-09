# Green/black (1 bit per pixel) VGA driver

This is a simpler version of the black/white driver (see `BW_CHANGES.md`). Lit pixels show as **green** instead of white. That means each pixel bit only has to drive **one** pin instead of all four, which simplifies the PIO program.

| | Black/white | Green/black (new) |
|---|---|---|
| Driver | `vga1_graphics_v3.c` | `vga1_green_graphics_v3.c` |
| PIO program | `rgb1.pio` (9 instructions) | `rgb_green.pio` (7 instructions) |
| Pins driven | GPIO 18–21 (all 4) | GPIO 19 only (green high bit) |
| Lit pixel color | `WHITE` | `MED_GREEN` |
| Bytes per frame buffer | 38,400 | 38,400 (unchanged) |
| pio0 instruction use | 32 / 32 | 30 / 32 |

The frame buffer and the RAM savings are the same as the black/white driver. Only the way bits reach the screen changed.

## Switching to the green driver

Change two lines in `CMakeLists.txt`:

```cmake
pico_generate_pio_header(VGA_Animation_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb_green.pio)
target_sources(VGA_Animation_Demo PRIVATE animation.c VGA/vga1_green_graphics_v3.c)
```

`animation.c` doesn't need any changes. Drawing with `WHITE` (or any nonzero color) gives green, and `BLACK` gives black.

---

## `rgb_green.pio` (based on `rgb1.pio`)

### Before (`rgb1.pio`): one bit copied to all 4 pins

```
pull block
out isr, 32               ; save count in ISR, empty OSR
.wrap_target
set pins, 0
mov y, isr
wait 1 irq 1 [3]
colorout:
  out x, 1                ; bit -> x
  jmp x-- invert          ; 1 -> 0, 0 -> 0xFFFFFFFF
invert:
  mov pins, ~x [6]        ; 1111 or 0000 on 4 pins
  jmp y-- colorout
.wrap
```

### After (`rgb_green.pio`): one bit to one pin

```
pull block
out y, 32                 ; save count in y, empty OSR
.wrap_target
set pins, 0
mov x, y
wait 1 irq 1 [3]
colorout:
  out pins, 1 [8]         ; bit -> green pin directly
  jmp x-- colorout
.wrap
```

### What changed

1. **The bit goes straight to the pin.** With only one pin to drive, `out pins, 1` sends the pixel bit directly. The three-step trick (`out x, 1` → `jmp x--` → `mov pins, ~x`) is gone. The inner loop is now 2 instructions, the same shape as the original 16-color `rgb.pio`.

2. **The counter is back in `x` with its saved copy in `y`,** as in the original `rgb.pio`. `x` is no longer needed for the pixel bit, so the count no longer has to be stored in ISR.

3. **Startup uses `out y, 32`.** This saves the count in `y` and empties the OSR in one instruction. The first `out pins, 1` then autopulls pixel data instead of sending leftover bits of the count to the screen.

4. **Pixel timing.** At 250 MHz each pixel must last 10 cycles:

   | Instruction | Cycles |
   |---|---|
   | `out pins, 1 [8]` | 1 + 8 |
   | `jmp x-- colorout` | 1 |
   | **Total** | **10** |

   `pixelhold` is now **8**, up from 6. **At 150 MHz, change it to 4** (6 cycles per pixel).

5. **Pin setup in `rgb_green_program_init`:**
   - The OUT and SET pin groups are 1 pin wide instead of 4.
   - Only that one pin is connected to the PIO (`pio_gpio_init`) and set as an output.
   - Autopull (32 bits, shift right) is the same as in `rgb1.pio`.

6. **Instruction budget:** hsync 9 + vsync 14 + rgb_green 7 = **30 / 32**. pio0 now has 2 free instructions, where the black/white version used all 32.

### Why GPIO 19 (the green high bit)?

GPIO 19 drives green through the 330 Ω resistor, and GPIO 18 drives it through 470 Ω. The 330 Ω pin is the brighter of the two, the same shade as `MED_GREEN`. Driving both pins for full `GREEN` would need the bit-copying trick again, which is what this version removes.

GPIO 18, 20 and 21 are never connected to the PIO. Like every RP2350 pin at reset, they have pull-downs, and the monitor's 75 Ω input termination also pulls those color lines to 0 V. So red, blue and the low green bit stay dark.

---

## `vga1_green_graphics_v3.c` (based on `vga1_graphics_v3.c`)

The 1-bit buffer, `drawPixel`, line drawing, text and clear functions are all **unchanged**. The black/white driver had already simplified those to 1 bit per pixel. The only changes are where the C code connects to the PIO:

| Location | Before | After |
|---|---|---|
| PIO header | `#include "rgb1.pio.h"` | `#include "rgb_green.pio.h"` |
| `initVGA`: load program | `pio_add_program(pio, &rgb1_program)` | `pio_add_program(pio, &rgb_green_program)` |
| `initVGA`: init program | `rgb1_program_init(..., LO_GRN)` | `rgb_green_program_init(..., HI_GRN)` |
| `readPixel` return value | `WHITE` / `BLACK` | `GREEN` / `BLACK` |
| Header comment | black/white description | green/black description |

Notes:

- **The init pin changed from `LO_GRN` to `HI_GRN`.** The old program used 4 pins starting at GPIO 18. The new program uses one pin, and it should be the brighter green.
- **`readPixel` returns `GREEN` (3),** which matches the new color and keeps `readPixel(...) & 1` (used by `isAlive`) working. `MED_GREEN` (2) would have broken `& 1`. Code that checks `readPixel(...) == WHITE` would need to change, but `animation.c` doesn't call `readPixel`.

---

## Testing so far

- A copy of the project builds with the two files swapped in through `CMakeLists.txt`.
- The generated PIO headers confirm hsync = 9, vsync = 14 and rgb_green = 7 instructions (30 total).
- **It has not been run on hardware yet.** Things to check on the monitor: lit pixels are green with no red or blue, and the image isn't shifted or wrapped horizontally. A shift or wrap would point to `pixelhold`.
