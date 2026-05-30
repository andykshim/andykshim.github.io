---

layout: default
title: Project Title
---

# Nuclear Fission Simulation on RP2040 Microprocessor

---

---

<iframe width="720" height="405"
src="https://www.youtube.com/watch?v=SqB7Jm-Cdmk&list=PLDqMkB5cbBA6AwYC_DElkDStUdOsTuIL7&index=3&t=2s">
</iframe>


## 1\. Overview and Credits

Designed an emebedded system simulation of a RBMK-style nuclear fission reactor on Raspberry Pi Pico 2040 Microprocessor, with outputs to external monitor via VGA, and control panel of hardware inputs

**Project Members**   
Andrew Shim  
Tyler Wisniewski (ttw24)

This project was completed for ECE 4760/5730 at Cornell University, taught in Spring of 2025 by Professor V. Hunter Adams (vha3).

**Tech Stack**

* Languages: C/C++, PIO (Programmable I/O)
* Tools: Oscilloscope
* Hardware platforms: RP 2040
* Key Concepts: Real-time embedded systems, fixed-point arithmetic, PIO state machine programming, DMA-driven peripherals, multicore scheduling, framebuffer rendering optimization

**Related Links**   
[Project Website](https://ece4760.github.io/Projects/Spring2025/ks875_ttw24_3rdGroupMember/index.html)   
[Video Demo](https://www.youtube.com/watch?v=SqB7Jm-Cdmk&list=PLDqMkB5cbBA6AwYC_DElkDStUdOsTuIL7&index=3&t=2s)   
[Code](https://ece4760.github.io/Projects/Spring2025/ks875_ttw24_3rdGroupMember/code.html)

**In The News**
June 19, 2025: This project was highlighted in an article by Tyler August on Hackaday.com. Thank you Tyler! Read the article [here](https://hackaday.com/2025/06/19/fission-simulator-melts-down-rp2040/).


May 21, 2026: Professor V. Hunter Adams was interviewed by Ashley Whittaker for the Raspberry Pi Official Magazine. This project was one of three highlighted by prof. Adams as his favorite projects. Huge thank you to prof. Adams and Ashley! Read the article [here](https://magazine.raspberrypi.com/articles/v-hunter-adams-interview).


---

## 2\. Problem Definition

The goal was to recreate an RBMK-style nuclear fission reactor simulation on a Raspberry Pi Pico — a self-contained, interactive embedded appliance with VGA output and a physical control panel of buttons and encoders. The inspiration is this [video/simulation](https://www.youtube.com/watch?v=P3oKNE72EzU) by Higgsino Physics that explores the design flaws and operator decisions behind the Chernobyl disaster. Porting that same idea off the browser and onto a single microcontroller is a much harder problem.

The central design constraint is **real-time latency**. A 640 x 480 VGA output running the simulation at 30 frames per second gives the system a 33 ms budget per frame, and every step in the loop has to fit inside that window: neutron motion, collisions against 280 nuclei and 15 rods, water-heat propagation, button and encoder polling, screen redraw, and two live strip charts. Overrun on any one of them and the simulation falls behind the wall clock.

The RP2040 makes hitting that budget hard in three specific ways. First, its Cortex-M0+ cores have no hardware floating-point unit, so any naive use of `float` for particle positions or velocities would burn the budget on software emulation. Second, the chip has 264 kB of SRAM, and the framebuffer alone consumes 153.6 kB (640 x 480 x 4 bits) — leaving roughly 110 kB for the neutron arrays, the nuclei grid, and everything else. Third, the chip has no dedicated video hardware; the VGA signal itself is synthesized on the fly by the PIO state machines, sharing the same silicon and the same memory with the simulation. Closing the gap between the workload and the hardware is the subject of Section 5.

The scope of this report is the software side. My partner Tyler Wisniewski designed the hardware architecture — PCB, control panel, power, and level conversion — and his report is available [here](https://ece4760.github.io/Projects/Spring2025/ks875_ttw24_3rdGroupMember/hd.html). What follows covers the simulation, the rendering pipeline, the input handling, and the optimization work behind the 30 Hz target.

<div style="text-align: center;">      
<img src="/assets/fissionSim/controlPanel.jpg" alt="Hardware Control Panel" width="410">                                                                                                                                 
</div> 

<p style="text-align: center;">The hardware control panel</p>

---

## 3\. Features

A quick overview of what the finished system does, before getting into how it does it.

**Simulation physics**

* 28 x 10 grid of nucleus cells, 3 species: U-235 (fissile), Xe-135 (neutron poison), non-fissile filler
* Up to 2000 simultaneous neutron particles
* Two-population neutron model: fast neutrons (red) and thermal neutrons (pink), distinguished by an energy threshold
* Water moderation with 4 visual states tied to local heat: dark blue → blue → light blue → cyan (steam)
* 8 fixed graphite moderator rods that bounce and slow fast neutrons
* 7 movable control rods that absorb neutrons on contact
* Spontaneous regeneration of U-235 and Xe-135 from non-fissile nuclei, with probability scaled by neutron population
* Spontaneous neutron emission from non-fissile material (background activity)
* EAP (energy / activity parameter) that responds to the balance of moderation vs. cooling

**Hardware control panel**

* 4 buttons: SCRAM (slam rods to max), auto mode, manual mode, sim speed toggle (100% / 50%)
* 4 rotary encoders: control rod position, water flow rate, target neutron count, neutrons-per-fission
* Auto mode runs a closed-loop controller that holds `neutrons_active` within ±25% of the user's target by raising and lowering the control rods
* Manual mode hands rod position directly to encoder 1
* Audio output via SPI → external DAC, DMA-fed: a harmonically rich "click" fires on every U-235 fission event

**Display (640 x 480 @ 30 Hz, 16 colors)**

* Live legend identifying every on-screen species and rod type
* Stats row showing spare microseconds per frame, active neutrons, target, water flow, mode, sim speed, neutrons-per-fission
* Two real-time strip charts on Core 1:
   * left: xenon population, control rod position, EAP over time
   * right: uranium fission events per second

---

## 4\. Software Architecture and Implementation Details

### 4.1 System overview

The system is organized around two principles: keep the simulation on one core, and push every peripheral off the CPU. Core 0 runs the simulation step and the main framebuffer draws. Core 1 runs the two live strip charts and all hardware input polling — four buttons and four rotary encoders. Both cores schedule their threads cooperatively using protothreads. Three peripherals sit alongside them: the VGA output (three PIO state machines and one DMA channel), the audio output (SPI to an external DAC, fed by a two-channel DMA chain), and the control panel.

(insert block diagram: cores → threads → peripherals)

### 4.2 The VGA driver

The VGA driver is built on top of Hunter Adams's PIO-based VGA library. The RP2040 has no video peripheral, so the 25 MHz pixel clock and the front-porch / sync / back-porch timing on both axes are synthesized entirely in PIO assembly.

Three state machines on PIO0 run at 25 MHz (system clock divided by 5). `hsync` and `vsync` generate the sync pulses; `rgb` shifts pixel data onto a 4-bit color bus. They are kept in lockstep through IRQs: `hsync` raises IRQ 0 at end-of-line, `vsync` waits on IRQ 0 and raises IRQ 1 during active video, and `rgb` waits on IRQ 1 before shifting out pixel data. All three are started in the same call so they stay phase-locked from cycle zero.

Color is packed at four bits per pixel — sixteen colors. Two adjacent pixels share a byte, so `vga_data_array` is 640 × 480 × 4 bits / 8 bits per byte = 153.6 kB, roughly 58% of the RP2040's SRAM. A DMA channel continuously refills the `rgb` state machine's TX FIFO from this array and wraps each frame. Once started, the pixel pipeline is autonomous — neither core needs to touch it to keep a picture on screen.

### 4.3 Dual-core and protothreads

Cooperative threading is intentional: protothreads run each step to completion before yielding, so timing is reasoned about by reading the loop instead of by reasoning about interrupt priorities and re-entrancy. The frame budget is whatever the loop takes, measured at the end with a single `time_us_32()` call.

The Core 0 / Core 1 split moves the strip charts (heavy enough to drop frames if they shared the sim core) and the input polling (latency-tolerant but constant at 1 kHz) off the simulation's critical path. The cores share state through plain volatile globals — the encoder threads write directly into `waterFlow`, `neutrons_target_num`, `controlRods_y`, and Core 0 reads them on the next iteration. No locks; the writes are word-sized and a one-frame lag on a knob reading is invisible.

<div style="text-align: center;">      
<img src="/assets/fissionSim/Program.png" alt="Software Architecture" width="510">                                                                                                                                 
</div> 

<p style="text-align: center;">Software Architecture</p>

### 4.4 The main animation loop

The simulation is a single thread, `protothread_anim`, running on Core 0. Every iteration is one display frame, and the shape of that frame is the same every time.

The first half is per-neutron work. For each active particle: clear its old pixels (using a saved background color, §5), run whatever interactions apply, update position, redraw. Fast neutrons check for graphite-rod bounces. Thermal neutrons check for fission against every U-235 and Xe-135 nucleus — fission kicks off a click sound, recycles the neutron, and spawns a configurable number of new ones; xenon absorption removes it. All neutrons run water moderation (decrement energy, heat the local water cell) and check for absorption against the seven control rods.

The second half is system-wide work: water cooling, spontaneous regeneration of U-235 / Xe-135 / background neutrons, control-rod auto/manual update, staggered grid refresh (§5), arena border, stats row. Finally the loop computes `spare_time = FRAME_RATE − elapsed` and yields the rest. That value is printed to the screen — when it gets close to zero, a frame is about to slip, and it has been the single most useful debugging signal in the project.

### 4.5 Audio subsystem

Every U-235 fission triggers a short audible click. This click resembles that of a Geiger Counter, which sounds like an "audio pop" (instantaenous activation) across a wide frequency, and looks like a vertical line on a spectogram. To recreate this sound we have a 256-entry sine table — harmonically enriched and shaped by an exponential attack envelope, computed once at startup — and streamed over SPI to a 12-bit DAC via two chained DMA channels. The *control* channel reloads the *data* channel's read pointer to the start of the table; the data channel pushes 16-bit samples into the SPI data register, paced by a hardware timer DREQ at roughly 44 kHz. When fission fires, the simulation calls `dma_channel_set_read_addr` and starts the data channel; the click plays itself out, the control channel re-arms, and neither core is involved beyond the kickoff. Same pattern as the VGA driver, applied to a second peripheral.

### 4.6 Hardware input polling

Each of the four buttons and four rotary encoders has its own protothread on Core 1, running at 1 kHz.

The buttons run a four-state **debouncer** (`not_pressed → maybe_pressed → pressed → maybe_not_pressed`) that advances only when consecutive reads agree, rejecting contact bounce. The action — switching modes, slamming the rods to maximum, toggling sim speed — fires on the transition into `pressed`.

The encoders are decoded from their CLK and DT lines: on a falling edge of CLK, the level of DT picks the direction. Three encoders map directly to `waterFlow`, `neutrons_target_num`, and `numNeutronSpawn`. The fourth drives the control-rod target in manual mode; in auto mode the simulation owns rod position and ignores it. The mode-switch buttons sync the encoder accumulator back to the rods' current position on transition to manual, so manual control picks up smoothly from wherever the autopilot left off.

---

## 5\. Optimization Strategies

The optimization work is what closes the gap between the workload from §4 and the 33 ms-per-frame budget. Three things needed fixing: the arithmetic (so it doesn't fall into soft-float), the memory layout (so the simulation fits alongside the framebuffer in 264 kB), and the renderer (so it isn't repainting the screen from scratch every frame).

<div style="text-align: center;">      
<img src="/assets/fissionSim/display.png" alt="Simulation Display" width="510">                                                                                                                                 
</div> 

<p style="text-align: center;">Simulation Display</p>


### 5.1 Fixed-point arithmetic

The Cortex-M0+ has no FPU, so any `float` multiply or divide goes through soft-float routines in ROM — fast for a few calls, ruinous when it happens twice per particle per frame across two thousand particles. Every quantity in the simulation that would naturally be a `float` — neutron `x`, `y`, `vx`, `vy`, water heat, the EAP — is instead stored in `fix15`, a custom signed 32-bit format with one sign bit, 16 integer bits, and 15 fractional bits. Multiplication is a `signed long long` multiply followed by a right shift:

```c
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define int2fix15(a)   ((fix15)(a << 15))
#define fix2int15(a)   ((int)(((a) + 0x4000) >> 15))
```

These are macros, not functions, so they inline directly into the hot loops. The integer range is ±32,767, which is plenty for 640 × 480 coordinates and energy values up to 24,000.

### 5.2 Short datatypes

The 264 kB of SRAM is tight: the framebuffer takes 153.6 kB (§4.2), leaving about 110 kB. Two thousand `Neutron` structs and 280 `Nucleus` structs — plus their parallel index arrays — fill that quickly, so any field whose range allows it uses `short` (16 bits) instead of `int` (32 bits): neutron `energy` and `active` flag, nucleus type, the index arrays themselves. `int` is reserved for the few quantities that actually need 32 bits.

### 5.3 Draw only what has to be drawn

The default renderer for a particle simulation — clear the framebuffer, redraw everything at new positions — would blow the budget by itself. The simulation's display is mostly static: 280 nucleus cells, 15 rods, a legend, an arena border. Only the neutrons and the slow water-color transitions actually move frame to frame. Three techniques squeeze the per-frame drawing down to roughly that minimum.

**(a) Don't paint pixels you'll overwrite.** Each nucleus cell is a 4-pixel-radius circle overlapping a 17 × 17 water square. The straightforward sequence — fill the square, then the circle — wastes the center pixels of the square because the circle clobbers them. `fillRectAroundCircle` paints only the four side rectangles and the four corner arcs, leaving the circle's footprint untouched:

```c
void fillRectAroundCircle(short x, short y, char color) {
  fillRect(x - 8, y - 8, 17, 4, color) ; // top
  fillRect(x - 8, y + 5, 17, 4, color) ; // bottom
  fillRect(x - 8, y - 4,  4, 9, color) ; // left
  fillRect(x + 5, y - 4,  4, 9, color) ; // right
  drawHLine(x - 4, y - 4, 3, color) ;
  drawHLine(x + 2, y - 4, 3, color) ;
  drawHLine(x - 4, y + 4, 3, color) ;
  drawHLine(x + 2, y + 4, 3, color) ;
  drawVLine(x - 4, y - 3, 2, color) ;
  drawVLine(x + 4, y - 3, 2, color) ;
  drawVLine(x - 4, y + 2, 2, color) ;
  drawVLine(x + 4, y + 2, 2, color) ;
}
```

**(b) Cache the background under moving objects.** A neutron is a 2 × 2 sprite. The naïve flow — erase the old position with black, redraw at the new — leaves a black trail wherever the neutron has been, and forces a full water/nucleus refresh every frame to mask the trail. The fix is to sample the four pixels currently at the neutron's position *before* drawing it, stash those colors in the neutron struct, and use them to erase on the next frame. The base VGA library only exposed a write API, so I added a `readPixel` that handles the 4-bits-per-pixel packing (two pixels per byte, upper or lower nibble depending on parity):

```c
short readPixel(short x, short y) {
  int pixel = (640 * y) + x ;
  if (pixel & 1) return vga_data_array[pixel >> 1] >> 4 ;
  else           return vga_data_array[pixel >> 1] & 0xf ;
}
```

In theory this eliminates the periodic full-screen refresh entirely. In practice an edge case (covered in §7) meant we still had to refresh the background occasionally — but only every few seconds rather than every frame.

**(c) Stagger the periodic refresh.** "Every few seconds" can't mean *all 280 nuclei in one frame*: that's a tail-latency spike that would drop the frame. The grid (28 × 10 = 280 cells) is split into 56 stripes of 5 cells each, and `redrawScreen` paints one stripe per frame. At 30 FPS each cell is repainted once every 56 / 30 ≈ 1.87 seconds, and the per-frame cost stays flat:

```c
int counter1 = 0 ;
void redrawScreen() {
  drawWater(counter1) ;
  drawWater(counter1 + 56) ;
  drawWater(counter1 + 112) ;
  drawWater(counter1 + 168) ;
  drawWater(counter1 + 224) ;
  // ... and the nucleus circles at the same five indices
  counter1 = (counter1 == 55) ? 0 : counter1 + 1 ;
  // a second counter cycles graphite and control rods on the same pattern
}
```

**(d) Stagger the stats too.** The seven numerical fields in the stats row (spare µs, active neutrons, target, water flow, mode, sim speed, decay count) are refreshed round-robin, one per frame — about 4 Hz per field. Updating them all every frame would cause a visible flicker on the digits and a regular tail-latency spike for the same reason as (c). The slower update rate also helps legibility — the same reason a hospital vitals monitor doesn't redraw at 60 Hz.

### 5.4 Overclocking

The Pico SDK exposes `set_sys_clock_khz` for pushing the system clock past the 125 MHz default; the simulation runs at 250 MHz. The one thing to remember when overclocking is to double the PIO `clkdiv` (5 → 10) so the VGA state machines still tick at 25 MHz. Without that, the pixel clock doubles and the monitor refuses the signal.

---

## 6\. Results

The simulation has two practical operating bounds, and they don't quite meet. At roughly **600 active neutrons** the loop sustains the 30 FPS target — the on-screen `spare_time` µs counter stays comfortably positive. Above ~600, the frame budget begins to slip. The neutron population, however, never approaches the hardware ceiling of `neutrons_max = 2000`; in steady state it tops out around **1100**, because the U-235 regeneration rate in `spontaneous()` cannot replenish uranium fast enough to keep pace with fissions at that scale. So while the chip is the bottleneck above 600, it never has to be — under normal control-rod settings the equilibrium population sits well below the 30-FPS bound, and the natural physics cap (~1100) is hit only with manual override of control rods (to simulate the stuck state) or messing with the "fun" knob of neutrons emitted per collision.

The qualitative proof — what the simulation actually looks and sounds like at the operating point — is in the [video demo](https://www.youtube.com/watch?v=SqB7Jm-Cdmk).

---

## 7\. Debugging and Challenges

### 7.1 `spare_time` on screen

The single most useful debugging tool was already in the code: at the end of every frame, the animation loop computes `spare_time = FRAME_RATE − elapsed` and writes that value to the stats row in microseconds. When it stays positive and stable, the simulation has headroom. When it drifts toward zero, the next optimization or particle bump is going to break things. When it goes negative, frames are slipping right now. Every optimization in §5 was guided by watching that number on the actual monitor while toggling features and pushing particle counts — no profiler, no logic analyzer, just the report on the screen while the simulation ran.

### 7.2 The red-trail bug

The pixel-cache trick from §5.3(b) — sample the pixels under a neutron, stash them, restore them on erase — works perfectly until two neutrons fly through the same square within a frame of each other. The trailing neutron's `readPixel` call samples the *leading* neutron, which is still on screen, and caches its red (fast) or pink (thermal) color as if it were background. On the next frame, the trailing neutron erases its old position by writing those red/pink pixels back, and the leading neutron's footprint becomes a streak that the simulation now believes is supposed to be there.

The fix is one line in the cache helper: if `readPixel` returns one of the neutron colors, substitute black. The cost is that two neutrons sharing a path now leave a *black* streak instead of a red one — visible against the blue water but cleaned up by the staggered refresh (§5.3(c)) within a couple of seconds. The streak shows up briefly in the demo video at moments of high neutron density. The trade was easy: red trails look like fission tracks and mislead the eye; black trails read as harmless rendering artifacts.

---

## 8\. Lessons Learned

Three takeaways from this project that generalize beyond it.

**The PIO is a real superpower.** The RP2040 has no video peripheral, but the PIO is a tiny deterministic state machine that can bit-bang any clocked protocol — VGA in this project, audio in the next one, a custom serial bus in the one after that. Once you understand how to write to it, you stop seeing missing peripherals as a dealbreaker.

**Push streaming to DMA, push timing to PIO, let the CPU do logic.** The VGA driver and the audio subsystem use the same pattern: a peripheral plus DMA so that streaming is autonomous, and the CPU is only ever responsible for *producing* data, not transferring it. This is the right mental model for fighting for cycles on a small chip.

**Build the profiler into the UI.** The on-screen `spare_time` µs counter turned every optimization into a directly observable experiment, with no external tooling. If timing matters, instrument the most-likely-to-slip metric and make it visible while the system runs.

---

## 9\. Future Work

The project is already pretty squeezed, so the candidate next steps come with real engineering tradeoffs rather than free wins. Two directions: more out of the chip, and a more faithful physical model.

### 9.1 More cycles, more memory

A custom **16-bit fixed-point type (`fix10`)** would halve the memory for every `x`, `y`, `vx`, `vy` field. Display coordinates only need 10 bits of integer range (max 640), leaving 6 fractional bits — enough resolution for the simulation's particle speeds. Across two thousand `Neutron` structs and 280 `Nucleus` structs, this is the largest single memory win on the table.

**Bit-packed booleans.** Many `short` fields store a single flag and waste fifteen bits each. Packing eight `active` flags into one byte would free real memory across the neutron pool, at the cost of bit-masking on every access. Worth measuring; not obviously a win.

**External PSRAM as VRAM.** The 4-bit color palette is set by SRAM size — the framebuffer alone is already 58% of the chip. Wiring an external PSRAM (e.g. [polpo/rp2040-psram](https://github.com/polpo/rp2040-psram)) would unlock 8-bit or wider color, with corresponding gains in legibility and the room for a more readable HUD. This is a hardware revision more than a software one.

### 9.2 Better physics

The current simulation is a 2D cross-section of a fundamentally cylindrical reactor. A 3D simulation would be a meaningfully different project — both for the physics it exposes (axial flux distribution, end effects) and for the data structures and rendering it requires — but it is the most interesting direction the project could go.

Smaller incremental wins on the existing 2D model: a live **void-coefficient** display (the simulation already has all the inputs and the void coefficient is the fundamental reason the RBMK design is dangerous); more tunable parameters (enrichment ratio, fission cross-sections per species); secondary fission products like iodine and krypton with their feedback on the reaction rate.

The simulation is fundamentally a teaching tool, and each of these would tighten its match to the physics it is trying to illustrate.

---

## 10\. Reproducibility

**Hardware**

* Raspberry Pi Pico (RP2040)
* VGA monitor and connector (hsync, vsync, R, G, B)
* External 12-bit DAC (MCP4822 class) on SPI, with a 3.5 mm audio output jack
* 4 momentary push buttons
* 4 rotary encoders
* Full PCB schematic and BOM in Tyler's [hardware report](https://ece4760.github.io/Projects/Spring2025/ks875_ttw24_3rdGroupMember/hd.html).

**Build**

* Pico SDK, CMake, `arm-none-eabi-gcc`
* Source on the ECE 4760 [project page](https://ece4760.github.io/Projects/Spring2025/ks875_ttw24_3rdGroupMember/code.html) (linked in §1)