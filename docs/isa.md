# Protocol Engine — Architecture & ISA (draft v0.2)

Status: **draft**. Field widths and register maps may change once synthesis
tells us the real area cost. Every change is recorded in the changelog at the bottom.

## 1. Overview

The chip is a small set of **state machines (SMs)** that run tiny programs to
toggle and sample pins with cycle-exact timing. A protocol (UART, SPI, I2C, …)
is a program, not hardware, so the chip can run protocols it was never designed for.

| Item | v1 value | Notes |
|---|---|---|
| State machines | 2 | Parameter `N_SM`; raise to 4 if area allows |
| Instruction memory | 32 × 16 bit, shared by all SMs | Loaded by the host after reset |
| Data width | 8 bit | X, Y, OSR, ISR are 8 bit |
| FIFOs per SM | TX 4 × 8, RX 4 × 8 | Host ↔ SM data path |
| Protocol pins | `uio[7:0]` (8 bidirectional) | Enough for UART + SPI + I2C at once |
| Host interface | SPI target on `ui_in` / `uo_out` | Program loading and data |
| System clock | 50 MHz (single clock domain) | All timing uses `clk_div` ticks as enables |

The design is inspired by the RP2040 PIO. Planned differences:
8-bit datapath (smaller), **per-pin open-drain mode** (I2C without software
tricks), and later a CRC/line-coding helper for USB and Ethernet.

## 2. Pin map

| Pin | Direction | Function |
|---|---|---|
| `ui_in[0]` | in | Host SPI SCK |
| `ui_in[1]` | in | Host SPI CS_n |
| `ui_in[2]` | in | Host SPI MOSI |
| `ui_in[7:3]` | in | Reserved (future: extra SM inputs) |
| `uo_out[0]` | out | Host SPI MISO |
| `uo_out[1]` | out | IRQ to host (any enabled IRQ flag set) |
| `uo_out[6:2]` | out | Debug: SM0 program counter |
| `uo_out[7]` | out | Debug: SM0 stalled |
| `uio[7:0]` | in/out | Protocol pins, numbered 0–7, used by the SMs |

Pin windows (OUT, SET, IN, side-set) are a base plus a count. They wrap modulo 8,
so base 6 with count 4 covers pins 6, 7, 0, 1.

**Open-drain mode:** each pin has a bit in `ODMASK`. For an open-drain pin,
writing 0 drives the pad low (`oe = 1`, `out = 0`), and writing 1 releases it
(`oe = 0`), so an external pull-up makes it high. `PINDIRS` is ignored for those pins.

## 3. Per-SM state

| Register | Width | Description |
|---|---|---|
| `PC` | 5 | Program counter |
| `X`, `Y` | 8 | Scratch registers and loop counters |
| `OSR` | 8 | Output shift register (data going out to pins) |
| `ISR` | 8 | Input shift register (data coming in from pins) |
| `osr_count` | 4 | Bits shifted out of OSR since the last load (0–8) |
| `isr_count` | 4 | Bits shifted into ISR since the last clear (0–8) |
| `delay_cnt` | 3 | Remaining delay cycles |

## 4. Configuration registers (per SM, written by the host)

| Reg | Name | Fields |
|---|---|---|
| 0, 1 | `CLKDIV` lo, hi | 16-bit divider. The SM advances once every `CLKDIV + 1` system clocks |
| 2 | `EXEC_LO` | `wrap_bottom[4:0]` |
| 3 | `EXEC_HI` | `wrap_top[4:0]`, `side_opt[5]`, `side_count[7:6]` (0–2) |
| 4 | `SHIFT` | `out_dir[0]` (0 = MSB-first), `in_dir[1]`, `autopull[2]`, `autopush[3]`, `pull_thresh[5:4]`\*, `push_thresh[7:6]`\* |
| 5 | `PIN_OUT` | `out_base[2:0]`, `out_count[6:3]` (0–8) |
| 6 | `PIN_SET` | `set_base[2:0]`, `set_count[5:3]` (0–5) |
| 7 | `PIN_IN` | `in_base[2:0]`, `side_base[5:3]` |
| 8 | `JMP_PIN` | `jmp_pin[2:0]` |
| 9 | `ODMASK` | One open-drain bit per `uio` pin (shared by all SMs) |

\*Thresholds are encoded as 0 = 8 bits, 1 = 1, 2 = 4, 3 = 7. **TBD:** check whether a full 1–8 range is worth the extra bit.

## 5. Instruction format

All instructions are 16 bits wide:

```
 15 14 13 | 12 11 10  9  8 | 7 6 5 4 3 2 1 0
 opcode   | side / delay   | operands
```

**Bits [12:8] (side-set and delay)** are split according to the config:

- If `side_opt = 1`, bit 12 is the side-set enable.
- The next `side_count` bits are the side-set value. It is applied to the
  side pins in the cycle the instruction starts, even if the instruction stalls.
- The remaining low bits are the delay. That many idle cycles are added after
  the instruction completes.

For example, `side_opt = 1` with `side_count = 1` gives an enable bit, 1 side bit and a 3-bit delay (0–7).
`side_count = 0` gives a 5-bit delay (0–31).

Side-set has priority over OUT and SET when both drive the same pin in the same cycle.

### 5.1 Opcodes

| [15:13] | Mnemonic | Operand bits [7:0] | Effect |
|---|---|---|---|
| `000` | `JMP cond, addr` | `cond[7:5] addr[4:0]` | If cond is true, `PC ← addr` |
| `001` | `WAIT pol, src, idx` | `pol[7] src[6:5] idx[4:0]` | Stall until the source equals `pol` |
| `010` | `IN src, n` | `src[7:5] n[2:0]` | Shift n bits (0 = 8) from src into ISR |
| `011` | `OUT dst, n` | `dst[7:5] n[2:0]` | Shift n bits out of OSR to dst |
| `100` | `PUSH` / `PULL` | `pull[7] ifx[6] block[5]` | ISR → RX FIFO, or TX FIFO → OSR |
| `101` | `MOV dst, op, src` | `dst[7:5] op[4:3] src[2:0]` | `dst ← op(src)` |
| `110` | `IRQ` | `clr[6] wait[5] idx[2:0]` | Set or clear an IRQ flag (8 flags, shared) |
| `111` | `SET dst, data` | `dst[7:5] data[4:0]` | `dst ← data` |

**JMP conditions:** 000 always · 001 `!X` · 010 `X--` (test X ≠ 0, then decrement) · 011 `!Y` · 100 `Y--` ·
101 `X != Y` · 110 `PIN` (`jmp_pin` is high) · 111 `!OSRE` (OSR not empty)

**WAIT sources:** 00 absolute pin `idx` · 01 pin `in_base + idx` · 10 IRQ flag `idx` (cleared on wake if `pol = 1`) · 11 reserved

**IN sources:** 000 `PINS` · 001 `X` · 010 `Y` · 011 `NULL` (zeros) · 110 `ISR` · 111 `OSR`

**OUT destinations:** 000 `PINS` · 001 `X` · 010 `Y` · 011 `NULL` · 100 `PINDIRS` · 101 `PC` · 110 `ISR`

**PUSH / PULL:**

- PUSH (`pull = 0`) moves ISR to the RX FIFO and clears ISR. `ifx` means push only if `isr_count ≥ push_thresh`. `block` means stall while the RX FIFO is full.
- PULL (`pull = 1`) moves the next TX FIFO entry into OSR. `ifx` means pull only if `osr_count ≥ pull_thresh`. `block` means stall while the TX FIFO is empty. A non-blocking pull from an empty FIFO copies X into OSR.

**MOV:** dst 000 `PINS` · 001 `X` · 010 `Y` · 011 `PINDIRS` · 100 `PC` · 110 `ISR` · 111 `OSR`.
op 00 none · 01 invert · 10 bit-reverse. src 000 `PINS` · 001 `X` · 010 `Y` · 011 `NULL` · 101 `STATUS` (TX FIFO empty = 0xFF) · 110 `ISR` · 111 `OSR`

**SET destinations:** 000 `PINS` · 001 `X` · 010 `Y` · 100 `PINDIRS`

**Autopull / autopush:**

- With `autopull = 1`, an `OUT` that would shift past `pull_thresh` first refills OSR from the TX FIFO, stalling if the FIFO is empty.
- With `autopush = 1`, ISR is pushed automatically once `isr_count` reaches `push_thresh`.

**Wrap:** after executing the instruction at `wrap_top` (if it did not jump), `PC ← wrap_bottom`. Loops cost no instructions.

## 6. Host SPI protocol (SPI mode 0, MSB first)

Each transaction is: CS_n goes low, a command byte, then payload bytes, then CS_n goes high.
The host SPI clock must be at most system clock / 8, which is 6.25 MHz at 50 MHz.

| Cmd | Name | Payload |
|---|---|---|
| `0x10` | WRITE_IMEM | `addr`, then `hi lo` pairs. The address auto-increments |
| `0x2s` | WRITE_CFG (SM `s`) | `reg`, then data bytes. The register number auto-increments |
| `0x3s` | TX_PUSH (SM `s`) | Bytes to push into the TX FIFO |
| `0x4s` | RX_POP (SM `s`) | Dummy bytes. MISO returns the popped data (0x00 if the FIFO is empty) |
| `0x50` | STATUS | One dummy byte. Returns `{rx_nonempty[3:0], tx_full[3:0]}` |
| `0x6m` | ENABLE | `m` = 4-bit SM enable mask. Enabling an SM resets its PC to `wrap_bottom` |
| `0x70` | IRQ_CLEAR | Mask of IRQ flags to clear |
| `0x8s` | EXEC (SM `s`) | `hi lo`: run this one instruction on SM `s` immediately. It ignores the delay field and changes PC only if it jumps. An EXEC that would stall (e.g. `pull` with an empty FIFO) is dropped |

EXEC is how the host sets pin directions and idle levels before enabling an SM.
For UART TX: `set pindirs, 1` then `set pins, 1`.

## 7. Reset and reload

- `rst_n` low clears all SM enables, config registers, FIFOs and IRQ flags, and sets all `uio_oe` to 0 (all pins are inputs).
- Instruction memory is **not** reset (to save area), so the host must reload it after every reset or deselect, as the competition rules require.

## 8. Worked examples

### 8.1 UART TX (8N1)

Config: SM clock = 8 × baud, `side_opt = 1`, `side_count = 1`, side and OUT pins = TX,
`autopull` off, `out_dir` = LSB-first.

```
.side_set 1 opt
.wrap_target
    pull        side 1 [7]   ; stop bit / idle high; waits for the next byte
    set x, 7    side 0 [7]   ; start bit, 8 cycles low
bitloop:
    out pins, 1               ; 1 cycle: drive the next data bit
    jmp x-- bitloop      [6]  ; 7 more cycles, so 8 cycles per bit
.wrap
```

The program is 4 instructions. For 115200 baud from 50 MHz:
50 MHz ÷ (8 × 115200) = 54.25, so `CLKDIV = 53` (divide by 54). That gives 115741 baud, an error of +0.47%,
well inside the ±2% that UART tolerates.

### 8.2 SPI controller, mode 0

Config: `side_count = 1` (not optional), side pin = SCK, OUT pin = MOSI, IN pin = MISO,
`autopull = 1`, `autopush = 1`, both thresholds = 8, MSB-first.

```
.side_set 1
.wrap_target
    out pins, 1   side 0 [1]   ; SCK low: put the next bit on MOSI
    in  pins, 1   side 1 [1]   ; SCK high: sample MISO
.wrap
```

The program is 2 instructions, and the SPI clock is the SM clock ÷ 4. Modes 1–3 come from swapping the side values and the order of the two instructions.

### 8.3 I2C

I2C is planned using open-drain mode on SDA and SCL. It is the hardest of the three because of
clock stretching and the ACK bit. To be written after UART and SPI pass in simulation.

## Changelog

- v0.1 (2026-11-20): first draft.
- v0.2 (2026-11-20): added the EXEC host command (`0x8s`).
