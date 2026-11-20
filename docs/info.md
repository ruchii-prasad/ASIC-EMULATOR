<!---
This file is used to generate the project datasheet.
-->

## How it works

The chip is a small programmable I/O engine for bit-banging serial protocols such as UART, SPI and I2C.
Protocols are software: each is a short program, not a fixed hardware block.

- **Two state machines** run programs from a shared 32 × 16-bit instruction memory.
  Each step can drive or sample pins, shift data through 8-bit shift registers, wait on a pin or flag, or jump.
  An optional side-set field toggles extra pins (e.g. a clock) in the same cycle.
- Each state machine has its own **clock divider**, a 4-byte **TX FIFO** (host → pins) and a 4-byte **RX FIFO** (pins → host).
- The eight bidirectional pins `uio[7:0]` are the protocol pins. Any pin can be put in **open-drain mode** for I2C-style buses.
- The host (for example the RP2350 on the Tiny Tapeout demo board) loads programs and moves data over a
  **mode-0 SPI link** on `ui_in[2:0]` / `uo_out[0]`.

The instruction set, register map and host commands are documented in `docs/isa.md` in the project repository.

## How to test

1. Reset the chip. All protocol pins start as inputs. The instruction memory must be reloaded after every reset.
2. Over the host SPI link (SCK ≤ 6.25 MHz with a 50 MHz clock):
   - `0x10 addr hi lo hi lo …` writes the program into instruction memory.
   - `0x2s reg data …` writes state machine `s`'s configuration (clock divider, wrap, pins, shifting).
   - `0x8s hi lo` runs one instruction immediately, e.g. `set pindirs, 1` to make a pin an output.
   - `0x6m` enables the state machines in mask `m`.
3. Push data with `0x3s byte …` and read results with `0x4s 00 …`. `0x50 00` returns FIFO status.

Example (UART TX at 115200 baud on `uio[0]`): assemble `programs/uart_tx.pio` with `tools/pioasm.py`,
write it to address 0, set the clock divider to 53, run `set pindirs, 1` and `set pins, 1`,
enable state machine 0, and push bytes.

## External hardware

None for UART and SPI tests: connect the target device to `uio[7:0]` (3.3 V).
I2C needs external pull-up resistors (e.g. 4.7 kΩ) on SDA and SCL.
