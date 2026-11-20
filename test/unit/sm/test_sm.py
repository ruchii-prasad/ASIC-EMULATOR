# SPDX-License-Identifier: Apache-2.0
# Unit tests for the state machine core. Run with: make
# Step 1: JMP, OUT, PULL, SET, side-set, delay, wrap, EXEC.
# Step 2: WAIT, IN, PUSH, MOV, IRQ, autopull/autopush, SPI against a device model.
import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools"))
from pioasm import assemble  # noqa: E402

PERIOD_NS = 20  # 50 MHz


def program_file(name):
    with open(os.path.join(REPO, "programs", name)) as f:
        return assemble(f.read())


class Harness:
    """Drives sm_wrap. Inputs change on the falling edge so they never race
    with the rising edge where the design samples them."""

    def __init__(self, dut):
        self.dut = dut

    async def reset(self):
        d = self.dut
        cocotb.start_soon(Clock(d.clk, PERIOD_NS, unit="ns").start())
        for sig, val in [("rst_n", 0), ("imem_we", 0), ("imem_addr", 0), ("imem_wdata", 0),
                         ("tx_push", 0), ("tx_wdata", 0), ("en", 0), ("restart", 0),
                         ("clkdiv", 0), ("exec_valid", 0), ("exec_instr", 0),
                         ("wrap_bottom", 0), ("wrap_top", 31), ("side_opt", 0),
                         ("side_count", 0), ("out_shift_right", 1), ("pull_thresh", 8),
                         ("out_base", 0), ("out_count", 1), ("set_base", 0),
                         ("set_count", 1), ("side_base", 0), ("jmp_pin", 0), ("pins_in", 0),
                         ("in_shift_right", 0), ("autopull", 0), ("autopush", 0),
                         ("push_thresh", 8), ("in_base", 0), ("rx_pop", 0),
                         ("host_irq_set", 0), ("host_irq_clr", 0)]:
            getattr(d, sig).value = val
        await ClockCycles(d.clk, 3)
        await FallingEdge(d.clk)
        d.rst_n.value = 1

    async def load(self, prog, **cfg):
        """Write the program into instruction memory and apply its config."""
        d = self.dut
        for addr, word in enumerate(prog.words):
            await FallingEdge(d.clk)
            d.imem_we.value, d.imem_addr.value, d.imem_wdata.value = 1, addr, word
        await FallingEdge(d.clk)
        d.imem_we.value = 0
        c = prog.config()
        d.wrap_bottom.value = c["wrap_bottom"]
        d.wrap_top.value = c["wrap_top"]
        d.side_opt.value = c["side_opt"]
        d.side_count.value = c["side_count"]
        for name, val in cfg.items():
            getattr(d, name).value = val

    async def write_word(self, addr, word):
        d = self.dut
        await FallingEdge(d.clk)
        d.imem_we.value, d.imem_addr.value, d.imem_wdata.value = 1, addr, word
        await FallingEdge(d.clk)
        d.imem_we.value = 0

    async def exec(self, src):
        """Run one instruction immediately, like the host EXEC command."""
        d = self.dut
        await FallingEdge(d.clk)
        d.exec_valid.value, d.exec_instr.value = 1, assemble(src).words[0]
        await FallingEdge(d.clk)
        d.exec_valid.value = 0

    async def push(self, byte):
        d = self.dut
        await FallingEdge(d.clk)
        d.tx_push.value, d.tx_wdata.value = 1, byte
        await FallingEdge(d.clk)
        d.tx_push.value = 0

    async def pop_all(self):
        """Drain the RX FIFO and return its contents."""
        d, out = self.dut, []
        await FallingEdge(d.clk)
        while int(d.rx_empty.value) == 0:
            out.append(int(d.rx_rdata.value))
            d.rx_pop.value = 1
            await FallingEdge(d.clk)
            d.rx_pop.value = 0
            await FallingEdge(d.clk)
        return out

    async def pulse(self, sig, val):
        """Drive a host signal for exactly one clock."""
        d = self.dut
        await FallingEdge(d.clk)
        getattr(d, sig).value = val
        await FallingEdge(d.clk)
        getattr(d, sig).value = 0

    async def start(self):
        d = self.dut
        await FallingEdge(d.clk)
        d.restart.value = 1
        await FallingEdge(d.clk)
        d.restart.value, d.en.value = 0, 1

    async def record(self, n, fn):
        """Sample fn() just after each of the next n rising edges."""
        out = []
        for _ in range(n):
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            out.append(fn())
        return out


def pin(dut, i):
    return (int(dut.pin_val.value) >> i) & 1


def uart_bits(byte):
    return [(byte >> i) & 1 for i in range(8)]  # LSB first


async def uart_setup(h, tx_pin, clkdiv):
    await h.reset()
    await h.load(program_file("uart_tx.pio"), clkdiv=clkdiv, out_base=tx_pin, out_count=1,
                 side_base=tx_pin, set_base=tx_pin, set_count=1, out_shift_right=1)
    await h.exec("set pindirs, 1")   # TX is an output
    await h.exec("set pins, 1")      # idle high before the program starts


@cocotb.test()
async def test_uart_tx_matches_hand_trace(dut):
    """UART TX of 0x41 with one SM step per clock reproduces the hand-traced table."""
    h = Harness(dut)
    TX = 2
    await uart_setup(h, TX, clkdiv=0)
    assert int(dut.pin_dir.value) == 1 << TX
    await h.push(0x41)
    await h.start()

    trace = await h.record(140, lambda: (int(dut.pc.value), pin(dut, TX)))
    tx = [t for _, t in trace]

    # Ticks 1-8: `pull side 1 [7]` holds TX high for exactly 8 steps
    # (PC already shows 1 while the delay runs).
    assert sum(1 for pc, t in trace if pc == 1 and t == 1) == 8

    start = tx.index(0)
    assert tx[start:start + 8] == [0] * 8, "start bit must be 8 steps"
    for k, bit in enumerate(uart_bits(0x41)):
        seg = tx[start + 8 + 8 * k: start + 16 + 8 * k]
        assert seg == [bit] * 8, f"data bit {k}: expected {bit}x8, got {seg}"
    assert tx[start + 72:start + 80] == [1] * 8, "stop bit"
    assert all(t == 1 for t in tx[start + 72:]), "line idles high after the stop bit"

    # After the final `jmp x--` X wrapped from 0 to 0xFF (8-bit datapath)
    assert int(dut.u_sm.x.value) == 0xFF
    assert int(dut.stalled.value) == 1, "SM waits in `pull` for the next byte"


@cocotb.test()
async def test_uart_tx_115200_decodes(dut):
    """At CLKDIV=53 (115741 baud) a software UART receiver decodes back-to-back bytes."""
    h = Harness(dut)
    TX = 0
    await uart_setup(h, TX, clkdiv=53)
    data = [0x55, 0xA3] + [random.randrange(256) for _ in range(2)]
    for b in data:
        await h.push(b)
    await h.start()

    bit_cycles = 54 * 8
    tx = await h.record(bit_cycles * 10 * len(data) + 2 * bit_cycles, lambda: pin(dut, TX))

    got, i = [], 0
    while len(got) < len(data):
        i = tx.index(0, i)                           # falling edge = start of start bit
        mid = i + bit_cycles // 2
        assert tx[mid] == 0, "start bit"
        byte = sum(tx[mid + bit_cycles * (k + 1)] << k for k in range(8))
        assert tx[mid + bit_cycles * 9] == 1, "stop bit"
        got.append(byte)
        i = mid + bit_cycles * 9                     # resume search inside the stop bit
    assert got == data


@cocotb.test()
async def test_pull_stalls_on_empty_fifo(dut):
    """With an empty FIFO the SM waits in `pull`, keeping TX high (side-set applies while stalled)."""
    h = Harness(dut)
    TX = 0
    await uart_setup(h, TX, clkdiv=0)
    await h.start()
    trace = await h.record(50, lambda: (int(dut.pc.value), pin(dut, TX), int(dut.stalled.value)))
    assert all(pc == 0 and t == 1 for pc, t, _ in trace)
    assert trace[-1][2] == 1

    await h.push(0x00)
    tx = await h.record(40, lambda: pin(dut, TX))
    assert 0 in tx, "transmission starts once data arrives"


@cocotb.test()
async def test_set_pins_window_wraps(dut):
    """SET with base 6, count 4 covers pins 6, 7, 0, 1."""
    h = Harness(dut)
    await h.reset()
    dut.set_base.value, dut.set_count.value = 6, 4
    await h.exec("set pins, 0b1011")      # bit0->pin6, bit1->pin7, bit2->pin0, bit3->pin1
    assert int(dut.pin_val.value) == 0b1100_0010
    await h.exec("set pindirs, 0b1111")
    assert int(dut.pin_dir.value) == 0b1100_0011


@cocotb.test()
async def test_jmp_x_decrement_loop(dut):
    """`jmp x--` with X=3 jumps 3 times, falls through on X=0 and leaves X=0xFF."""
    h = Harness(dut)
    await h.reset()
    prog = assemble("""
        set x, 3
    loop:
        jmp x-- loop
        set pins, 1
    halt:
        jmp halt
    """)
    await h.load(prog, set_base=0, set_count=1)
    await h.start()
    pcs = await h.record(20, lambda: int(dut.pc.value))
    # after set x: PC=1; jmp taken 3 times (PC stays 1); 4th jmp falls through
    assert pcs.count(1) == 4, pcs
    assert pcs[-1] == 3 and pin(dut, 0) == 1
    assert int(dut.u_sm.x.value) == 0xFF


async def run_out_nibbles(dut, shift_right):
    h = Harness(dut)
    await h.reset()
    prog = assemble("""
        pull
        out pins, 4
        out pins, 4
    halt:
        jmp halt
    """)
    await h.load(prog, out_base=0, out_count=4, out_shift_right=shift_right)
    await h.push(0xA5)
    await h.start()
    vals = await h.record(20, lambda: int(dut.pin_val.value) & 0xF)
    return [v for k, v in enumerate(vals) if k == 0 or v != vals[k - 1]]


@cocotb.test()
async def test_out_msb_first(dut):
    assert await run_out_nibbles(dut, shift_right=0) == [0x0, 0xA, 0x5]


@cocotb.test()
async def test_out_lsb_first(dut):
    assert await run_out_nibbles(dut, shift_right=1) == [0x0, 0x5, 0xA]


@cocotb.test()
async def test_out_pc_and_exec_jmp(dut):
    """EXEC of `jmp 7` moves PC; `out pc, 5` jumps to the value shifted out of OSR."""
    h = Harness(dut)
    await h.reset()
    await h.exec("jmp 7")
    assert int(dut.pc.value) == 7

    prog = assemble("""
        pull
        out pc, 5
    """)
    await h.load(prog)
    await h.write_word(30, assemble("jmp 30").words[0])  # landing pad
    await h.push(0b0001_1110)          # low 5 bits = 30
    await h.start()
    pcs = await h.record(10, lambda: int(dut.pc.value))
    assert 30 in pcs


# ======================= step 2 =======================

class SpiDevice:
    """Model of an SPI mode-0 target device wired to the SM's pins.
    It samples MOSI on SCK rising edges and shifts MISO on SCK falling edges.
    Inputs are only driven on the falling clk edge, like real wiring would settle."""

    def __init__(self, dut, sck, mosi, miso, reply):
        self.dut, self.sck, self.mosi, self.miso = dut, sck, mosi, miso
        self.bits_out = [(b >> (7 - i)) & 1 for b in reply for i in range(8)]  # MSB first
        self.bits_in, self.sck_rises = [], []
        self.cycle = 0

    def received(self):
        b = self.bits_in
        return [sum(b[i + k] << (7 - k) for k in range(8)) for i in range(0, len(b) - 7, 8)]

    async def run(self):
        d, prev_sck, idx = self.dut, 0, 0
        while True:
            await FallingEdge(d.clk)
            self.cycle += 1
            pv = int(d.pin_val.value)
            sck, mosi = (pv >> self.sck) & 1, (pv >> self.mosi) & 1
            if sck and not prev_sck:
                self.bits_in.append(mosi)
                self.sck_rises.append(self.cycle)
            if prev_sck and not sck:
                idx += 1                                  # present the next bit
            bit = self.bits_out[idx] if idx < len(self.bits_out) else 0
            d.pins_in.value = bit << self.miso
            prev_sck = sck


@cocotb.test()
async def test_spi_mode0_with_device(dut):
    """spi_mode0.pio (2 instructions, autopull + autopush) exchanges bytes with a device model."""
    h = Harness(dut)
    await h.reset()
    SCK, MOSI, MISO = 0, 1, 2
    await h.load(program_file("spi_mode0.pio"), side_base=SCK, out_base=MOSI, out_count=1,
                 in_base=MISO, set_base=SCK, set_count=2, out_shift_right=0, in_shift_right=0,
                 autopull=1, autopush=1, pull_thresh=8, push_thresh=8)
    await h.exec("set pindirs, 0b11")                     # SCK and MOSI are outputs

    send = [0x9F, 0x00, 0xA5, 0x3C]
    reply = [0xEF, 0x40, 0x18, 0x5A]
    dev = SpiDevice(dut, SCK, MOSI, MISO, reply)
    cocotb.start_soon(dev.run())
    for b in send:
        await h.push(b)
    await h.start()

    await ClockCycles(dut.clk, 4 * 8 * len(send) + 40)
    assert dev.received()[:len(send)] == send, "device saw the wrong MOSI bytes"
    assert await h.pop_all() == reply, "SM read back the wrong MISO bytes"

    gaps = {b - a for a, b in zip(dev.sck_rises, dev.sck_rises[1:])}
    assert gaps == {4}, f"SCK must be a steady SM clock / 4, got periods {gaps}"
    assert int(dut.stalled.value) == 1, "waits in `out` (autopull) once the TX FIFO is empty"
    assert (int(dut.pin_val.value) >> SCK) & 1 == 0, "SCK idles low in mode 0"


async def in_x_then_push(dut, shift_right):
    h = Harness(dut)
    await h.reset()
    dut.in_shift_right.value = shift_right
    await h.exec("set x, 0b10110")
    await h.exec("in x, 5")
    await h.exec("push")
    return await h.pop_all()


@cocotb.test()
async def test_in_shift_left_then_push(dut):
    """Shift left: new bits enter at the LSB end."""
    assert await in_x_then_push(dut, 0) == [0b0001_0110]


@cocotb.test()
async def test_in_shift_right_then_push(dut):
    """Shift right: new bits enter at the MSB end."""
    assert await in_x_then_push(dut, 1) == [0b1011_0000]


@cocotb.test()
async def test_in_pins_relative_and_autopush_nibbles(dut):
    """IN PINS reads from in_base upward; autopush at threshold 4 pushes each nibble."""
    h = Harness(dut)
    await h.reset()
    prog = assemble("""
        in pins, 4
    """)
    await h.load(prog, in_base=4, autopush=1, push_thresh=4, in_shift_right=0)
    dut.pins_in.value = 0xB0                 # pins 7..4 = 1011
    await h.start()
    await ClockCycles(dut.clk, 3)
    dut.en.value = 0
    got = await h.pop_all()
    assert got and all(v == 0x0B for v in got), got


@cocotb.test()
async def test_push_iffull_and_blocking(dut):
    """PUSH IFFULL waits for the threshold; a blocking PUSH stalls on a full RX FIFO."""
    h = Harness(dut)
    await h.reset()
    dut.push_thresh.value = 8
    await h.exec("in null, 4")
    await h.exec("push iffull")              # only 4 of 8 bits: no push
    assert int(dut.rx_empty.value) == 1
    await h.exec("in null, 4")
    await h.exec("push iffull")              # now 8 bits: pushes
    assert int(dut.rx_count.value) == 1

    prog = assemble("""
        set x, 7
    loop:
        mov isr, x
        push
        jmp loop
    """)
    await h.load(prog)
    await h.start()
    await ClockCycles(dut.clk, 40)
    assert int(dut.rx_count.value) == 4, "FIFO holds 4"
    assert int(dut.stalled.value) == 1, "5th push stalls"
    assert int(dut.pc.value) == 2


@cocotb.test()
async def test_mov_operations(dut):
    h = Harness(dut)
    await h.reset()
    await h.exec("set x, 0b00011")
    await h.exec("mov y, ::x")
    assert int(dut.u_sm.y.value) == 0b1100_0000
    await h.exec("mov y, !x")
    assert int(dut.u_sm.y.value) == 0b1111_1100
    await h.exec("mov osr, y")
    assert int(dut.u_sm.osr.value) == 0b1111_1100
    await h.exec("mov x, status")             # TX FIFO empty -> 0xFF
    assert int(dut.u_sm.x.value) == 0xFF
    await h.push(1)
    await h.exec("mov x, status")
    assert int(dut.u_sm.x.value) == 0x00
    dut.out_base.value, dut.out_count.value = 4, 4
    await h.exec("mov pins, y")               # low 4 bits of 0xFC -> pins 7..4
    assert int(dut.pin_val.value) == 0xC0


@cocotb.test()
async def test_wait_pin(dut):
    """WAIT stalls until the pin reaches the requested level."""
    h = Harness(dut)
    await h.reset()
    prog = assemble("""
        wait 1 gpio 3
        wait 0 pin 1
        set pins, 1
    halt:
        jmp halt
    """)
    await h.load(prog, in_base=5, set_base=0, set_count=1)
    dut.pins_in.value = 0b0100_0000          # pin 6 (= in_base + 1) high, pin 3 low
    await h.start()
    await ClockCycles(dut.clk, 20)
    assert int(dut.pc.value) == 0 and int(dut.stalled.value) == 1
    dut.pins_in.value = 0b0100_1000          # pin 3 high
    await ClockCycles(dut.clk, 20)
    assert int(dut.pc.value) == 1, "first WAIT passed, second still waiting on pin 6"
    dut.pins_in.value = 0b0000_1000          # pin 6 low
    await ClockCycles(dut.clk, 20)
    assert int(dut.pc.value) == 3 and pin(dut, 0) == 1


@cocotb.test()
async def test_irq_wait_and_wait_irq(dut):
    """`wait 1 irq 5` blocks until the flag is set and consumes it;
    `irq wait 3` raises flag 3 and blocks until it is cleared."""
    h = Harness(dut)
    await h.reset()
    prog = assemble("""
        wait 1 irq 5
        irq wait 3
        set pins, 1
    halt:
        jmp halt
    """)
    await h.load(prog, set_base=0, set_count=1)
    await h.start()
    await ClockCycles(dut.clk, 20)
    assert int(dut.pc.value) == 0

    await h.pulse("host_irq_set", 1 << 5)
    await ClockCycles(dut.clk, 20)
    flags = int(dut.irq_flags.value)
    assert not (flags >> 5) & 1, "WAIT 1 IRQ consumed flag 5"
    assert (flags >> 3) & 1, "IRQ WAIT raised flag 3"
    assert int(dut.pc.value) == 1 and int(dut.stalled.value) == 1

    await h.pulse("host_irq_clr", 1 << 3)
    await ClockCycles(dut.clk, 20)
    assert int(dut.pc.value) == 3 and pin(dut, 0) == 1


@cocotb.test()
async def test_irq_set_and_clear(dut):
    h = Harness(dut)
    await h.reset()
    await h.exec("irq 6")
    assert int(dut.irq_flags.value) == 1 << 6
    await h.exec("irq clear 6")
    assert int(dut.irq_flags.value) == 0


@cocotb.test()
async def test_wait_low_on_absolute_pin(dut):
    """`wait 0 gpio 4` stalls while pin 4 is high (polarity 0 on an absolute pin).
    Added after mutation testing showed WAIT polarity was unchecked for gpio sources."""
    h = Harness(dut)
    await h.reset()
    prog = assemble("""
        wait 0 gpio 4
        set pins, 1
    halt:
        jmp halt
    """)
    await h.load(prog, set_base=0, set_count=1)
    dut.pins_in.value = 1 << 4
    await h.start()
    await ClockCycles(dut.clk, 20)
    assert int(dut.pc.value) == 0 and pin(dut, 0) == 0
    dut.pins_in.value = 0
    await ClockCycles(dut.clk, 20)
    assert int(dut.pc.value) == 2 and pin(dut, 0) == 1
