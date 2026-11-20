# SPDX-FileCopyrightText: © 2026 Ruchi Prasad
# SPDX-License-Identifier: Apache-2.0
#
# Top-level tests: the testbench plays the RP2350 host. Everything goes
# through the real pins: programs are loaded over the host SPI link on
# ui_in/uo_out, and protocol traffic is checked on uio.

import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, ReadOnly

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
from host import ProtoEmuHost  # noqa: E402
from pioasm import assemble    # noqa: E402

SCK, CSN, MOSI = 0, 1, 2           # ui_in bits
HALF = 6                           # host SCK half-period in clk cycles (SCK = clk / 12)


def program_file(name, origin=0):
    with open(os.path.join(HERE, "..", "programs", name)) as f:
        return assemble(f.read(), origin=origin)


async def reset(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())  # 50 MHz
    dut.ena.value = 1
    dut.ui_in.value = 1 << CSN       # CS_n idle high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


def make_host(dut):
    """Bit-banged SPI mode 0 master on ui_in, reading MISO from uo_out[0]."""
    async def xfer(data):
        reply = []
        await FallingEdge(dut.clk)
        dut.ui_in.value = 0                                  # CS_n low
        await ClockCycles(dut.clk, HALF)
        for byte in data:
            r = 0
            for i in range(7, -1, -1):
                bit = (byte >> i) & 1
                dut.ui_in.value = bit << MOSI                # SCK low, set MOSI
                await ClockCycles(dut.clk, HALF)
                r = (r << 1) | (int(dut.uo_out.value) & 1)   # sample MISO just before the rising edge
                dut.ui_in.value = bit << MOSI | 1 << SCK     # SCK high
                await ClockCycles(dut.clk, HALF)
            reply.append(r)
        dut.ui_in.value = 0
        await ClockCycles(dut.clk, HALF)
        dut.ui_in.value = 1 << CSN                           # CS_n high
        await ClockCycles(dut.clk, 2 * HALF)
        return bytes(reply)
    return ProtoEmuHost(xfer)


def uio_bit(dut, i):
    return (int(dut.uio_out.value) >> i) & 1


def oe_bit(dut, i):
    return (int(dut.uio_oe.value) >> i) & 1


async def record(dut, n, fn):
    out = []
    for _ in range(n):
        await RisingEdge(dut.clk)
        await ReadOnly()
        out.append(fn())
    return out


def uart_decode(samples, bit_cycles, count):
    got, i = [], 0
    while len(got) < count:
        i = samples.index(0, i)
        mid = i + bit_cycles // 2
        assert samples[mid] == 0, "start bit"
        got.append(sum(samples[mid + bit_cycles * (k + 1)] << k for k in range(8)))
        assert samples[mid + bit_cycles * 9] == 1, "stop bit"
        i = mid + bit_cycles * 9
    return got


async def setup_uart(host, sm, tx_pin, clkdiv, origin=0):
    prog = program_file("uart_tx.pio", origin)
    await host.load(sm, prog, clkdiv=clkdiv, out_lsb_first=1, out_base=tx_pin, out_count=1,
                    set_base=tx_pin, set_count=1, side_base=tx_pin)
    await host.exec(sm, assemble("set pindirs, 1").words[0])
    await host.exec(sm, assemble("set pins, 1").words[0])


async def spi_device(dut, sck, mosi, miso, reply, log):
    """SPI mode-0 target on uio: samples MOSI on SCK rise, shifts MISO on SCK fall."""
    bits = [(b >> (7 - i)) & 1 for b in reply for i in range(8)]
    prev, idx = 0, 0
    while True:
        await FallingEdge(dut.clk)
        out = int(dut.uio_out.value) & int(dut.uio_oe.value)   # undriven pins read as 0
        s, m = (out >> sck) & 1, (out >> mosi) & 1
        if s and not prev:
            log.append(m)
        if prev and not s:
            idx += 1
        dut.uio_in.value = (bits[idx] if idx < len(bits) else 0) << miso
        prev = s


def bits_to_bytes(b):
    return [sum(b[i + k] << (7 - k) for k in range(8)) for i in range(0, len(b) - 7, 8)]


# ============================ tests ============================

@cocotb.test()
async def test_pins_safe_after_reset(dut):
    """After reset nothing is driven: protocol pins are inputs, MISO and IRQ low."""
    await reset(dut)
    for _ in range(50):
        assert dut.uio_oe.value == 0
        assert int(dut.uo_out.value) & 0b11 == 0
        await ClockCycles(dut.clk, 1)


@cocotb.test()
async def test_status_reports_fifos(dut):
    await reset(dut)
    host = make_host(dut)
    assert await host.status() == {"rx_nonempty": 0, "tx_full": 0}
    await host.push(1, [1, 2, 3, 4])                    # SM1 disabled: its TX FIFO fills
    assert await host.status() == {"rx_nonempty": 0, "tx_full": 0b10}
    assert await host.pop(0, 2) == [0, 0], "popping an empty RX FIFO returns 0x00"


@cocotb.test()
async def test_uart_tx_loaded_over_host_spi(dut):
    """Load uart_tx.pio over the host link and receive the bytes on uio[5]."""
    await reset(dut)
    host = make_host(dut)
    TX, DIV = 5, 3                                       # 4 clk per SM step, 32 clk per bit
    await setup_uart(host, 0, TX, DIV)
    assert oe_bit(dut, TX) == 1 and uio_bit(dut, TX) == 1, "TX is an output, idling high"

    data = [0x48, 0x69, 0x21]                            # "Hi!"
    await host.push(0, data)
    await host.enable(0b01)
    samples = await record(dut, 32 * 10 * len(data) + 200, lambda: uio_bit(dut, TX))
    assert uart_decode(samples, 32, len(data)) == data
    assert int(dut.uo_out.value) >> 7 == 1, "debug pin: SM0 stalled in `pull` afterwards"


@cocotb.test()
async def test_spi_controller_with_device(dut):
    """SM0 runs spi_mode0.pio against a device model; replies are read back over the host link."""
    await reset(dut)
    host = make_host(dut)
    P_SCK, P_MOSI, P_MISO = 0, 1, 2
    send = [0x9F, 0x12, 0xC3, 0x7E]
    reply = [0xEF, 0x40, 0x18, 0x5A]
    log = []
    cocotb.start_soon(spi_device(dut, P_SCK, P_MOSI, P_MISO, reply, log))

    await host.load(0, program_file("spi_mode0.pio"), clkdiv=1, side_base=P_SCK,
                    out_base=P_MOSI, out_count=1, in_base=P_MISO, set_base=P_SCK, set_count=2,
                    autopull=1, autopush=1)
    await host.exec(0, assemble("set pindirs, 0b11").words[0])
    await host.push(0, send)
    await host.enable(0b01)
    await ClockCycles(dut.clk, 2 * 4 * 8 * len(send) + 50)

    assert bits_to_bytes(log)[:4] == send, "device received MOSI bytes"
    assert await host.pop(0, 4) == reply, "host read MISO bytes from the RX FIFO"
    assert await host.pop(0, 1) == [0], "RX FIFO now empty"


@cocotb.test()
async def test_two_state_machines_at_once(dut):
    """SM0 sends UART on uio[7] while SM1 runs SPI on uio[2:0], from one instruction memory."""
    await reset(dut)
    host = make_host(dut)
    await setup_uart(host, 0, tx_pin=7, clkdiv=3, origin=0)       # words 0..3

    log = []
    reply = [0xA1, 0xB2, 0xC3]
    cocotb.start_soon(spi_device(dut, 0, 1, 2, reply, log))
    await host.load(1, program_file("spi_mode0.pio", origin=4), clkdiv=1, side_base=0,
                    out_base=1, out_count=1, in_base=2, set_base=0, set_count=2,
                    autopull=1, autopush=1)                       # words 4..5
    await host.exec(1, assemble("set pindirs, 0b11").words[0])

    uart_data = [0x55, 0xF0]
    spi_data = [0x01, 0x80, 0xFF]
    await host.push(0, uart_data)
    await host.push(1, spi_data)
    await host.enable(0b11)
    samples = await record(dut, 32 * 10 * len(uart_data) + 200, lambda: uio_bit(dut, 7))

    assert uart_decode(samples, 32, len(uart_data)) == uart_data
    assert bits_to_bytes(log)[:3] == spi_data
    assert await host.pop(1, 3) == reply


@cocotb.test()
async def test_open_drain_pin(dut):
    """An open-drain pin pulls low for 0 and releases (oe=0) for 1; it never drives high."""
    await reset(dut)
    host = make_host(dut)
    P = 3
    await host.write_odmask(1 << P)
    await host.write_cfg(0, 6, [P | 1 << 3])                      # set_base = 3, set_count = 1
    await host.exec(0, assemble("set pindirs, 1").words[0])
    await host.exec(0, assemble("set pins, 0").words[0])
    assert oe_bit(dut, P) == 1 and uio_bit(dut, P) == 0, "drives low"
    await host.exec(0, assemble("set pins, 1").words[0])
    assert oe_bit(dut, P) == 0, "released: the pull-up makes it high"
    assert uio_bit(dut, P) == 0, "never drives high"


@cocotb.test()
async def test_irq_pin_and_host_clear(dut):
    await reset(dut)
    host = make_host(dut)
    await host.exec(0, assemble("irq 2").words[0])
    assert (int(dut.uo_out.value) >> 1) & 1 == 1, "IRQ output goes high"
    await host.irq_clear(1 << 2)
    assert (int(dut.uo_out.value) >> 1) & 1 == 0


@cocotb.test()
async def test_reset_returns_to_safe_state(dut):
    """A running design returns to the safe state on rst_n (competition rule)."""
    await reset(dut)
    host = make_host(dut)
    await setup_uart(host, 0, tx_pin=4, clkdiv=0)
    await host.push(0, [random.randrange(256)])
    await host.enable(0b01)
    await ClockCycles(dut.clk, 30)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 2)
    await ReadOnly()
    assert int(dut.uio_oe.value) == 0 and int(dut.uo_out.value) & 0b11 == 0
