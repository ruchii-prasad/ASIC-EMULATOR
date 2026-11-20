# SPDX-License-Identifier: Apache-2.0
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, ReadOnly


async def setup(dut, div):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.en.value = 0
    dut.div.value = div
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    dut.en.value = 1


async def tick_cycles(dut, n_ticks):
    """Return the clock-cycle index of each of the next n_ticks ticks."""
    seen, cycle = [], 0
    while len(seen) < n_ticks:
        await RisingEdge(dut.clk)
        await ReadOnly()
        cycle += 1
        if dut.tick.value == 1:
            seen.append(cycle)
    return seen


@cocotb.test()
async def test_period_matches_div(dut):
    """Tick period is exactly div+1 for a spread of divisor values."""
    await setup(dut, 0)
    for div in [0, 1, 2, 7, 433] + [random.randint(0, 300) for _ in range(5)]:
        await FallingEdge(dut.clk)      # change inputs away from the sampling edge
        dut.en.value = 0
        dut.div.value = div
        await FallingEdge(dut.clk)
        dut.en.value = 1
        t = await tick_cycles(dut, 6)
        gaps = {b - a for a, b in zip(t, t[1:])}
        assert gaps == {div + 1}, f"div={div}: gaps {gaps}"


@cocotb.test()
async def test_tick_is_one_cycle(dut):
    """For div>0 the tick never stays high two cycles in a row."""
    await setup(dut, 3)
    prev = 0
    for _ in range(100):
        await RisingEdge(dut.clk)
        await ReadOnly()
        cur = int(dut.tick.value)
        assert not (prev and cur)
        prev = cur


@cocotb.test()
async def test_disable_stops_ticks(dut):
    """en=0 holds tick low."""
    await setup(dut, 2)
    await ClockCycles(dut.clk, 10)
    dut.en.value = 0
    await RisingEdge(dut.clk)
    for _ in range(30):
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert dut.tick.value == 0
