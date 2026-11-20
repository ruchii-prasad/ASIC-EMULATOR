# SPDX-FileCopyrightText: © 2026 Ruchi Prasad
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


async def reset(dut):
    clock = Clock(dut.clk, 20, unit="ns")  # 50 MHz
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_pins_safe_after_reset(dut):
    """After reset nothing is driven: outputs low, bidirectional pins are inputs."""
    await reset(dut)
    for _ in range(50):
        assert dut.uo_out.value == 0
        assert dut.uio_oe.value == 0
        await ClockCycles(dut.clk, 1)
