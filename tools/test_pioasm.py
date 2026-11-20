# SPDX-License-Identifier: Apache-2.0
# Run with:  python -m pytest tools
# Expected words are worked out by hand from docs/isa.md section 5.
import os
import pytest
from pioasm import assemble, AsmError

HERE = os.path.dirname(os.path.abspath(__file__))
PROGRAMS = os.path.join(HERE, "..", "programs")


def asm_file(name):
    with open(os.path.join(PROGRAMS, name)) as f:
        return assemble(f.read())


def test_uart_tx_encoding():
    p = asm_file("uart_tx.pio")
    # pull side 1 [7]   = 100 | 1 1 111 | 1 0 1 00000
    # set x, 7 side 0 [7] = 111 | 1 0 111 | 001 00111
    # out pins, 1       = 011 | 00000   | 000 00001
    # jmp x-- 2 [6]     = 000 | 0 0 110 | 010 00010
    assert p.words == [0x9FA0, 0xF727, 0x6001, 0x0642]
    assert p.config() == {"side_count": 1, "side_opt": 1, "wrap_bottom": 0, "wrap_top": 3}


def test_spi_mode0_encoding():
    p = asm_file("spi_mode0.pio")
    # out pins, 1 side 0 [1] = 011 | 0 0001 | 000 00001
    # in  pins, 1 side 1 [1] = 010 | 1 0001 | 000 00001
    assert p.words == [0x6101, 0x5101]
    assert p.config() == {"side_count": 1, "side_opt": 0, "wrap_bottom": 0, "wrap_top": 1}


@pytest.mark.parametrize("src, word", [
    ("jmp 5", 0x0005),
    ("jmp !x, 3", 0x0023),
    ("jmp pin, 0", 0x00C0),
    ("jmp !osre, 31", 0x00FF),
    ("jmp x != y 4", 0x00A4),
    ("jmp y-- 1", 0x0081),
    ("wait 1 pin 2", 0x20A2),
    ("wait 0 irq 7", 0x2047),
    ("in pins, 8", 0x4000),
    ("in null, 3", 0x4063),
    ("out pindirs, 2", 0x6082),
    ("out pc, 5", 0x60A5),
    ("push", 0x8020),
    ("push iffull noblock", 0x8040),
    ("pull ifempty", 0x80E0),
    ("pull noblock", 0x8080),
    ("mov x, y", 0xA022),
    ("mov isr, !osr", 0xA0CF),
    ("mov pins, ::x", 0xA011),
    ("irq 3", 0xC003),
    ("irq wait 1", 0xC021),
    ("irq clear 7", 0xC047),
    ("set pindirs, 3", 0xE083),
    ("set y, 31", 0xE05F),
    ("set pins, 1 [31]", 0xFF01),   # no side-set -> 5-bit delay
])
def test_single_instructions(src, word):
    assert assemble(src).words == [word]


def test_wrap_defaults_to_whole_program():
    p = assemble("set x, 1\nset y, 2\nset pins, 0")
    assert (p.wrap_bottom, p.wrap_top) == (0, 2)


def test_side_set_two_bits_opt():
    # opt + 2 side bits leaves a 2-bit delay
    p = assemble(".side_set 2 opt\nset x, 0 side 3 [3]\nset x, 0")
    assert p.words == [0xE000 | 0b11111 << 8 | 0x20, 0xE020]


@pytest.mark.parametrize("src, msg", [
    ("jmp nowhere", "expected a number"),
    ("set x, 32", "out of range"),
    ("out pins, 9", "out of range"),
    ("bogus x", "unknown instruction"),
    (".side_set 1\nset x, 1", "mandatory"),
    (".side_set 1 opt\nset x, 1 [8]", "delay"),
    ("set x, 1 side 1", "side"),
    ("a:\na:\nset x, 1", "duplicate label"),
    ("\n".join(["set x, 0"] * 33), "holds 32"),
])
def test_errors(src, msg):
    with pytest.raises(AsmError, match=msg):
        assemble(src)


def test_origin_shifts_labels_and_wrap():
    p = assemble(".wrap_target\nloop:\n set x, 1\n jmp loop\n.wrap", origin=4)
    assert p.words == [0xE021, 0x0004]
    assert (p.origin, p.wrap_bottom, p.wrap_top) == (4, 4, 5)


def test_origin_overflow():
    with pytest.raises(AsmError, match="origin 30"):
        assemble("set x, 0\nset x, 0\nset x, 0", origin=30)
