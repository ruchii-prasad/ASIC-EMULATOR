#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
pioasm.py — assembler for the protocol-engine ISA (docs/isa.md, v0.1).

Usage:
    python tools/pioasm.py programs/uart_tx.pio                 # print listing
    python tools/pioasm.py programs/uart_tx.pio -o uart_tx.hex  # $readmemh file
    python tools/pioasm.py programs/uart_tx.pio --json          # words + config

Syntax (close to RP2040 pioasm):
    .side_set <n> [opt]        n = 0..2
    .wrap_target / .wrap
    label:
    <instr> [side <v>] [[delay]]
    ; or // start a comment
"""

import argparse
import json
import re
import sys
from dataclasses import dataclass, field

IMEM_SIZE = 32

OPC = {"jmp": 0b000, "wait": 0b001, "in": 0b010, "out": 0b011,
       "push": 0b100, "pull": 0b100, "mov": 0b101, "irq": 0b110, "set": 0b111}

JMP_COND = {"": 0, "!x": 1, "x--": 2, "!y": 3, "y--": 4, "x!=y": 5, "pin": 6, "!osre": 7}
WAIT_SRC = {"gpio": 0, "pin": 1, "irq": 2}
IN_SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "isr": 6, "osr": 7}
OUT_DST = {"pins": 0, "x": 1, "y": 2, "null": 3, "pindirs": 4, "pc": 5, "isr": 6}
MOV_DST = {"pins": 0, "x": 1, "y": 2, "pindirs": 3, "pc": 4, "isr": 6, "osr": 7}
MOV_SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "status": 5, "isr": 6, "osr": 7}
SET_DST = {"pins": 0, "x": 1, "y": 2, "pindirs": 4}


class AsmError(Exception):
    pass


@dataclass
class Program:
    words: list = field(default_factory=list)
    side_count: int = 0
    side_opt: bool = False
    wrap_bottom: int = 0
    wrap_top: int = 0
    source_lines: list = field(default_factory=list)  # (addr, text) for listings

    def config(self):
        return {"side_count": self.side_count, "side_opt": int(self.side_opt),
                "wrap_bottom": self.wrap_bottom, "wrap_top": self.wrap_top}


def parse_int(tok, line_no):
    try:
        return int(tok, 0)
    except ValueError:
        raise AsmError(f"line {line_no}: expected a number, got '{tok}'")


def lookup(table, key, what, line_no):
    if key not in table:
        raise AsmError(f"line {line_no}: unknown {what} '{key}' (valid: {', '.join(k or 'always' for k in table)})")
    return table[key]


def check_range(val, lo, hi, what, line_no):
    if not lo <= val <= hi:
        raise AsmError(f"line {line_no}: {what} {val} out of range {lo}..{hi}")
    return val


def strip_comment(line):
    return re.split(r";|//", line, maxsplit=1)[0].strip()


def assemble(text):
    prog = Program()
    lines = text.splitlines()

    # ---- pass 1: directives, labels, instruction addresses ----
    labels, instrs = {}, []          # instrs: (addr, line_no, text)
    wrap_target = wrap = None
    for line_no, raw in enumerate(lines, 1):
        line = strip_comment(raw)
        if not line:
            continue
        m = re.match(r"^([A-Za-z_]\w*)\s*:\s*(.*)$", line)
        if m:
            name, line = m.group(1), m.group(2)
            if name in labels:
                raise AsmError(f"line {line_no}: duplicate label '{name}'")
            labels[name] = len(instrs)
            if not line:
                continue
        if line.startswith("."):
            parts = line.split()
            d = parts[0].lower()
            if d == ".side_set":
                if len(parts) < 2:
                    raise AsmError(f"line {line_no}: .side_set needs a count")
                prog.side_count = check_range(parse_int(parts[1], line_no), 0, 2, "side_set count", line_no)
                prog.side_opt = len(parts) > 2 and parts[2].lower() == "opt"
                if prog.side_opt and prog.side_count == 0:
                    raise AsmError(f"line {line_no}: '.side_set 0 opt' makes no sense")
            elif d == ".wrap_target":
                wrap_target = len(instrs)
            elif d == ".wrap":
                if not instrs:
                    raise AsmError(f"line {line_no}: .wrap before any instruction")
                wrap = len(instrs) - 1
            elif d == ".program":
                pass
            else:
                raise AsmError(f"line {line_no}: unknown directive '{parts[0]}'")
            continue
        instrs.append((len(instrs), line_no, line))

    if not instrs:
        raise AsmError("program has no instructions")
    if len(instrs) > IMEM_SIZE:
        raise AsmError(f"program has {len(instrs)} instructions; instruction memory holds {IMEM_SIZE}")
    prog.wrap_bottom = wrap_target if wrap_target is not None else 0
    prog.wrap_top = wrap if wrap is not None else len(instrs) - 1

    # ---- pass 2: encode ----
    delay_bits = 5 - prog.side_count - (1 if prog.side_opt else 0)
    for addr, line_no, line in instrs:
        word = encode(line, line_no, labels, prog, delay_bits)
        prog.words.append(word)
        prog.source_lines.append((addr, line))
    return prog


def encode(line, line_no, labels, prog, delay_bits):
    # Pull off the trailing "[delay]" and "side <v>" parts.
    delay = 0
    m = re.search(r"\[\s*([^\]]+)\s*\]\s*$", line)
    if m:
        delay = parse_int(m.group(1), line_no)
        line = line[:m.start()].strip()
    side = None
    m = re.search(r"\bside\s+(\S+)\s*$", line, re.IGNORECASE)
    if m:
        side = parse_int(m.group(1), line_no)
        line = line[:m.start()].strip()

    check_range(delay, 0, (1 << delay_bits) - 1, "delay", line_no)
    if side is not None:
        if prog.side_count == 0:
            raise AsmError(f"line {line_no}: 'side' used but .side_set count is 0")
        check_range(side, 0, (1 << prog.side_count) - 1, "side value", line_no)
    elif prog.side_count > 0 and not prog.side_opt:
        raise AsmError(f"line {line_no}: side-set is mandatory (.side_set without opt)")

    ss = 0
    if side is not None:
        ss = side << delay_bits
        if prog.side_opt:
            ss |= 1 << 4
    ss |= delay

    mnem, _, rest = line.partition(" ")
    mnem = mnem.lower()
    args = [a.strip().lower() for a in rest.split(",")] if rest.strip() else []
    if mnem not in OPC:
        raise AsmError(f"line {line_no}: unknown instruction '{mnem}'")
    op = OPC[mnem] << 13 | ss << 8

    def need(n):
        if len(args) != n:
            raise AsmError(f"line {line_no}: '{mnem}' takes {n} argument(s), got {len(args)}")

    def bitcount(tok):
        n = check_range(parse_int(tok, line_no), 1, 8, "bit count", line_no)
        return n & 7  # 8 is encoded as 0

    def target(tok):
        if tok in labels:
            return labels[tok]
        return check_range(parse_int(tok, line_no), 0, IMEM_SIZE - 1, "jump address", line_no)

    if mnem == "jmp":
        # Accept "jmp target", "jmp cond target" and "jmp cond, target".
        toks = rest.replace(",", " ").split()
        if not toks:
            raise AsmError(f"line {line_no}: jmp needs a target")
        tgt = toks[-1]                              # original case: labels are case-sensitive
        cond = "".join(toks[:-1]).lower()           # "x != y" -> "x!=y"
        return op | lookup(JMP_COND, cond, "jmp condition", line_no) << 5 | target(tgt)

    if mnem == "wait":
        toks = rest.split()
        if len(toks) != 3:
            raise AsmError(f"line {line_no}: wait syntax is 'wait <0|1> <gpio|pin|irq> <idx>'")
        pol = check_range(parse_int(toks[0], line_no), 0, 1, "polarity", line_no)
        src = lookup(WAIT_SRC, toks[1].lower(), "wait source", line_no)
        idx = check_range(parse_int(toks[2], line_no), 0, 7, "wait index", line_no)
        return op | pol << 7 | src << 5 | idx

    if mnem == "in":
        need(2)
        return op | lookup(IN_SRC, args[0], "in source", line_no) << 5 | bitcount(args[1])

    if mnem == "out":
        need(2)
        return op | lookup(OUT_DST, args[0], "out destination", line_no) << 5 | bitcount(args[1])

    if mnem in ("push", "pull"):
        toks = rest.lower().split()
        ifx_word = "iffull" if mnem == "push" else "ifempty"
        ifx, block = 0, 1  # blocking is the default, as in RP2040 pioasm
        for t in toks:
            if t == ifx_word:
                ifx = 1
            elif t == "block":
                block = 1
            elif t == "noblock":
                block = 0
            else:
                raise AsmError(f"line {line_no}: unexpected '{t}' after {mnem}")
        return op | (mnem == "pull") << 7 | ifx << 6 | block << 5

    if mnem == "mov":
        need(2)
        src, mop = args[1], 0
        if src.startswith("!") or src.startswith("~"):
            src, mop = src[1:].strip(), 1
        elif src.startswith("::"):
            src, mop = src[2:].strip(), 2
        return (op | lookup(MOV_DST, args[0], "mov destination", line_no) << 5
                | mop << 3 | lookup(MOV_SRC, src, "mov source", line_no))

    if mnem == "irq":
        toks = rest.lower().split()
        clr = wait = 0
        if len(toks) == 2:
            mode = toks[0]
            if mode == "clear":
                clr = 1
            elif mode == "wait":
                wait = 1
            elif mode != "set":
                raise AsmError(f"line {line_no}: irq mode must be set, wait or clear")
            toks = toks[1:]
        if len(toks) != 1:
            raise AsmError(f"line {line_no}: irq syntax is 'irq [set|wait|clear] <idx>'")
        idx = check_range(parse_int(toks[0], line_no), 0, 7, "irq index", line_no)
        return op | clr << 6 | wait << 5 | idx

    if mnem == "set":
        need(2)
        data = check_range(parse_int(args[1], line_no), 0, 31, "set value", line_no)
        return op | lookup(SET_DST, args[0], "set destination", line_no) << 5 | data

    raise AsmError(f"line {line_no}: unhandled instruction '{mnem}'")


def listing(prog):
    out = []
    for (addr, text), word in zip(prog.source_lines, prog.words):
        mark = ""
        if addr == prog.wrap_bottom:
            mark += " <- wrap_target"
        if addr == prog.wrap_top:
            mark += " <- wrap"
        out.append(f"{addr:2d}: {word:04X}   {text}{mark}")
    out.append(f"config: {prog.config()}")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Assemble a protocol-engine program.")
    ap.add_argument("source")
    ap.add_argument("-o", "--output", help="write a $readmemh hex file (one 16-bit word per line)")
    ap.add_argument("--json", action="store_true", help="print words and SM config as JSON")
    a = ap.parse_args(argv)
    try:
        with open(a.source) as f:
            prog = assemble(f.read())
    except AsmError as e:
        print(f"{a.source}: error: {e}", file=sys.stderr)
        return 1
    if a.output:
        with open(a.output, "w") as f:
            f.write("\n".join(f"{w:04x}" for w in prog.words) + "\n")
    if a.json:
        print(json.dumps({"words": prog.words, **prog.config()}, indent=2))
    elif not a.output:
        print(listing(prog))
    return 0


if __name__ == "__main__":
    sys.exit(main())
