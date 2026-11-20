# SPDX-License-Identifier: Apache-2.0
"""
Host-side driver for the protocol emulator (docs/isa.md, section 6).

The command encoding lives here, separate from the transport, so the same code
can run in a cocotb test (bit-banged SPI in simulation) and later on the RP2350
(real SPI). The transport is any async function: xfer(bytes) -> bytes.
"""

THRESH_CODE = {8: 0, 1: 1, 4: 2, 7: 3}


def cfg_bytes(clkdiv=0, wrap_bottom=0, wrap_top=31, side_opt=0, side_count=0,
              out_lsb_first=0, in_shift_right=0, autopull=0, autopush=0,
              pull_thresh=8, push_thresh=8, out_base=0, out_count=1,
              set_base=0, set_count=1, in_base=0, side_base=0, jmp_pin=0):
    """Config registers 0..8 for one state machine."""
    return bytes([
        clkdiv & 0xFF,
        (clkdiv >> 8) & 0xFF,
        wrap_bottom & 0x1F,
        (wrap_top & 0x1F) | (side_opt & 1) << 5 | (side_count & 3) << 6,
        (out_lsb_first & 1) | (in_shift_right & 1) << 1 | (autopull & 1) << 2 | (autopush & 1) << 3
        | THRESH_CODE[pull_thresh] << 4 | THRESH_CODE[push_thresh] << 6,
        (out_base & 7) | (out_count & 0xF) << 3,
        (set_base & 7) | (set_count & 7) << 3,
        (in_base & 7) | (side_base & 7) << 3,
        jmp_pin & 7,
    ])


class ProtoEmuHost:
    def __init__(self, xfer):
        self.xfer = xfer

    async def write_imem(self, words, addr=0):
        payload = bytearray([0x10, addr])
        for w in words:
            payload += bytes([w >> 8, w & 0xFF])
        await self.xfer(bytes(payload))

    async def write_cfg(self, sm, reg, data):
        await self.xfer(bytes([0x20 | sm, reg]) + bytes(data))

    async def write_odmask(self, mask):
        await self.xfer(bytes([0x20, 9, mask]))

    async def load(self, sm, prog, **cfg):
        """Load an assembled program (pioasm.Program) at its origin and configure SM `sm`."""
        c = prog.config()
        await self.write_imem(prog.words, addr=prog.origin)
        await self.write_cfg(sm, 0, cfg_bytes(
            wrap_bottom=c["wrap_bottom"], wrap_top=c["wrap_top"],
            side_opt=c["side_opt"], side_count=c["side_count"], **cfg))

    async def exec(self, sm, word):
        await self.xfer(bytes([0x80 | sm, word >> 8, word & 0xFF]))

    async def enable(self, mask):
        await self.xfer(bytes([0x60 | mask]))

    async def push(self, sm, data):
        await self.xfer(bytes([0x30 | sm]) + bytes(data))

    async def pop(self, sm, n):
        return list((await self.xfer(bytes([0x40 | sm]) + bytes(n)))[1:])

    async def status(self):
        s = (await self.xfer(bytes([0x50, 0x00])))[1]
        return {"rx_nonempty": s >> 4, "tx_full": s & 0xF}

    async def irq_clear(self, mask):
        await self.xfer(bytes([0x70, mask]))
