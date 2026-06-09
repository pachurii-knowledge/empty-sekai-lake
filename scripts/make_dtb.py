#!/usr/bin/env python3
"""Generate a minimal flattened device tree (.dtb) for the niigo-lake platform.

`dtc` is not available in this environment, so this builds the FDT (v17) blob
directly: one CPU (rv32imafd, Sv32), RAM at 0x8000_0000, and the CLINT / PLIC /
NS16550 UART devices at their niigo addresses. The blob is handed to the S-mode
kernel in a1 per the RISC-V boot protocol (see tests/boot_dtb.S).

Usage:  python3 scripts/make_dtb.py [out.dtb]   (default: output/niigo_platform.dtb)
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

FDT_BEGIN_NODE = 0x1
FDT_END_NODE = 0x2
FDT_PROP = 0x3
FDT_END = 0x9
FDT_MAGIC = 0xD00DFEED
FDT_VERSION = 17
FDT_LAST_COMP = 16


class FdtWriter:
    def __init__(self) -> None:
        self.struct = bytearray()
        self.strings = bytearray()
        self._stroff: dict[str, int] = {}

    def _str(self, name: str) -> int:
        if name not in self._stroff:
            self._stroff[name] = len(self.strings)
            self.strings += name.encode() + b"\x00"
        return self._stroff[name]

    def _align(self) -> None:
        while len(self.struct) % 4:
            self.struct += b"\x00"

    def begin_node(self, name: str) -> None:
        self.struct += struct.pack(">I", FDT_BEGIN_NODE)
        self.struct += name.encode() + b"\x00"
        self._align()

    def end_node(self) -> None:
        self.struct += struct.pack(">I", FDT_END_NODE)

    def prop(self, name: str, value: bytes) -> None:
        self.struct += struct.pack(">III", FDT_PROP, len(value), self._str(name))
        self.struct += value
        self._align()

    def prop_u32(self, name: str, *vals: int) -> None:
        self.prop(name, b"".join(struct.pack(">I", v) for v in vals))

    def prop_str(self, name: str, s: str) -> None:
        self.prop(name, s.encode() + b"\x00")

    def build(self, boot_cpuid: int = 0) -> bytes:
        self.struct += struct.pack(">I", FDT_END)
        # memory reservation block: empty (terminator only), 8-byte aligned
        rsvmap = struct.pack(">QQ", 0, 0)
        header_size = 40
        off_rsvmap = header_size
        off_struct = off_rsvmap + len(rsvmap)
        off_strings = off_struct + len(self.struct)
        total = off_strings + len(self.strings)
        header = struct.pack(
            ">IIIIIIIIII",
            FDT_MAGIC, total, off_struct, off_strings, off_rsvmap,
            FDT_VERSION, FDT_LAST_COMP, boot_cpuid,
            len(self.strings), len(self.struct),
        )
        return header + rsvmap + bytes(self.struct) + bytes(self.strings)


def build_dtb() -> bytes:
    w = FdtWriter()
    w.begin_node("")                       # root
    w.prop_u32("#address-cells", 1)
    w.prop_u32("#size-cells", 1)
    w.prop_str("compatible", "niigo,lake")
    w.prop_str("model", "niigo-lake")

    w.begin_node("chosen")
    w.prop_str("bootargs", "console=ttyS0 earlycon")
    w.prop_str("stdout-path", "/soc/serial@d000000")
    w.end_node()

    w.begin_node("cpus")
    w.prop_u32("#address-cells", 1)
    w.prop_u32("#size-cells", 0)
    w.prop_u32("timebase-frequency", 10_000_000)
    w.begin_node("cpu@0")
    w.prop_str("device_type", "cpu")
    w.prop_u32("reg", 0)
    w.prop_str("status", "okay")
    w.prop_str("compatible", "riscv")
    w.prop_str("riscv,isa", "rv32imafd")
    w.prop_str("mmu-type", "riscv,sv32")
    w.begin_node("interrupt-controller")
    w.prop_u32("#interrupt-cells", 1)
    w.prop("interrupt-controller", b"")
    w.prop_str("compatible", "riscv,cpu-intc")
    w.prop_u32("phandle", 1)
    w.end_node()
    w.end_node()
    w.end_node()

    w.begin_node("memory@80000000")
    w.prop_str("device_type", "memory")
    w.prop_u32("reg", 0x8000_0000, 0x0800_0000)   # 128 MiB RAM
    w.end_node()

    w.begin_node("soc")
    w.prop_u32("#address-cells", 1)
    w.prop_u32("#size-cells", 1)
    w.prop_str("compatible", "simple-bus")
    w.prop("ranges", b"")

    w.begin_node("clint@2000000")
    w.prop_str("compatible", "riscv,clint0")
    w.prop_u32("reg", 0x0200_0000, 0x0001_0000)
    w.prop_u32("interrupts-extended", 1, 3, 1, 7)
    w.end_node()

    w.begin_node("plic@c000000")
    w.prop_str("compatible", "riscv,plic0")
    w.prop_u32("reg", 0x0C00_0000, 0x0400_0000)
    w.prop("interrupt-controller", b"")
    w.prop_u32("#interrupt-cells", 1)
    w.prop_u32("riscv,ndev", 31)
    w.prop_u32("phandle", 2)
    w.end_node()

    w.begin_node("serial@d000000")
    w.prop_str("compatible", "ns16550a")
    w.prop_u32("reg", 0x0D00_0000, 0x0000_0100)
    # One register per 32-bit word (reg-shift=2) accessed as 32-bit words
    # (reg-io-width=4) -- matches src/uart.sv and the word-addressed device bus.
    w.prop_u32("reg-shift", 2)
    w.prop_u32("reg-io-width", 4)
    w.prop_u32("clock-frequency", 10_000_000)
    w.prop_u32("interrupt-parent", 2)
    w.prop_u32("interrupts", 10)
    w.end_node()

    w.end_node()                           # soc
    w.end_node()                           # root
    return w.build()


def emit_asm(blob: bytes, label: str = "dtb_blob") -> str:
    lines = [
        "# Generated by scripts/make_dtb.py -- do not edit by hand.",
        "    .balign 8",
        f"    .globl {label}",
        f"{label}:",
    ]
    for i in range(0, len(blob), 12):
        chunk = blob[i : i + 12]
        lines.append("    .byte " + ", ".join(f"0x{b:02x}" for b in chunk))
    lines.append(f"{label}_end:")
    return "\n".join(lines) + "\n"


def main(argv: list[str]) -> int:
    out = Path(argv[1]) if len(argv) > 1 else Path("output/niigo_platform.dtb")
    out.parent.mkdir(parents=True, exist_ok=True)
    blob = build_dtb()
    out.write_bytes(blob)
    print(f"wrote {len(blob)} bytes -> {out}")
    # Also emit an assembly .byte include so a directed test can embed the blob
    # without depending on .incbin path resolution.
    inc = out.with_suffix(".inc")
    inc.write_text(emit_asm(blob))
    print(f"wrote asm include -> {inc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
