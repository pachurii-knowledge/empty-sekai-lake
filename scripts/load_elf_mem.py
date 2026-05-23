#!/usr/bin/env python3
"""Load a RISC-V ELF into niigo-lake Verilator memory image files."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

SEGMENTS = (
    (0x0040_0000, 512 * 1024, "mem.text.bin"),
    (0x1000_0000, 512 * 1024, "mem.data.bin"),
    (0x8000_0000, 512 * 1024, "mem.ktext.bin"),
    (0x9000_0000, 512 * 1024, "mem.kdata.bin"),
)

# auipc t0, 0x7fc00; jr t0  -> jumps from 0x00400000 to 0x80000000
BOOTSTRAP = bytes.fromhex("9702c07f 67800200")


def parse_program_headers(elf: Path) -> list[tuple[int, int, bytes]]:
    output = subprocess.check_output(
        ["readelf", "-l", "-W", str(elf)],
        text=True,
        errors="replace",
    )
    segments: list[tuple[int, int, bytes]] = []
    in_load = False
    for line in output.splitlines():
        if line.startswith("Program Headers:"):
            in_load = True
            continue
        if not in_load:
            continue
        if not line.startswith("  LOAD"):
            if line.startswith(" Section to Segment") or line.startswith("Key to Flags"):
                break
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        offset = int(parts[1], 16)
        vaddr = int(parts[2], 16)
        filesz = int(parts[4], 16)
        data = elf.read_bytes()[offset : offset + filesz]
        segments.append((vaddr, filesz, data))
    if not segments:
        raise RuntimeError(f"no LOAD segments found in {elf}")
    return segments


def locate_segment(vaddr: int) -> tuple[int, str] | None:
    for base, size, name in SEGMENTS:
        if base <= vaddr < base + size:
            return base, name
    return None


def load_elf(elf: Path, output_dir: Path, *, bootstrap: bool) -> None:
    images: dict[str, bytearray] = {name: bytearray(size) for _, size, name in SEGMENTS}

    if bootstrap:
        images["mem.text.bin"][0 : len(BOOTSTRAP)] = BOOTSTRAP
    for vaddr, _filesz, data in parse_program_headers(elf):
        end = vaddr + len(data)
        cursor = vaddr
        offset_in_data = 0
        while cursor < end:
            loc = locate_segment(cursor)
            if loc is None:
                raise RuntimeError(
                    f"ELF byte 0x{cursor:08x} is outside the niigo-lake memory map"
                )
            base, name = loc
            seg_end = base + len(images[name])
            chunk_end = min(end, seg_end)
            chunk_len = chunk_end - cursor
            dst_off = cursor - base
            images[name][dst_off : dst_off + chunk_len] = data[
                offset_in_data : offset_in_data + chunk_len
            ]
            cursor += chunk_len
            offset_in_data += chunk_len

    output_dir.mkdir(parents=True, exist_ok=True)
    for _, _, name in SEGMENTS:
        path = output_dir / name
        path.write_bytes(images[name])


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("output/simulation"),
        help="Directory for mem.*.bin files",
    )
    parser.add_argument(
        "--no-bootstrap",
        action="store_true",
        help="Do not install the 0x00400000 -> 0x80000000 trampoline",
    )
    args = parser.parse_args(argv)
    load_elf(args.elf, args.output, bootstrap=not args.no_bootstrap)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
