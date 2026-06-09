#!/usr/bin/env python3
"""Stage a Linux-class boot image into the niigo-lake flat-image memory loader.

The simulator's `main_memory` reads an optional `mem.image.manifest` in the run
directory listing `<hex_byte_base> <file>` regions, and byte-loads each into the
sparse physical memory with no size cap (see src/main_memory.sv
initialize_flat_images). This is the path for blobs too big for the fixed 512 KiB
segment windows: the kernel Image, the DTB, and an initramfs cpio.

Two modes:

  real      Place named regions (kernel/dtb/initrd or arbitrary --region A:FILE)
            and write the manifest. Used to boot a real kernel.

  synthetic Emit a multi-MB blob filled with deterministic sentinel words at
            known offsets (plus the manifest), for the Phase H1 self-test
            (tests/big_image.S) that proves the loader + sparse DRAM hold a
            Linux-sized image. Prints the sentinel table it wrote.

Usage:
  load_linux_image.py real -o DIR [--kernel F@ADDR] [--dtb F@ADDR]
                                   [--initrd F@ADDR] [--region ADDR:F ...]
  load_linux_image.py synthetic -o DIR [--base ADDR] [--size BYTES]
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

MANIFEST_NAME = "mem.image.manifest"

# Deterministic sentinel scheme for the synthetic self-test. The probe word at
# byte offset `off` within the blob holds SENTINEL_BASE | index. tests/big_image.S
# hard-codes the same (base+offset, magic) pairs.
SENTINEL_BASE = 0xC0DE_0000
# Offsets (in bytes from the blob base) at which sentinels are written. Chosen to
# span several MB and to land on/after the 512 KiB segment-window boundary so the
# test exercises addresses the fixed windows could never reach.
SENTINEL_OFFSETS = (
    0x000_0000,   # 0 MiB   (blob base)
    0x008_0000,   # 512 KiB (exactly at the old window edge)
    0x010_0000,   # 1 MiB
    0x040_0000,   # 4 MiB
    0x080_0000,   # 8 MiB
    0x0C0_0000,   # 12 MiB
)


def _write_manifest(out_dir: Path, regions: list[tuple[int, str]]) -> None:
    lines = [f"{base:08x} {name}\n" for base, name in regions]
    (out_dir / MANIFEST_NAME).write_text("".join(lines))


def _parse_at(spec: str, sep: str) -> tuple[Path, int]:
    """Parse FILE@ADDR (sep='@') or ADDR:FILE (sep=':')."""
    if sep not in spec:
        raise argparse.ArgumentTypeError(f"expected FILE{sep}ADDR, got {spec!r}")
    if sep == "@":
        f, a = spec.rsplit("@", 1)
        return Path(f), int(a, 0)
    a, f = spec.split(":", 1)
    return Path(f), int(a, 0)


def do_real(args: argparse.Namespace) -> int:
    out_dir: Path = args.output
    out_dir.mkdir(parents=True, exist_ok=True)
    regions: list[tuple[int, str]] = []

    def stage(src: Path, base: int) -> None:
        data = src.read_bytes()
        name = f"mem.img_{base:08x}.bin"
        (out_dir / name).write_bytes(data)
        regions.append((base, name))
        print(f"  staged {src} -> {name} @ 0x{base:08x} ({len(data)} bytes)")

    if args.kernel:
        f, a = _parse_at(args.kernel, "@")
        stage(f, a)
    if args.dtb:
        f, a = _parse_at(args.dtb, "@")
        stage(f, a)
    if args.initrd:
        f, a = _parse_at(args.initrd, "@")
        stage(f, a)
    for r in args.region or []:
        f, a = _parse_at(r, ":")
        stage(f, a)

    if not regions:
        print("error: no regions specified", file=sys.stderr)
        return 2
    _write_manifest(out_dir, regions)
    print(f"wrote manifest -> {out_dir / MANIFEST_NAME}")
    return 0


def do_synthetic(args: argparse.Namespace) -> int:
    out_dir: Path = args.output
    out_dir.mkdir(parents=True, exist_ok=True)
    base = args.base
    size = args.size

    blob = bytearray(size)
    print(f"synthetic blob: base=0x{base:08x} size={size} (0x{size:x}) bytes")
    print("  sentinels (addr -> magic):")
    for i, off in enumerate(SENTINEL_OFFSETS):
        if off + 4 > size:
            continue
        magic = SENTINEL_BASE | i
        struct.pack_into("<I", blob, off, magic)
        print(f"    0x{base + off:08x} -> 0x{magic:08x}")

    name = "mem.synthimg.bin"
    (out_dir / name).write_bytes(blob)
    _write_manifest(out_dir, [(base, name)])
    print(f"wrote {name} + manifest -> {out_dir}")
    return 0


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    pr = sub.add_parser("real", help="stage kernel/dtb/initrd regions")
    pr.add_argument("-o", "--output", type=Path, required=True)
    pr.add_argument("--kernel", help="kernel image as FILE@ADDR")
    pr.add_argument("--dtb", help="device tree blob as FILE@ADDR")
    pr.add_argument("--initrd", help="initramfs cpio as FILE@ADDR")
    pr.add_argument("--region", action="append", help="extra region ADDR:FILE")
    pr.set_defaults(func=do_real)

    ps = sub.add_parser("synthetic", help="emit sentinel blob for the H1 self-test")
    ps.add_argument("-o", "--output", type=Path, required=True)
    ps.add_argument("--base", type=lambda s: int(s, 0), default=0x8000_0000)
    ps.add_argument("--size", type=lambda s: int(s, 0), default=16 * 1024 * 1024)
    ps.set_defaults(func=do_synthetic)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
