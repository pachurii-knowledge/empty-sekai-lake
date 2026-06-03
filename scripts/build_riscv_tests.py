#!/usr/bin/env python3
"""Build ACT ELFs for niigo-lake, bypassing the act Gemfile bundle install."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RISCV_TESTS = ROOT / "references" / "riscv-tests"


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="config/cores/niigo-lake/niigo-rv32g/test_config.yaml",
        help="ACT test_config.yaml path relative to references/riscv-tests",
    )
    parser.add_argument("--extensions", default="I", help="Comma-separated extension filter")
    parser.add_argument(
        "--exclude",
        default="Sm,PMPSm",
        help="Comma-separated extensions to exclude (set empty to build the "
        "privileged/Sv32 suites)",
    )
    parser.add_argument("--jobs", type=int, default=24)
    parser.add_argument("--fast", action="store_true", default=True)
    args = parser.parse_args(argv)

    sys.path.insert(0, str(RISCV_TESTS / "framework" / "src"))
    import act.parse_udb_config as parse_udb_config
    from act import act as act_module

    parse_udb_config._ensure_udb_installed = lambda: None  # noqa: SLF001

    def _run_udb(cmd: list[str], **kwargs: object):
        return subprocess.run(cmd, check=kwargs.get("check", False), **{  # type: ignore[arg-type]
            k: v for k, v in kwargs.items() if k != "check"
        })

    parse_udb_config._bundle_exec = _run_udb  # noqa: SLF001

    os.chdir(RISCV_TESTS)
    try:
        act_module.run_act(
            [Path(args.config)],
            test_dir=Path("tests"),
            workdir=Path("work"),
            extensions=args.extensions,
            exclude=args.exclude,
            jobs=args.jobs,
            fast=args.fast,
            keep_going=True,
        )
    except SystemExit as exc:
        code = exc.code
        return int(code) if isinstance(code, int) else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
