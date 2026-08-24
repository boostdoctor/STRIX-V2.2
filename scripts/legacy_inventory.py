#!/usr/bin/env python3
"""Print a compact inventory of legacy/ (or a path)."""
from __future__ import annotations
import argparse
import os
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("root", nargs="?", default="legacy", help="legacy root (default: legacy)")
    args = ap.parse_args()
    root = Path(args.root)
    if not root.is_dir():
        raise SystemExit(f"not a directory: {root}")

    print(f"Legacy inventory: {root.resolve()}\n")
    total_files = 0
    total_bytes = 0
    for child in sorted(root.iterdir()):
        if not child.is_dir():
            continue
        files = [p for p in child.rglob("*") if p.is_file()]
        nbytes = sum(p.stat().st_size for p in files)
        total_files += len(files)
        total_bytes += nbytes
        print(f"  {child.name:28s}  {len(files):5d} files  {nbytes/1024:8.1f} KiB")
    print(f"\n  {'TOTAL':28s}  {total_files:5d} files  {total_bytes/1024:8.1f} KiB")


if __name__ == "__main__":
    main()
