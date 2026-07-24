"""Diagnose Windows DLL loading for a RoboStack rclpy installation."""

from __future__ import annotations

import argparse
import ctypes
import os
from pathlib import Path
import re
import subprocess


DLL_PATTERN = re.compile(r"DLL Name:\s*(\S+)", re.IGNORECASE)


def dependencies(path: Path, objdump: Path) -> list[str]:
    completed = subprocess.run(
        [str(objdump), "-p", str(path)], capture_output=True, text=True, errors="replace", check=False,
    )
    return DLL_PATTERN.findall(completed.stdout)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--objdump", type=Path, default=Path(r"C:\msys64\mingw64\bin\objdump.exe"))
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    roots = [prefix, prefix / "Library" / "bin", prefix / "DLLs", Path(os.environ["WINDIR"]) / "System32"]
    target = prefix / "Lib" / "site-packages" / "rclpy" / "_rclpy_pybind11.cp312-win_amd64.pyd"
    queue = [target]
    seen: set[Path] = set()
    ordered: list[Path] = []
    while queue:
        path = queue.pop(0)
        if path in seen:
            continue
        seen.add(path)
        ordered.append(path)
        for name in dependencies(path, args.objdump):
            found = next((root / name for root in roots if (root / name).exists()), None)
            if found is not None and prefix in found.parents:
                queue.append(found)
    os.environ["PATH"] = os.pathsep.join(str(root) for root in roots[:3]) + os.pathsep + os.environ["PATH"]
    handles = []
    for path in reversed(ordered):
        try:
            handles.append(ctypes.WinDLL(str(path)))
            print(f"OK   {path.name}")
        except OSError as error:
            print(f"FAIL {path.name}: winerror={getattr(error, 'winerror', None)} {error}")


if __name__ == "__main__":
    main()
