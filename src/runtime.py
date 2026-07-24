"""Small dependency-free runtime measurement helpers."""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import wintypes

import numpy as np


def process_rss_bytes() -> int | None:
    """Return the current process resident memory/working set in bytes.

    Windows reports the process working set, which is its resident physical memory.
    Linux reports VmRSS.  Returning ``None`` is explicit rather than fabricating a
    value on an unsupported platform.
    """
    if sys.platform == "win32":
        class ProcessMemoryCounters(ctypes.Structure):
            _fields_ = [
                ("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t),
            ]
        counters = ProcessMemoryCounters()
        counters.cb = ctypes.sizeof(counters)
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(ProcessMemoryCounters), wintypes.DWORD]
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        process = kernel32.GetCurrentProcess()
        ok = psapi.GetProcessMemoryInfo(process, ctypes.byref(counters), counters.cb)
        return int(counters.WorkingSetSize) if ok else None
    if sys.platform.startswith("linux"):
        for line in open("/proc/self/status", encoding="utf-8"):
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    return None


def latency_stats(values: list[float]) -> dict[str, float]:
    """Return mean, median, p95 and reciprocal mean throughput for timings."""
    samples = np.asarray(values, dtype=float)
    if len(samples) == 0:
        raise ValueError("At least one timing sample is required")
    mean = float(np.mean(samples))
    return {
        "samples": int(len(samples)),
        "mean_s": mean,
        "median_s": float(np.median(samples)),
        "p95_s": float(np.percentile(samples, 95)),
        "fps_from_mean": float(1.0 / mean) if mean > 0 else float("inf"),
    }
