#!/usr/bin/env python3
"""
STM32 Quadcopter Flight Log Analyzer
=====================================
Parses raw SD card sector dump and computes dynamic angle tracking metrics.

LogEntry_t layout (16 fields × 4 bytes = 64 bytes, 8 entries per 512-byte sector):

Offset  Field             Type       Description
------  ----------------  ---------  ---------------------------
 0~3    magic             uint32_t   0x4C4F4745 ("LOGE")
 4~7    seq               uint32_t   Sequence number
 8~11   timestamp_ms      uint32_t   System tick (ms)
12~15   roll              float      Actual roll angle (°)
16~19   pitch             float      Actual pitch angle (°)
20~23   yaw               float      Actual yaw angle (°)
24~27   gyro_x            float      Gyro X (°/s)
28~31   gyro_y            float      Gyro Y (°/s)
32~35   gyro_z            float      Gyro Z (°/s)
36~39   m1                float      Motor 1 (μs)
40~43   m2                float      Motor 2 (μs)
44~47   m3                float      Motor 3 (μs)
48~51   m4                float      Motor 4 (μs)
52~55   target_roll       float      Target roll angle (°)
56~59   target_pitch      float      Target pitch angle (°)
60~63   target_yaw_rate   float      Target yaw rate (°/s)

Usage:
    python log_analyzer.py sd_dump.bin [--axis roll|pitch] [--plot]
"""

import struct
import math
import sys
import argparse
from typing import Optional

# ── Constants ──────────────────────────────────────────────────
LOG_MAGIC       = 0x4C4F4745
LOG_ENTRY_SIZE  = 64
SECTOR_SIZE     = 512
ENTRIES_PER_SEC = SECTOR_SIZE // LOG_ENTRY_SIZE   # = 8
STRUCT_FMT      = '<III13f'                       # 3×uint32 + 13×float
STRUCT_SIZE     = struct.calcsize(STRUCT_FMT)      # = 64 bytes

# ── Parse raw sector dump ──────────────────────────────────────
def parse_log(bin_path: str) -> list:
    """
    Read raw SD card sector dump and return list of parsed log entries.

    Each log entry is a dict with fields matching LogEntry_t.
    Returns empty list if file not found or no valid entries.
    """
    entries: list = []
    invalid_count = 0

    with open(bin_path, 'rb') as f:
        sector_num = 0
        while True:
            chunk = f.read(SECTOR_SIZE)
            if not chunk:
                break
            if len(chunk) < SECTOR_SIZE:
                print(f"[WARN] Sector {sector_num}: partial read ({len(chunk)} bytes), skipping")
                break

            for i in range(ENTRIES_PER_SEC):
                off = i * LOG_ENTRY_SIZE
                raw = chunk[off : off + STRUCT_SIZE]
                if len(raw) < STRUCT_SIZE:
                    break

                magic = struct.unpack_from('<I', raw, 0)[0]
                if magic != LOG_MAGIC:
                    invalid_count += 1
                    continue   # Empty or corrupted entry

                vals = struct.unpack_from(STRUCT_FMT, raw, 0)
                entries.append({
                    'seq':            vals[1],
                    'ts_ms':          vals[2],
                    'roll':           vals[3],
                    'pitch':          vals[4],
                    'yaw':            vals[5],
                    'gyro_x':         vals[6],
                    'gyro_y':         vals[7],
                    'gyro_z':         vals[8],
                    'm1':             vals[9],
                    'm2':             vals[10],
                    'm3':             vals[11],
                    'm4':             vals[12],
                    'target_roll':     vals[13],
                    'target_pitch':    vals[14],
                    'target_yaw_rate': vals[15],
                })
            sector_num += 1

    print(f"Parsed {len(entries)} valid entries "
          f"(skipped {invalid_count} invalid/incomplete) "
          f"from {sector_num} sectors")
    return entries

# ── Filter: entries during active maneuver ─────────────────────
def filter_maneuver(entries: list, axis: str = 'roll',
                    threshold: float = 5.0) -> list:
    """
    Return entries where target deviation exceeds threshold,
    indicating the pilot is actively commanding a maneuver.
    """
    target_key = f'target_{axis}'
    actual_key = axis

    return [e for e in entries
            if abs(e.get(target_key, 0.0)) > threshold]

# ── Metrics ────────────────────────────────────────────────────
def compute_metrics(entries: list, axis: str = 'roll') -> dict:
    """
    Compute tracking error metrics for a list of log entries.

    Returns dict with keys:
        rms_error, max_deviation, mean_steady_error,
        overshoot_pct, settling_time_ms
    """
    target_key = f'target_{axis}'
    actual_key = axis

    if not entries:
        return {'rms_error': float('nan'), 'max_deviation': float('nan'),
                'mean_steady_error': float('nan'), 'overshoot_pct': float('nan'),
                'settling_time_ms': float('nan'), 'count': 0}

    errors = [abs(e[target_key] - e[actual_key]) for e in entries]

    # RMS error
    rms = math.sqrt(sum(e**2 for e in errors) / len(errors))

    # Max instantaneous deviation
    max_err = max(errors)

    # Steady-state error: average error during the last 2 seconds of maneuver
    # (rough: last 200 entries at 100Hz ≈ 2 sec)
    steady_window = min(200, len(errors))
    steady_entries = entries[-steady_window:]
    steady_errors = [abs(e[target_key] - e[actual_key]) for e in steady_entries]
    mean_steady = sum(steady_errors) / len(steady_errors)

    # Overshoot: max of (actual - target) when both have same sign
    overshoots = []
    for e in entries:
        tgt = e[target_key]
        act = e[actual_key]
        if abs(tgt) > 1.0:   # Only meaningful when target is non-trivial
            overshoot = act - tgt
            if (tgt > 0 and overshoot > 0) or (tgt < 0 and overshoot < 0):
                overshoots.append(abs(overshoot))
    overshoot_pct = (max(overshoots) / abs(entries[-1][target_key]) * 100) \
                    if overshoots and abs(entries[-1][target_key]) > 1.0 \
                    else 0.0

    return {
        'rms_error':          rms,
        'max_deviation':      max_err,
        'mean_steady_error':  mean_steady,
        'overshoot_pct':      overshoot_pct,
        'count':              len(entries),
    }

# ── Pass/fail judgment ─────────────────────────────────────────
# Criteria from test plan:
#   RMS error ≤ 2.0°  |  Max deviation ≤ 3.5°
#   Steady error ≤ 0.5°  |  Overshoot < 25%
PASS_CRITERIA = {
    'rms_error':          (float('-inf'), 2.0),
    'max_deviation':      (float('-inf'), 3.5),
    'mean_steady_error':  (float('-inf'), 0.5),
    'overshoot_pct':      (float('-inf'), 25.0),
}

def judge(metrics: dict) -> bool:
    """Return True if all criteria pass."""
    if metrics.get('count', 0) == 0:
        return False
    for key, (lo, hi) in PASS_CRITERIA.items():
        val = metrics.get(key, float('nan'))
        if math.isnan(val):
            continue
        if not (lo <= val <= hi):
            return False
    return True

# ── Print report ───────────────────────────────────────────────
def print_report(metrics: dict, axis: str = 'roll'):
    """Print formatted metrics report."""
    labels = {
        'rms_error':         'RMS dynamic error',
        'max_deviation':     'Max instantaneous deviation',
        'mean_steady_error': 'Mean steady-state error',
        'overshoot_pct':     'Overshoot',
    }
    units = {
        'rms_error': '°', 'max_deviation': '°',
        'mean_steady_error': '°', 'overshoot_pct': '%',
    }
    passed = judge(metrics)

    print(f"\n{'='*60}")
    print(f"  Test Report — {axis.upper()} Axis")
    print(f"  Sample count: {metrics.get('count', 0)}")
    print(f"{'='*60}")
    for key, label in labels.items():
        val = metrics.get(key, float('nan'))
        if not math.isnan(val):
            lo, hi = PASS_CRITERIA[key]
            status = '[PASS]' if lo <= val <= hi else '[FAIL]'
            print(f"  {label:30s}: {val:7.2f} {units[key]:3s}  "
                  f"(target: ≤{hi:.1f}{units[key]}) {status}")

    print(f"{'='*60}")
    print(f"  OVERALL: {'PASS' if passed else 'FAIL — needs tuning'}")
    print(f"{'='*60}\n")

# ── Plot (optional) ────────────────────────────────────────────
def plot_tracking(entries: list, axis: str = 'roll'):
    """Plot target vs actual tracking over time."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[INFO] matplotlib not installed, skipping plot")
        return

    target_key = f'target_{axis}'
    actual_key = axis

    ts   = [e['ts_ms'] / 1000.0 for e in entries]   # seconds
    tgt  = [e[target_key] for e in entries]
    act  = [e[actual_key] for e in entries]
    err  = [abs(e[target_key] - e[actual_key]) for e in entries]

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 8))

    ax1.plot(ts, tgt, 'b--', label='Target', linewidth=1.5)
    ax1.plot(ts, act, 'r-',  label='Actual', linewidth=1.0)
    ax1.set_ylabel(f'{axis.capitalize()} Angle (°)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_title(f'{axis.capitalize()} Axis Tracking Performance')

    ax2.plot(ts, err, 'k-', linewidth=0.8)
    ax2.axhline(y=2.0, color='orange', linestyle='--', label='±2° threshold')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Absolute Error (°)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

# ── CLI entry point ────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='STM32 Quadcopter Flight Log Analyzer')
    parser.add_argument('bin_path', help='Path to raw SD card sector dump (.bin)')
    parser.add_argument('--axis', choices=['roll', 'pitch', 'yaw'],
                        default='roll', help='Axis to analyze (default: roll)')
    parser.add_argument('--plot', action='store_true',
                        help='Show tracking plot (requires matplotlib)')
    parser.add_argument('--all', action='store_true',
                        help='Report all axes (roll, pitch, yaw)')
    args = parser.parse_args()

    # Parse log
    entries = parse_log(args.bin_path)
    if not entries:
        print("[ERROR] No valid log entries found.")
        sys.exit(1)

    axes = ['roll', 'pitch', 'yaw'] if args.all else [args.axis]

    for axis in axes:
        maneuver = filter_maneuver(entries, axis)
        if not maneuver:
            print(f"[WARN] No maneuver data for {axis} axis "
                  f"(target|{axis}| always < 5°)")
            continue

        metrics = compute_metrics(maneuver, axis)
        print_report(metrics, axis)

        if args.plot and axis == args.axis:
            plot_tracking(maneuver, axis)


if __name__ == '__main__':
    main()
