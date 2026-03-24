"""Capture raw BNO055 magnetometer samples for Magneto calibration."""

from __future__ import annotations

import argparse
import csv
import math
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import board
from adafruit_bno055 import BNO055_I2C


@dataclass
class SessionStats:
    sample_count: int = 0
    skipped_count: int = 0
    duplicate_count: int = 0
    magnitude_min: float = math.inf
    magnitude_max: float = 0.0
    magnitude_sum: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Log raw BNO055 magnetometer data to CSV for Magneto calibration."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent
        / f"magnetometer_samples_{time.strftime('%Y%m%d_%H%M%S')}.csv",
        help="Output CSV path.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help="Capture duration in seconds.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Sampling rate in Hz.",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="Warmup delay before capture starts (seconds).",
    )
    parser.add_argument(
        "--no-timestamp",
        action="store_true",
        help="Omit timestamp column.",
    )
    parser.add_argument(
        "--include-cal-status",
        action="store_true",
        help="Include sys/gyro/accel/mag calibration level columns.",
    )
    return parser.parse_args()


def build_headers(include_timestamp: bool, include_cal_status: bool) -> list[str]:
    headers = []
    if include_timestamp:
        headers.append("timestamp")
    headers.extend(["mx", "my", "mz"])
    if include_cal_status:
        headers.extend(["sys_cal", "gyro_cal", "accel_cal", "mag_cal"])
    return headers


def is_valid_sample(sample: object) -> bool:
    if not isinstance(sample, tuple) or len(sample) != 3:
        return False
    for val in sample:
        if val is None:
            return False
        try:
            value = float(val)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(value):
            return False
    return True


def format_row(
    sample: tuple[float, float, float],
    include_timestamp: bool,
    include_cal_status: bool,
    bno: BNO055_I2C,
) -> dict[str, float]:
    row: dict[str, float] = {
        "mx": float(sample[0]),
        "my": float(sample[1]),
        "mz": float(sample[2]),
    }
    if include_timestamp:
        row["timestamp"] = time.time()
    if include_cal_status:
        try:
            sys_cal, gyro_cal, accel_cal, mag_cal = bno.calibration_status
            row["sys_cal"] = int(sys_cal)
            row["gyro_cal"] = int(gyro_cal)
            row["accel_cal"] = int(accel_cal)
            row["mag_cal"] = int(mag_cal)
        except Exception:
            row["sys_cal"] = -1
            row["gyro_cal"] = -1
            row["accel_cal"] = -1
            row["mag_cal"] = -1
    return row


def update_stats(stats: SessionStats, sample: Iterable[float]) -> None:
    mx, my, mz = sample
    magnitude = math.sqrt(mx * mx + my * my + mz * mz)
    stats.magnitude_min = min(stats.magnitude_min, magnitude)
    stats.magnitude_max = max(stats.magnitude_max, magnitude)
    stats.magnitude_sum += magnitude


def print_summary(stats: SessionStats, duration: float, output_path: Path) -> None:
    print("\nCapture complete")
    print(f"  Output file: {output_path}")
    print(f"  Duration: {duration:.2f}s")
    print(f"  Samples written: {stats.sample_count}")
    print(f"  Invalid/Skipped samples: {stats.skipped_count}")

    if stats.sample_count > 0:
        mean_magnitude = stats.magnitude_sum / stats.sample_count
        duplicate_pct = (stats.duplicate_count / stats.sample_count) * 100.0
        print(f"  |B| min/mean/max: {stats.magnitude_min:.2f} / {mean_magnitude:.2f} / {stats.magnitude_max:.2f} uT")
        print(f"  Consecutive duplicate samples: {stats.duplicate_count} ({duplicate_pct:.1f}%)")

    if stats.sample_count < 200:
        print("  Warning: low sample count; rotate/tumble longer for better ellipsoid fitting.")
    if stats.sample_count > 0 and (stats.duplicate_count / stats.sample_count) > 0.5:
        print("  Warning: many duplicate samples; sensor motion may have been insufficient.")


def main() -> int:
    args = parse_args()
    include_timestamp = not args.no_timestamp

    if args.rate <= 0.0:
        raise ValueError("--rate must be greater than 0")
    if args.duration <= 0.0:
        raise ValueError("--duration must be greater than 0")
    if args.warmup < 0.0:
        raise ValueError("--warmup must be >= 0")

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("Initializing BNO055 over I2C...")
    try:
        i2c = board.I2C()
        bno = BNO055_I2C(i2c)
    except Exception as exc:
        print(f"Failed to initialize BNO055: {exc}")
        return 1

    if args.warmup > 0:
        print(f"Warmup: {args.warmup:.1f}s")
        time.sleep(args.warmup)

    headers = build_headers(include_timestamp, args.include_cal_status)
    stats = SessionStats()
    stop_requested = False

    def handle_signal(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)

    interval = 1.0 / args.rate
    start = time.perf_counter()
    next_tick = start
    last_print = start
    last_sample: tuple[float, float, float] | None = None

    print("Collecting magnetometer data. Move sensor through all orientations.")
    print(f"Target: {args.duration:.1f}s at {args.rate:.1f} Hz")

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=headers)
        writer.writeheader()

        while not stop_requested:
            now = time.perf_counter()
            elapsed = now - start
            if elapsed >= args.duration:
                break

            if now < next_tick:
                time.sleep(min(next_tick - now, 0.002))
                continue

            next_tick += interval

            raw_mag = bno.magnetic
            if not is_valid_sample(raw_mag):
                stats.skipped_count += 1
                continue

            mag = (float(raw_mag[0]), float(raw_mag[1]), float(raw_mag[2]))
            row = format_row(mag, include_timestamp, args.include_cal_status, bno)
            writer.writerow(row)
            csv_file.flush()

            stats.sample_count += 1
            update_stats(stats, mag)

            if last_sample is not None and mag == last_sample:
                stats.duplicate_count += 1
            last_sample = mag

            if (now - last_print) >= 1.0:
                print(f"  Progress: {elapsed:6.1f}s | samples: {stats.sample_count:6d} | skipped: {stats.skipped_count:4d}")
                last_print = now

    print_summary(stats, time.perf_counter() - start, output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
