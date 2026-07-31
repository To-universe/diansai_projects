#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


def read_rows(path):
    with Path(path).open("r", newline="", encoding="ascii") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"no rows found in {path}")
    return rows


def require_float(row, key):
    value = row.get(key, "")
    if value == "":
        raise ValueError(f"missing {key!r} in row {row}")
    return float(value)


def find_base_row(rows, base_frequency_hz, tolerance_hz):
    best = min(rows, key=lambda row: abs(require_float(row, "target_frequency_hz") - base_frequency_hz))
    error = abs(require_float(best, "target_frequency_hz") - base_frequency_hz)
    if error > tolerance_hz:
        raise SystemExit(
            f"no base row near {base_frequency_hz} Hz; closest is "
            f"{require_float(best, 'target_frequency_hz')} Hz"
        )
    return best


def write_normalized_csv(path, fieldnames, rows):
    with Path(path).open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_c_arrays(path, rows, base_frequency_hz, base_measured_vpp):
    lines = [
        "/* Copy these normalized Vpp response constants into the MCU project if needed. */",
        f"#define VPP_NORM_POINT_COUNT {len(rows)}U",
        f"#define VPP_NORM_BASE_FREQ_HZ {base_frequency_hz:.6f}f",
        f"#define VPP_NORM_BASE_MEASURED_VPP {base_measured_vpp:.9f}f",
        "",
        "static const float vpp_norm_freq_hz[VPP_NORM_POINT_COUNT] = {",
    ]

    for row in rows:
        lines.append(f"    {require_float(row, 'target_frequency_hz'):.6f}f,")

    lines.extend(["};", "", "static const float vpp_normalized_to_10k[VPP_NORM_POINT_COUNT] = {"])
    for row in rows:
        lines.append(f"    {require_float(row, 'vpp_normalized_to_10k'):.9f}f,")

    lines.extend(["};", "", "static const float vpp_norm_gain_correction[VPP_NORM_POINT_COUNT] = {"])
    for row in rows:
        normalized = require_float(row, "vpp_normalized_to_10k")
        correction = 1.0 / normalized if normalized != 0.0 else math.nan
        lines.append(f"    {correction:.9f}f,")

    lines.extend(["};", ""])
    Path(path).write_text("\n".join(lines), encoding="ascii")


def main():
    parser = argparse.ArgumentParser(
        description="Normalize measured Vpp response by the measured 10 kHz Vpp."
    )
    parser.add_argument("--input", default="calibration_vpp.csv", help="input calibration CSV")
    parser.add_argument("--output", default="calibration_vpp_normalized.csv", help="output CSV")
    parser.add_argument("--c-output", default="calibration_vpp_normalized_arrays.txt", help="copyable C arrays output")
    parser.add_argument("--base-frequency", type=float, default=10_000.0, help="normalization base frequency in Hz")
    parser.add_argument("--base-tolerance", type=float, default=250.0, help="allowed base frequency mismatch in Hz")
    args = parser.parse_args()

    rows = read_rows(args.input)
    base_row = find_base_row(rows, args.base_frequency, args.base_tolerance)
    base_freq = require_float(base_row, "target_frequency_hz")
    base_vpp = require_float(base_row, "measured_vpp_v")
    if base_vpp <= 0.0:
        raise SystemExit(f"invalid base measured_vpp_v: {base_vpp}")

    for row in rows:
        measured_vpp = require_float(row, "measured_vpp_v")
        normalized = measured_vpp / base_vpp
        row["vpp_normalized_to_10k"] = f"{normalized:.9f}"
        row["vpp_norm_gain_correction"] = f"{(1.0 / normalized):.9f}" if normalized != 0.0 else ""

    fieldnames = list(rows[0].keys())
    for key in ("vpp_normalized_to_10k", "vpp_norm_gain_correction"):
        if key not in fieldnames:
            fieldnames.append(key)

    write_normalized_csv(args.output, fieldnames, rows)
    write_c_arrays(args.c_output, rows, base_freq, base_vpp)

    print(f"Base: {base_freq:.6f} Hz, measured_vpp={base_vpp:.9f} V")
    print(f"Saved normalized CSV: {args.output}")
    print(f"Saved C arrays: {args.c_output}")


if __name__ == "__main__":
    main()
