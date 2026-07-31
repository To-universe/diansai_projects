#!/usr/bin/env python3
import argparse
import csv
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path


VRMS_RE = re.compile(r"^\s*Vrms\s*:\s*([-+]?\d+(?:\.\d+)?)\s*V\s*$", re.I)
VPP_RE = re.compile(r"^\s*Vpp\s*:\s*([-+]?\d+(?:\.\d+)?)\s*V\s*$", re.I)
F0_RE = re.compile(r"^\s*Fundamental\s*:\s*([-+]?\d+(?:\.\d+)?)\s*Hz\s*$", re.I)
HARMONIC_RE = re.compile(
    r"^\s*H(\d+)\s*:\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*Hz\s+"
    r"([-+]?\d+(?:\.\d+)?)\s*V\s+"
    r"\(valid\s+(\d+)/(\d+)\)\s*$",
    re.I,
)


def log_frequency_points(f_min, f_max, count, round_hz):
    if count <= 0:
        raise ValueError("count must be positive")
    if f_min <= 0 or f_max <= 0 or f_max < f_min:
        raise ValueError("invalid frequency range")

    if count == 1:
        values = [f_min]
    else:
        ratio = (f_max / f_min) ** (1.0 / (count - 1))
        values = [f_min * (ratio ** i) for i in range(count)]

    if round_hz > 0:
        values = [round(v / round_hz) * round_hz for v in values]
        values[0] = round(f_min / round_hz) * round_hz
        values[-1] = round(f_max / round_hz) * round_hz

    result = []
    for value in values:
        value = float(value)
        if result and value <= result[-1]:
            step = round_hz if round_hz > 0 else 1.0
            value = result[-1] + step
        result.append(value)
    return result


def dense_high_frequency_points(f_min, split_hz, f_max, low_count, high_count, round_hz):
    if not (f_min < split_hz < f_max):
        raise ValueError("split_hz must be between f_min and f_max")
    low = log_frequency_points(f_min, split_hz, low_count, round_hz)
    high = log_frequency_points(split_hz, f_max, high_count, round_hz)
    return low + high[1:]


def decode_line(raw):
    return raw.decode("ascii", errors="replace").rstrip("\r\n")


def read_measurement(serial_port, timeout, idle_timeout, echo):
    deadline = time.monotonic() + timeout
    last_rx = time.monotonic()
    in_result = False
    in_waveform = False
    seen_waveform_data = False
    lines = []
    harmonics = {}
    values = {
        "vrms_v": None,
        "vpp_v": None,
        "fundamental_hz": None,
    }

    old_timeout = serial_port.timeout
    serial_port.timeout = min(0.2, max(0.01, idle_timeout))
    try:
        while time.monotonic() < deadline:
            raw = serial_port.readline()
            if not raw:
                if in_result and time.monotonic() - last_rx >= idle_timeout:
                    break
                continue

            last_rx = time.monotonic()
            line = decode_line(raw)
            if echo:
                print(f"  {line}")

            if "=== Results ===" in line:
                in_result = True

            if not in_result:
                continue

            lines.append(line)

            match = VRMS_RE.match(line)
            if match:
                values["vrms_v"] = float(match.group(1))
                continue

            match = VPP_RE.match(line)
            if match:
                values["vpp_v"] = float(match.group(1))
                continue

            match = F0_RE.match(line)
            if match:
                values["fundamental_hz"] = float(match.group(1))
                continue

            match = HARMONIC_RE.match(line)
            if match:
                order = int(match.group(1))
                harmonics[order] = {
                    "frequency_hz": float(match.group(2)),
                    "amplitude_v": float(match.group(3)),
                    "valid": int(match.group(4)),
                    "total": int(match.group(5)),
                }
                continue

            if line.strip().startswith("Waveform"):
                in_waveform = True
            elif in_waveform and line.strip():
                seen_waveform_data = True
            elif in_waveform and seen_waveform_data and not line.strip():
                break
    finally:
        serial_port.timeout = old_timeout

    if values["vpp_v"] is None:
        text = "\n".join(lines[-20:]) if lines else "<no result block>"
        raise TimeoutError(f"did not receive a complete result block before timeout. Last lines:\n{text}")

    values["harmonics"] = harmonics
    values["raw_text"] = "\n".join(lines)
    return values


def send_mode_command(serial_port, command, expected_marker, timeout, echo):
    if not command:
        return

    payload = (command.strip() + "\n").encode("ascii")
    serial_port.write(payload)
    serial_port.flush()

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = serial_port.readline()
        if not raw:
            continue

        line = decode_line(raw)
        if echo:
            print(f"  {line}")
        if expected_marker in line:
            return

    raise TimeoutError(f"did not receive {expected_marker!r} after sending {command!r}")


def format_frequency(freq_hz):
    if abs(freq_hz - round(freq_hz)) < 1e-6:
        return f"{int(round(freq_hz))} Hz"
    return f"{freq_hz:.3f} Hz"


def write_header(writer):
    writer.writerow(
        [
            "timestamp",
            "point_index",
            "target_frequency_hz",
            "nominal_vpp_v",
            "measured_vpp_v",
            "measured_vrms_v",
            "measured_f0_hz",
            "vpp_measured_over_nominal",
            "vpp_error_v",
            "vpp_error_percent",
            "vpp_gain_correction",
            "h1_amp_v",
            "harmonics",
        ]
    )


def write_c_table(path, rows, nominal_vpp):
    path = Path(path)
    lines = [
        "/* Copy these calibration constants into the MCU project. */",
        f"#define VPP_CAL_POINT_COUNT {len(rows)}U",
        f"#define VPP_CAL_NOMINAL_VPP {nominal_vpp:.9g}f",
        "",
        "static const float vpp_cal_freq_hz[VPP_CAL_POINT_COUNT] = {",
    ]

    for row in rows:
        lines.append(f"    {row['target_frequency_hz']:.6f}f,")
    lines.extend(
        [
            "};",
            "",
            "static const float vpp_cal_log_freq[VPP_CAL_POINT_COUNT] = {",
        ]
    )

    for row in rows:
        lines.append(f"    {math.log(row['target_frequency_hz']):.9f}f,")
    lines.extend(
        [
            "};",
            "",
            "static const float vpp_gain_correction[VPP_CAL_POINT_COUNT] = {",
        ]
    )

    for row in rows:
        lines.append(f"    {row['vpp_gain_correction']:.9f}f,")
    lines.extend(
        [
            "};",
            "",
            "static const float vpp_measured_over_nominal[VPP_CAL_POINT_COUNT] = {",
        ]
    )

    for row in rows:
        lines.append(f"    {row['vpp_measured_over_nominal']:.9f}f,")
    lines.extend(["};", ""])

    path.write_text("\n".join(lines), encoding="ascii")


def main():
    parser = argparse.ArgumentParser(
        description="Trigger STM32 measurements over UART and save Vpp calibration points to CSV."
    )
    parser.add_argument("--port", default="COM20", help="serial port, default: COM20")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate, default: 115200")
    parser.add_argument("--output", default="calibration_vpp.csv", help="CSV output path")
    parser.add_argument("--c-output", default="calibration_vpp_arrays.txt", help="copyable C array output path")
    parser.add_argument("--points", type=int, default=None, help="use one logarithmic sweep with this many points")
    parser.add_argument("--f-min", type=float, default=10_000.0, help="minimum frequency in Hz")
    parser.add_argument("--f-max", type=float, default=500_000.0, help="maximum frequency in Hz")
    parser.add_argument("--split-hz", type=float, default=100_000.0, help="frequency where the denser high band starts")
    parser.add_argument("--low-points", type=int, default=10, help="log points from f-min to split-hz")
    parser.add_argument("--high-points", type=int, default=20, help="log points from split-hz to f-max")
    parser.add_argument("--round-hz", type=float, default=500.0, help="round frequencies to this grid; 0 disables")
    parser.add_argument("--nominal-vpp", type=float, default=0.2, help="calibration sine Vpp in volts")
    parser.add_argument("--command", default="M", help="measurement command sent to STM32")
    parser.add_argument("--mode-command", default="C", help="command sent once before calibration starts")
    parser.add_argument("--mode-timeout", type=float, default=60.0, help="timeout while entering calibration mode")
    parser.add_argument("--skip-mode-command", action="store_true", help="do not send the calibration mode command")
    parser.add_argument("--timeout", type=float, default=45.0, help="timeout for one measurement result")
    parser.add_argument("--idle-timeout", type=float, default=0.6, help="end result after this much serial idle time")
    parser.add_argument("--boot-delay", type=float, default=1.5, help="delay after opening the port")
    parser.add_argument("--settle-delay", type=float, default=0.2, help="delay after user confirms the signal is set")
    parser.add_argument("--auto", action="store_true", help="do not wait for Enter before each frequency")
    parser.add_argument("--keep-buffer", action="store_true", help="do not clear serial input before each command")
    parser.add_argument("--echo", action="store_true", help="print received STM32 lines")
    parser.add_argument("--raw-dir", default=None, help="optional directory for raw result text")
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is not installed. Install it with: python -m pip install pyserial") from exc

    if args.points is None:
        frequencies = dense_high_frequency_points(
            args.f_min,
            args.split_hz,
            args.f_max,
            args.low_points,
            args.high_points,
            args.round_hz,
        )
    else:
        frequencies = log_frequency_points(args.f_min, args.f_max, args.points, args.round_hz)
    output_path = Path(args.output)
    c_output_path = Path(args.c_output)
    raw_dir = Path(args.raw_dir) if args.raw_dir else None
    if raw_dir:
        raw_dir.mkdir(parents=True, exist_ok=True)

    print("Calibration frequencies:")
    for index, freq in enumerate(frequencies, start=1):
        print(f"  {index:02d}. {format_frequency(freq)}")
    print(f"Nominal input: {args.nominal_vpp:.6f} Vpp single-frequency sine")
    print()

    command = (args.command.strip() + "\n").encode("ascii")

    with serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1, timeout=0.2) as ser:
        time.sleep(args.boot_delay)
        ser.reset_input_buffer()

        if not args.skip_mode_command:
            print(f"Entering calibration mode with {args.mode_command!r}...")
            send_mode_command(ser, args.mode_command, "[mode] CALIBRATION", args.mode_timeout, args.echo)
            ser.reset_input_buffer()

        with output_path.open("w", newline="", encoding="ascii") as f:
            writer = csv.writer(f)
            write_header(writer)
            f.flush()
            c_rows = []

            for index, freq in enumerate(frequencies, start=1):
                print(f"[{index}/{len(frequencies)}] Set signal generator to {format_frequency(freq)}, "
                      f"{args.nominal_vpp:.6f} Vpp.")
                if not args.auto:
                    input("Press Enter after the waveform is stable...")
                if args.settle_delay > 0:
                    time.sleep(args.settle_delay)

                if not args.keep_buffer:
                    ser.reset_input_buffer()
                ser.write(command)
                ser.flush()

                started = time.perf_counter()
                result = read_measurement(ser, args.timeout, args.idle_timeout, args.echo)
                elapsed = time.perf_counter() - started

                measured_vpp = result["vpp_v"]
                vpp_error = measured_vpp - args.nominal_vpp
                measured_over_nominal = measured_vpp / args.nominal_vpp if args.nominal_vpp != 0 else math.nan
                vpp_error_percent = vpp_error / args.nominal_vpp * 100.0 if args.nominal_vpp != 0 else math.nan
                gain_correction = args.nominal_vpp / measured_vpp if measured_vpp != 0 else math.nan
                h1_amp = result["harmonics"].get(1, {}).get("amplitude_v", math.nan)
                harmonic_summary = ";".join(
                    f"H{order}:{info['frequency_hz']:.2f}Hz,{info['amplitude_v']:.6f}V,{info['valid']}/{info['total']}"
                    for order, info in sorted(result["harmonics"].items())
                )

                writer.writerow(
                    [
                        datetime.now().isoformat(timespec="seconds"),
                        index,
                        f"{freq:.6f}",
                        f"{args.nominal_vpp:.9f}",
                        f"{measured_vpp:.9f}",
                        "" if result["vrms_v"] is None else f"{result['vrms_v']:.9f}",
                        "" if result["fundamental_hz"] is None else f"{result['fundamental_hz']:.6f}",
                        f"{measured_over_nominal:.9f}",
                        f"{vpp_error:.9f}",
                        f"{vpp_error_percent:.6f}",
                        f"{gain_correction:.9f}",
                        "" if math.isnan(h1_amp) else f"{h1_amp:.9f}",
                        harmonic_summary,
                    ]
                )
                f.flush()

                c_rows.append(
                    {
                        "target_frequency_hz": freq,
                        "vpp_gain_correction": gain_correction,
                        "vpp_measured_over_nominal": measured_over_nominal,
                    }
                )
                write_c_table(c_output_path, c_rows, args.nominal_vpp)

                if raw_dir:
                    raw_path = raw_dir / f"point_{index:02d}_{int(round(freq))}Hz.txt"
                    raw_path.write_text(result["raw_text"], encoding="ascii")

                print(
                    f"  Vpp={measured_vpp:.6f} V, "
                    f"Vrms={result['vrms_v'] if result['vrms_v'] is not None else math.nan:.6f} V, "
                    f"f0={result['fundamental_hz'] if result['fundamental_hz'] is not None else math.nan:.2f} Hz, "
                    f"measured/nominal={measured_over_nominal:.6f}, "
                    f"gain_correction={gain_correction:.6f}, elapsed={elapsed:.2f}s"
                )

    print(f"\nSaved calibration CSV: {output_path}")
    print(f"Saved C table: {c_output_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
