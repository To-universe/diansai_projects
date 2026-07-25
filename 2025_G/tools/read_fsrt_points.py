#!/usr/bin/env python3
"""Read current STM32 UART magnitude response data and plot/save it.

Current firmware protocol:
    b"FSRT" + one little-endian float32 magnitude value, repeated for each FFT bin.

The frequency axis is the FFT-bin axis:
    f[k] = k * fs / nfft
not the AD9851 sweep table index.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from serial.tools import list_ports


POINT_MAGIC = b"FSRT"
FRAME_MAGIC = b"FRSP"
ADC_MAGIC = b"ADCT"


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device:10s} {port.description}")


def read_exact(ser: serial.Serial, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            raise TimeoutError(f"Timed out while reading {size} bytes; got {len(data)} bytes.")
        data.extend(chunk)
    return bytes(data)


def wait_magic(ser: serial.Serial, magic: bytes, timeout_s: float) -> int:
    matched = 0
    scanned = 0
    started = time.monotonic()

    while matched < len(magic):
        if time.monotonic() - started > timeout_s:
            raise TimeoutError(
                f"Timed out waiting for {magic!r} after scanning {scanned} bytes."
            )

        b = ser.read(1)
        if not b:
            continue

        scanned += 1
        if b[0] == magic[matched]:
            matched += 1
        else:
            matched = 1 if b[0] == magic[0] else 0

    return scanned


def wait_any_magic(ser: serial.Serial, magics: tuple[bytes, ...], timeout_s: float) -> bytes:
    max_len = max(len(magic) for magic in magics)
    window = bytearray()
    scanned = 0
    started = time.monotonic()

    while True:
        if time.monotonic() - started > timeout_s:
            expected = ", ".join(repr(magic) for magic in magics)
            raise TimeoutError(
                f"Timed out waiting for one of {expected} after scanning {scanned} bytes."
            )

        b = ser.read(1)
        if not b:
            continue

        scanned += 1
        window += b
        if len(window) > max_len:
            del window[:-max_len]

        for magic in magics:
            if bytes(window[-len(magic) :]) == magic:
                return magic


def read_adc_payload_after_magic(ser: serial.Serial) -> np.ndarray:
    payload_size = struct.unpack("<I", read_exact(ser, 4))[0]
    if payload_size == 0 or payload_size % 2 != 0:
        raise ValueError(f"Bad ADCT payload size: {payload_size}")

    payload = read_exact(ser, payload_size)
    return np.frombuffer(payload, dtype="<u2").copy()


def read_point_protocol(
    ser: serial.Serial,
    count: int,
    header_timeout_s: float,
    first_value: float | None = None,
) -> np.ndarray:
    values = np.empty(count, dtype=np.float32)
    start = 0

    if first_value is not None:
        values[0] = first_value
        start = 1
        print(f"\rRead points: 1/{count}", end="", flush=True)

    for i in range(start, count):
        wait_magic(ser, POINT_MAGIC, header_timeout_s)
        values[i] = struct.unpack("<f", read_exact(ser, 4))[0]
        print(f"\rRead points: {i + 1}/{count}", end="", flush=True)

    print()
    return values


def read_frame_protocol(ser: serial.Serial, header_timeout_s: float) -> np.ndarray:
    wait_magic(ser, FRAME_MAGIC, header_timeout_s)
    return read_frame_payload_after_magic(ser)


def read_frame_payload_after_magic(ser: serial.Serial) -> np.ndarray:
    payload_size = struct.unpack("<I", read_exact(ser, 4))[0]
    if payload_size == 0 or payload_size % 4 != 0:
        raise ValueError(f"Bad FRSP payload size: {payload_size}")

    payload = read_exact(ser, payload_size)
    return np.frombuffer(payload, dtype="<f4").copy()


def read_auto_protocol(
    ser: serial.Serial,
    count: int,
    header_timeout_s: float,
) -> tuple[str, np.ndarray, np.ndarray | None]:
    adc_data = None

    while True:
        magic = wait_any_magic(ser, (ADC_MAGIC, POINT_MAGIC, FRAME_MAGIC), header_timeout_s)

        if magic == ADC_MAGIC:
            adc_data = read_adc_payload_after_magic(ser)
            print(f"Read ADC frame: {adc_data.size} packed samples.")
            continue

        if magic == FRAME_MAGIC:
            return "frame", read_frame_payload_after_magic(ser), adc_data

        first_value = struct.unpack("<f", read_exact(ser, 4))[0]
        return "points", read_point_protocol(ser, count, header_timeout_s, first_value), adc_data


def read_capture(
    ser: serial.Serial,
    protocol: str,
    count: int,
    header_timeout_s: float,
) -> tuple[str, np.ndarray, np.ndarray | None]:
    adc_data = None

    while True:
        if protocol == "points":
            magic = wait_any_magic(ser, (ADC_MAGIC, POINT_MAGIC), header_timeout_s)
        elif protocol == "frame":
            magic = wait_any_magic(ser, (ADC_MAGIC, FRAME_MAGIC), header_timeout_s)
        else:
            return read_auto_protocol(ser, count, header_timeout_s)

        if magic == ADC_MAGIC:
            adc_data = read_adc_payload_after_magic(ser)
            print(f"Read ADC frame: {adc_data.size} packed samples.")
            continue

        if magic == POINT_MAGIC:
            first_value = struct.unpack("<f", read_exact(ser, 4))[0]
            return "points", read_point_protocol(ser, count, header_timeout_s, first_value), adc_data

        return "frame", read_frame_payload_after_magic(ser), adc_data


def default_bin_range(fs: float, nfft: int, start_hz: float, stop_hz: float) -> tuple[int, int]:
    bin_hz = fs / nfft
    k0 = int(math.ceil(start_hz / bin_hz))
    k1 = int(math.floor(stop_hz / bin_hz))
    k0 = max(0, min(k0, nfft // 2))
    k1 = max(k0, min(k1, nfft // 2))
    return k0, k1


def make_frequency_axis(fs: float, nfft: int, k0: int, count: int) -> np.ndarray:
    return (np.arange(count, dtype=np.float64) + k0) * (fs / nfft)


def save_csv(path: Path, freqs: np.ndarray, mag_db: np.ndarray, k0: int) -> None:
    rows = np.column_stack((np.arange(mag_db.size) + k0, freqs, mag_db))
    np.savetxt(
        path,
        rows,
        delimiter=",",
        header="k,freq_hz,mag_db",
        comments="",
        fmt=["%d", "%.9g", "%.9g"],
    )
    print(f"Saved CSV: {path}")


def save_adc_csv(path: Path, packed: np.ndarray, fs: float) -> None:
    sample_index = np.arange(packed.size, dtype=np.uint32)
    time_s = sample_index.astype(np.float64) / fs
    adc_low = (packed & 0x00FF).astype(np.uint16)
    adc_high = ((packed >> 8) & 0x00FF).astype(np.uint16)
    low_norm = adc_low.astype(np.float64) / 256.0
    high_norm = adc_high.astype(np.float64) / 256.0

    rows = np.column_stack((sample_index, time_s, adc_low, adc_high, low_norm, high_norm))
    np.savetxt(
        path,
        rows,
        delimiter=",",
        header="n,time_s,adc_low,adc_high,adc_low_norm,adc_high_norm",
        comments="",
        fmt=["%d", "%.9g", "%d", "%d", "%.9g", "%.9g"],
    )
    print(f"Saved ADC CSV: {path}")


def split_adc_channels(packed: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    adc_low = (packed & 0x00FF).astype(np.float64) / 256.0
    adc_high = ((packed >> 8) & 0x00FF).astype(np.float64) / 256.0
    return adc_low, adc_high


def adc_magnitude_spectrum_db(samples: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    if samples.size < 2:
        raise ValueError("Need at least two ADC samples for FFT.")

    spectrum = np.fft.rfft(samples)
    mag = np.abs(spectrum)
    freqs = np.fft.rfftfreq(samples.size, d=1.0 / fs)
    mag_db = 20.0 * np.log10(mag + 1e-12)
    return freqs, mag_db


def adc_frequency_response_mag_db(
    packed: np.ndarray,
    fs: float,
    guard_ratio: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    x, y = split_adc_channels(packed)
    x_fft = np.fft.rfft(x)
    y_fft = np.fft.rfft(y)

    denom = np.abs(x_fft) ** 2
    guard = float(np.max(denom)) * guard_ratio
    valid = denom >= guard

    response = np.zeros_like(y_fft, dtype=np.complex128)
    response[valid] = y_fft[valid] * np.conj(x_fft[valid]) / denom[valid]

    mag_db = np.full(response.shape, -120.0, dtype=np.float64)
    mag_db[valid] = 20.0 * np.log10(np.abs(response[valid]) + 1e-12)
    freqs = np.fft.rfftfreq(packed.size, d=1.0 / fs)
    return freqs, mag_db


def plot_adc_time_and_spectrum(
    packed: np.ndarray,
    fs: float,
    output: Path | None,
    show: bool,
) -> None:
    adc_low, adc_high = split_adc_channels(packed)
    time_ms = np.arange(packed.size, dtype=np.float64) * (1000.0 / fs)
    freqs, low_mag_db = adc_magnitude_spectrum_db(adc_low, fs)
    _, high_mag_db = adc_magnitude_spectrum_db(adc_high, fs)

    fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)

    ax_time.plot(time_ms, adc_low, label="ADC low byte", linewidth=1.0)
    ax_time.plot(time_ms, adc_high, label="ADC high byte", linewidth=1.0, alpha=0.85)
    ax_time.set_title("ADC Time Domain")
    ax_time.set_xlabel("Time (ms)")
    ax_time.set_ylabel("Normalized code")
    ax_time.grid(True, alpha=0.3)
    ax_time.legend()

    ax_fft.plot(freqs, low_mag_db, label="ADC low byte", linewidth=1.0)
    ax_fft.plot(freqs, high_mag_db, label="ADC high byte", linewidth=1.0, alpha=0.85)
    ax_fft.set_title("ADC FFT Magnitude Spectrum")
    ax_fft.set_xlabel("Frequency (Hz)")
    ax_fft.set_ylabel("Magnitude (dB, raw FFT)")
    ax_fft.set_xlim(0.0, fs / 2.0)
    ax_fft.grid(True, alpha=0.3)
    ax_fft.legend()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved ADC plot: {output}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_response(freqs: np.ndarray, mag_db: np.ndarray, output: Path | None) -> None:
    fig, ax = plt.subplots(figsize=(9.5, 5.2))
    if freqs.size and freqs[0] <= 0.0:
        ax.plot(freqs, mag_db, linewidth=1.0)
    else:
        ax.semilogx(freqs, mag_db, marker="o", linewidth=1.2, markersize=4)
    ax.set_title("Frequency Response Magnitude")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved plot: {output}")
    else:
        plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read STM32 FSRT UART magnitude points and plot/save frequency response."
    )
    parser.add_argument("--list", action="store_true", help="List serial ports and exit.")
    parser.add_argument("--port", help="Serial port, for example COM5.")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate.")
    parser.add_argument("--fs", type=float, default=100_000.0, help="ADC sampling rate in Hz.")
    parser.add_argument("--nfft", type=int, default=2048, help="FFT size used by firmware.")
    parser.add_argument("--start", type=float, default=1000.0, help="Start frequency used for k0.")
    parser.add_argument("--stop", type=float, default=20000.0, help="Stop frequency used for k1.")
    parser.add_argument("--k0", type=int, help="Override first FFT bin index.")
    parser.add_argument("--k1", type=int, help="Override last FFT bin index.")
    parser.add_argument(
        "--protocol",
        choices=("auto", "points", "frame"),
        default="points",
        help="points = repeated FSRT+float32; frame = FRSP+length+float32 payload.",
    )
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial byte timeout in seconds.")
    parser.add_argument(
        "--header-timeout",
        type=float,
        default=30.0,
        help="Timeout while searching for a frame header.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("result_uart_mag.csv"),
        help="CSV output path. Use --no-csv to disable.",
    )
    parser.add_argument(
        "--adc-csv",
        type=Path,
        default=Path("result_uart_adc.csv"),
        help="ADC CSV output path when an ADCT frame is received.",
    )
    parser.add_argument("--no-csv", action="store_true", help="Do not save CSV.")
    parser.add_argument("--no-adc-csv", action="store_true", help="Do not save ADC CSV.")
    parser.add_argument("--plot", action="store_true", help="Show a plot after reading.")
    parser.add_argument("--save-plot", type=Path, help="Save plot image instead of only showing it.")
    parser.add_argument("--plot-adc", action="store_true", help="Show ADC time/spectrum plots.")
    parser.add_argument("--save-adc-plot", type=Path, help="Save ADC time/spectrum plot image.")
    parser.add_argument(
        "--response-from-adc",
        action="store_true",
        help="Compute the first frequency-response plot from the ADCT frame instead of FSRT points.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list:
        list_serial_ports()
        return 0

    if not args.port:
        print("Missing --port. Use --list to see available ports.", file=sys.stderr)
        return 2

    k0, k1 = default_bin_range(args.fs, args.nfft, args.start, args.stop)
    if args.k0 is not None:
        k0 = args.k0
    if args.k1 is not None:
        k1 = args.k1
    if k0 < 0 or k1 < k0 or k1 > args.nfft // 2:
        raise ValueError(f"Invalid bin range: k0={k0}, k1={k1}, N_BINS={args.nfft // 2}")

    expected_count = k1 - k0 + 1
    bin_hz = args.fs / args.nfft
    print(
        f"Expecting bins k={k0}..{k1} ({expected_count} points), "
        f"BIN_HZ={bin_hz:.6f} Hz."
    )

    with serial.Serial(args.port, baudrate=args.baud, timeout=args.timeout) as ser:
        ser.reset_input_buffer()
        print(f"Opened {args.port} @ {args.baud}.")
        protocol, mag_db, adc_data = read_capture(
            ser,
            args.protocol,
            expected_count,
            args.header_timeout,
        )

    freqs = make_frequency_axis(args.fs, args.nfft, k0, mag_db.size)
    print(f"Protocol: {protocol}")
    print(f"Received {mag_db.size} float32 values.")
    print(
        f"mag_db: min={float(np.min(mag_db)):.3f}, "
        f"max={float(np.max(mag_db)):.3f}, mean={float(np.mean(mag_db)):.3f}"
    )

    response_freqs = freqs
    response_mag_db = mag_db
    response_k0 = k0
    if adc_data is not None and args.response_from_adc:
        response_freqs, response_mag_db = adc_frequency_response_mag_db(adc_data, args.fs)
        response_k0 = 0
        print(
            "Computed full-bin frequency response from ADC frame: "
            f"{response_mag_db.size} bins, 0..{args.fs / 2.0:.3f} Hz."
        )

    if not args.no_csv:
        save_csv(args.csv, response_freqs, response_mag_db, response_k0)
    if adc_data is not None and not args.no_adc_csv:
        save_adc_csv(args.adc_csv, adc_data, args.fs)
    if adc_data is not None and (args.plot_adc or args.save_adc_plot):
        plot_adc_time_and_spectrum(
            adc_data,
            args.fs,
            args.save_adc_plot,
            show=args.plot_adc,
        )

    if args.plot or args.save_plot:
        plot_response(response_freqs, response_mag_db, args.save_plot)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
