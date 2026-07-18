#!/usr/bin/env python3
"""Read STM32 UART frequency-response frames and plot them.

Current firmware sends frequency response as:
    "XYRT" + 4 float32 values for each sweep point while sweeping
    "FSRT" + freq_response raw float32 payload

freq_response stores 2048 complex sweep points as re/im pairs.
XYRT stores one point as xr/xi/yr/yi.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from serial.tools import list_ports


ADC_BUFFER_SIZE = 1024
N_FFT = ADC_BUFFER_SIZE
N_BINS = N_FFT // 2
SWEEP_POINTS = 2048
FREQ_RESPONSE_FLOATS = SWEEP_POINTS * 2
FREQ_RESPONSE_BYTES = FREQ_RESPONSE_FLOATS * 4
XY_RECORD_FLOATS = 4
XY_RECORD_BYTES = XY_RECORD_FLOATS * 4
FS_HZ = 1_000_000.0
BIN_HZ = FS_HZ / N_FFT
FRAME_MAGIC = b"FSRT"
XY_FRAME_MAGIC = b"XYRT"
ADC_FRAME_MAGIC = b"ADCT"


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device:10s} {port.description}")


def dump_serial_bytes(port: str, baud: int, timeout: float, size: int) -> None:
    with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
        ser.reset_input_buffer()
        payload = read_exact(ser, size)

    print("Hex:")
    for offset in range(0, len(payload), 16):
        chunk = payload[offset : offset + 16]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{offset:04X}: {hex_part:<47}  {ascii_part}")


def read_exact(ser: serial.Serial, size: int) -> bytes:
    chunks: list[bytes] = []
    total = 0
    started = time.monotonic()

    while total < size:
        chunk = ser.read(size - total)
        if not chunk:
            elapsed = time.monotonic() - started
            raise TimeoutError(
                f"Timed out after {elapsed:.1f}s: got {total}/{size} bytes"
            )
        chunks.append(chunk)
        total += len(chunk)
        print(f"\rReading: {total}/{size} bytes", end="", flush=True)

    print()
    return b"".join(chunks)


def read_framed_payload(
    ser: serial.Serial,
    header_timeout: float,
    magic: bytes,
    fixed_payload_size: int | None = None,
) -> bytes:
    matched = 0
    scanned = 0
    started = time.monotonic()
    print(f"Waiting for frame header {magic!r}...")

    while matched < len(magic):
        if time.monotonic() - started > header_timeout:
            raise TimeoutError(
                f"Timed out while waiting for frame header after scanning {scanned} bytes. "
                "If scanned is nonzero, the MCU is probably sending the old raw protocol "
                "or a different serial stream."
            )
        b = ser.read(1)
        if not b:
            continue
        scanned += 1
        if b[0] == magic[matched]:
            matched += 1
        else:
            matched = 1 if b[0] == magic[0] else 0

    if fixed_payload_size is not None:
        payload_size = fixed_payload_size
    else:
        length_raw = read_exact(ser, 4)
        (payload_size,) = struct.unpack("<I", length_raw)
        if payload_size == 0 or payload_size % 4 != 0:
            raise ValueError(
                f"Unexpected payload size {payload_size}. Expected a nonzero 4-byte aligned payload."
            )

    return read_exact(ser, payload_size)


def read_next_known_payload(
    ser: serial.Serial,
    header_timeout: float,
    frame_sizes: dict[bytes, int],
) -> tuple[bytes, bytes]:
    max_magic_len = max(len(magic) for magic in frame_sizes)
    window = bytearray()
    scanned = 0
    started = time.monotonic()
    expected = ", ".join(repr(magic) for magic in frame_sizes)
    print(f"Waiting for any frame header: {expected}...")

    while True:
        if time.monotonic() - started > header_timeout:
            raise TimeoutError(
                f"Timed out while waiting for frame header after scanning {scanned} bytes. "
                f"Expected one of: {expected}."
            )
        b = ser.read(1)
        if not b:
            continue
        scanned += 1
        window += b
        if len(window) > max_magic_len:
            del window[:-max_magic_len]

        for magic, payload_size in frame_sizes.items():
            if bytes(window[-len(magic):]) == magic:
                print(f"Found frame header {magic!r}.")
                return magic, read_exact(ser, payload_size)


def wait_for_idle_gap(ser: serial.Serial, gap_s: float) -> None:
    """Drain bytes until the stream is idle for gap_s seconds.

    This is useful when the MCU sends the same raw frame in a loop with a delay
    between frames. After the idle gap, the next byte should be the next frame's
    first byte.
    """
    old_timeout = ser.timeout
    ser.timeout = gap_s
    drained = 0

    while True:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            break
        drained += len(chunk)

    ser.timeout = old_timeout
    if drained:
        print(f"Drained {drained} bytes before idle gap.")


def load_uart_frame(
    port: str,
    baud: int,
    timeout: float,
    reset_input: bool,
    sync_gap: float,
    raw: bool,
    header_timeout: float,
    frame: str,
) -> np.ndarray:
    with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
        if reset_input:
            ser.reset_input_buffer()
        if raw and sync_gap > 0.0:
            print(f"Waiting for an idle gap of {sync_gap:.3f}s before reading a frame...")
            wait_for_idle_gap(ser, sync_gap)
        print(f"Opened {port} @ {baud}.")
        if raw:
            print(f"Waiting for {FREQ_RESPONSE_BYTES} raw bytes...")
            payload = read_exact(ser, FREQ_RESPONSE_BYTES)
        else:
            if frame == "adc":
                payload = read_framed_payload(ser, header_timeout, ADC_FRAME_MAGIC)
            elif frame == "xy":
                payload = read_framed_payload(
                    ser,
                    header_timeout,
                    XY_FRAME_MAGIC,
                    fixed_payload_size=XY_RECORD_BYTES,
                )
            else:
                payload = read_framed_payload(
                    ser,
                    header_timeout,
                    FRAME_MAGIC,
                    fixed_payload_size=FREQ_RESPONSE_BYTES,
                )

    dtype = "<u2" if frame == "adc" else "<f4"
    return np.frombuffer(payload, dtype=dtype).copy()


def load_freq_xy_frames(
    port: str,
    baud: int,
    timeout: float,
    reset_input: bool,
    header_timeout: float,
) -> tuple[np.ndarray, np.ndarray]:
    with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
        if reset_input:
            ser.reset_input_buffer()
        print(f"Opened {port} @ {baud}.")
        xy_records = []
        freq_payload = None
        for idx in range(SWEEP_POINTS):
            magic, payload = read_next_known_payload(
                ser,
                header_timeout,
                {
                    XY_FRAME_MAGIC: XY_RECORD_BYTES,
                    FRAME_MAGIC: FREQ_RESPONSE_BYTES,
                },
            )
            if magic == FRAME_MAGIC:
                freq_payload = payload
                print(
                    "Received FSRT before all XYRT records. "
                    f"Captured {idx}/{SWEEP_POINTS} XYRT records; plotting available freq_response."
                )
                break

            xy_records.append(np.frombuffer(payload, dtype="<f4").copy())
            if (idx + 1) % 256 == 0 or idx + 1 == SWEEP_POINTS:
                print(f"Read XYRT records: {idx + 1}/{SWEEP_POINTS}")

        while freq_payload is None:
            magic, payload = read_next_known_payload(
                ser,
                header_timeout,
                {
                    XY_FRAME_MAGIC: XY_RECORD_BYTES,
                    FRAME_MAGIC: FREQ_RESPONSE_BYTES,
                },
            )
            if magic == FRAME_MAGIC:
                freq_payload = payload
            else:
                xy_records.append(np.frombuffer(payload, dtype="<f4").copy())
                print(f"Read XYRT records: {len(xy_records)}/{SWEEP_POINTS}")

    freq_data = np.frombuffer(freq_payload, dtype="<f4").copy()
    if xy_records:
        xy_data = np.concatenate(xy_records).astype(np.float32, copy=False)
    else:
        xy_data = np.empty(0, dtype=np.float32)
    return freq_data, xy_data


def load_both_frames(
    port: str,
    baud: int,
    timeout: float,
    reset_input: bool,
    header_timeout: float,
) -> tuple[np.ndarray, np.ndarray]:
    with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
        if reset_input:
            ser.reset_input_buffer()
        print(f"Opened {port} @ {baud}.")
        adc_payload = read_framed_payload(ser, header_timeout, ADC_FRAME_MAGIC)
        freq_payload = read_framed_payload(
            ser,
            header_timeout,
            FRAME_MAGIC,
            fixed_payload_size=FREQ_RESPONSE_BYTES,
        )

    adc_data = np.frombuffer(adc_payload, dtype="<u2").copy()
    freq_data = np.frombuffer(freq_payload, dtype="<f4").copy()
    return adc_data, freq_data


def make_frequency_axis(count: int, start_hz: float, stop_hz: float, x_axis: str) -> np.ndarray:
    if count <= 0:
        raise ValueError("No samples received.")

    if x_axis == "log-sweep":
        if start_hz <= 0.0 or stop_hz <= 0.0:
            raise ValueError("Log sweep axis requires positive start/stop frequencies.")
        return np.geomspace(start_hz, stop_hz, count, dtype=np.float32)

    k0 = int(np.ceil(start_hz / BIN_HZ))
    return (np.arange(count, dtype=np.float32) + k0) * BIN_HZ


def complex_sweep_points(data: np.ndarray) -> np.ndarray:
    usable = min((data.size // 2) * 2, SWEEP_POINTS * 2)
    if usable == 0:
        raise ValueError("No complex frequency-response samples received.")
    return data[:usable].reshape(-1, 2)


def complex_to_mag_db(data: np.ndarray) -> np.ndarray:
    points = complex_sweep_points(data)
    response = points[:, 0] + 1j * points[:, 1]
    return 20.0 * np.log10(np.abs(response) + 1e-12)


def xy_sweep_points(data: np.ndarray) -> np.ndarray:
    usable = min((data.size // 4) * 4, SWEEP_POINTS * 4)
    if usable == 0:
        raise ValueError("No X/Y frequency-domain samples received.")
    return data[:usable].reshape(-1, 4)


def xy_to_mag_db(data: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    points = xy_sweep_points(data)
    x = points[:, 0] + 1j * points[:, 1]
    y = points[:, 2] + 1j * points[:, 3]
    x_mag_db = 20.0 * np.log10(np.abs(x) + 1e-12)
    y_mag_db = 20.0 * np.log10(np.abs(y) + 1e-12)
    return x_mag_db, y_mag_db


def plot_mag_mode(
    data: np.ndarray,
    start_hz: float,
    stop_hz: float,
    x_axis: str,
    output: Path | None,
) -> None:
    mag_db = complex_to_mag_db(data) if data.size >= 2 * SWEEP_POINTS else data
    count = mag_db.size
    freqs = make_frequency_axis(count, start_hz, stop_hz, x_axis)
    print_data_stats(mag_db, "mag_db")

    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.semilogx(freqs, mag_db, linewidth=1.2)
    ax.set_title("Frequency Response")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(freqs[0], freqs[-1])
    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved plot: {output}")
    else:
        plt.show()


def plot_complex_mode(
    data: np.ndarray,
    start_hz: float,
    stop_hz: float,
    x_axis: str,
    output: Path | None,
) -> None:
    complex_bins = complex_sweep_points(data)
    response = complex_bins[:, 0] + 1j * complex_bins[:, 1]
    freqs = make_frequency_axis(complex_bins.shape[0], start_hz, stop_hz, x_axis)

    mag_db = 20.0 * np.log10(np.abs(response) + 1e-12)
    phase_deg = np.angle(response, deg=True)
    print_data_stats(complex_bins[:, 0], "real")
    print_data_stats(complex_bins[:, 1], "imag")
    print_data_stats(mag_db, "mag_db")

    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax_mag.plot(freqs, mag_db, linewidth=1.1)
    ax_mag.set_ylabel("Magnitude (dB)")
    ax_mag.grid(True, alpha=0.3)

    ax_phase.plot(freqs, phase_deg, linewidth=1.1)
    ax_phase.set_xlabel("Frequency (Hz)")
    ax_phase.set_ylabel("Phase (deg)")
    ax_phase.grid(True, alpha=0.3)

    fig.suptitle("Complex Frequency Response")
    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved plot: {output}")
    else:
        plt.show()


def plot_xy_mode(
    data: np.ndarray,
    start_hz: float,
    stop_hz: float,
    x_axis: str,
    output: Path | None,
) -> None:
    points = xy_sweep_points(data)
    freqs = make_frequency_axis(points.shape[0], start_hz, stop_hz, x_axis)
    x_mag_db, y_mag_db = xy_to_mag_db(data)

    print_data_stats(x_mag_db, "X_mag_db")
    print_data_stats(y_mag_db, "Y_mag_db")

    fig, ax = plt.subplots(figsize=(10, 5.5))
    if x_axis == "log-sweep":
        ax.semilogx(freqs, x_mag_db, label="X input", linewidth=1.1)
        ax.semilogx(freqs, y_mag_db, label="Y output", linewidth=1.1)
    else:
        ax.plot(freqs, x_mag_db, label="X input", linewidth=1.1)
        ax.plot(freqs, y_mag_db, label="Y output", linewidth=1.1)
    ax.set_title("Input and Output FFT Bin Magnitude")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude (dBFS)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved plot: {output}")
    else:
        plt.show()


def plot_freq_xy_mode(
    freq_data: np.ndarray,
    xy_data: np.ndarray,
    start_hz: float,
    stop_hz: float,
    x_axis: str,
    output: Path | None,
) -> None:
    h_mag_db = complex_to_mag_db(freq_data)
    x_mag_db, y_mag_db = xy_to_mag_db(xy_data)
    count = min(h_mag_db.size, x_mag_db.size, y_mag_db.size)
    freqs = make_frequency_axis(count, start_hz, stop_hz, x_axis)

    h_mag_db = h_mag_db[:count]
    x_mag_db = x_mag_db[:count]
    y_mag_db = y_mag_db[:count]

    print_data_stats(h_mag_db, "H_mag_db")
    print_data_stats(x_mag_db, "X_mag_db")
    print_data_stats(y_mag_db, "Y_mag_db")

    fig, (ax_h, ax_xy) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    plot_fn_h = ax_h.semilogx if x_axis == "log-sweep" else ax_h.plot
    plot_fn_xy = ax_xy.semilogx if x_axis == "log-sweep" else ax_xy.plot

    plot_fn_h(freqs, h_mag_db, linewidth=1.1)
    ax_h.set_title("Frequency Response H = Y / X")
    ax_h.set_ylabel("Magnitude (dB)")
    ax_h.grid(True, alpha=0.3)

    plot_fn_xy(freqs, x_mag_db, label="X input", linewidth=1.0)
    plot_fn_xy(freqs, y_mag_db, label="Y output", linewidth=1.0)
    ax_xy.set_title("Selected FFT Bin Magnitude")
    ax_xy.set_xlabel("Frequency (Hz)")
    ax_xy.set_ylabel("Magnitude (dBFS)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend()

    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved plot: {output}")
    else:
        plt.show()


def plot_adc_mode(data: np.ndarray, output: Path | None) -> None:
    if data.size == 0:
        raise ValueError("No ADC samples received.")

    adc1 = (data & 0x00FF).astype(np.float32)
    adc2 = ((data >> 8) & 0x00FF).astype(np.float32)
    time_us = np.arange(data.size, dtype=np.float32) * (1_000_000.0 / FS_HZ)

    print_data_stats(adc1, "adc1")
    print_data_stats(adc2, "adc2")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    ax1.plot(time_us, adc1, linewidth=0.9)
    ax1.set_title("ADC1 Time Domain")
    ax1.set_ylabel("Code")
    ax1.grid(True, alpha=0.3)

    ax2.plot(time_us, adc2, linewidth=0.9)
    ax2.set_title("ADC2 Time Domain")
    ax2.set_xlabel("Time (us)")
    ax2.set_ylabel("Code")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved plot: {output}")
    else:
        plt.show()


def split_adc_channels(data: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    adc1 = (data & 0x00FF).astype(np.float32)
    adc2 = ((data >> 8) & 0x00FF).astype(np.float32)
    time_us = np.arange(data.size, dtype=np.float32) * (1_000_000.0 / FS_HZ)
    return time_us, adc1, adc2


def plot_both_mode(
    adc_data: np.ndarray,
    freq_data: np.ndarray,
    start_hz: float,
    stop_hz: float,
    x_axis: str,
    output: Path | None,
) -> None:
    time_us, adc1, adc2 = split_adc_channels(adc_data)
    freq_mag_db = complex_to_mag_db(freq_data) if freq_data.size >= 2 * SWEEP_POINTS else freq_data
    freqs = make_frequency_axis(freq_mag_db.size, start_hz, stop_hz, x_axis)

    print_data_stats(adc1, "adc1")
    print_data_stats(adc2, "adc2")
    print_data_stats(freq_mag_db, "mag_db")

    fig, (ax_adc1, ax_adc2, ax_freq) = plt.subplots(3, 1, figsize=(11, 8), sharex=False)
    ax_adc1.plot(time_us, adc1, linewidth=0.8)
    ax_adc1.set_title("ADC1 Time Domain")
    ax_adc1.set_ylabel("Code")
    ax_adc1.grid(True, alpha=0.3)

    ax_adc2.plot(time_us, adc2, linewidth=0.8)
    ax_adc2.set_title("ADC2 Time Domain")
    ax_adc2.set_xlabel("Time (us)")
    ax_adc2.set_ylabel("Code")
    ax_adc2.grid(True, alpha=0.3)

    if x_axis == "log-sweep":
        ax_freq.semilogx(freqs, freq_mag_db, linewidth=1.0)
    else:
        ax_freq.plot(freqs, freq_mag_db, linewidth=1.0)
    ax_freq.set_title("Frequency Response")
    ax_freq.set_xlabel("Frequency (Hz)")
    ax_freq.set_ylabel("Magnitude (dB)")
    ax_freq.grid(True, alpha=0.3)

    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=160)
        print(f"Saved plot: {output}")
    else:
        plt.show()


def save_csv(
    data: np.ndarray,
    mode: str,
    frame: str,
    path: Path,
    start_hz: float,
    stop_hz: float,
    x_axis: str,
) -> None:
    if frame == "adc":
        adc1 = data & 0x00FF
        adc2 = (data >> 8) & 0x00FF
        time_us = np.arange(data.size, dtype=np.float32) * (1_000_000.0 / FS_HZ)
        rows = np.column_stack([time_us, adc1, adc2])
        header = "time_us,adc1,adc2"
    elif frame == "xy":
        points = xy_sweep_points(data)
        freqs = make_frequency_axis(points.shape[0], start_hz, stop_hz, x_axis)
        x = points[:, 0] + 1j * points[:, 1]
        y = points[:, 2] + 1j * points[:, 3]
        rows = np.column_stack(
            [
                freqs,
                points[:, 0],
                points[:, 1],
                points[:, 2],
                points[:, 3],
                20.0 * np.log10(np.abs(x) + 1e-12),
                20.0 * np.log10(np.abs(y) + 1e-12),
            ]
        )
        header = "freq_hz,xr,xi,yr,yi,x_mag_db,y_mag_db"
    elif mode == "mag":
        mag_db = complex_to_mag_db(data) if data.size >= 2 * SWEEP_POINTS else data
        count = mag_db.size
        freqs = make_frequency_axis(count, start_hz, stop_hz, x_axis)
        rows = np.column_stack([freqs, mag_db])
        header = "freq_hz,mag_db"
    else:
        complex_bins = complex_sweep_points(data)
        response = complex_bins[:, 0] + 1j * complex_bins[:, 1]
        freqs = make_frequency_axis(complex_bins.shape[0], start_hz, stop_hz, x_axis)
        rows = np.column_stack(
            [
                freqs,
                complex_bins[:, 0],
                complex_bins[:, 1],
                20.0 * np.log10(np.abs(response) + 1e-12),
                np.angle(response, deg=True),
            ]
        )
        header = "freq_hz,re,im,mag_db,phase_deg"

    np.savetxt(path, rows, delimiter=",", header=header, comments="")
    print(f"Saved CSV: {path}")


def print_data_stats(values: np.ndarray, name: str) -> None:
    finite = values[np.isfinite(values)]
    nonfinite_count = values.size - finite.size
    if finite.size == 0:
        print(f"{name}: no finite values; frame is probably not aligned.")
        return

    large_count = int(np.count_nonzero(np.abs(finite) > 1.0e4))
    print(
        f"{name}: min={finite.min():.6g}, max={finite.max():.6g}, "
        f"nonfinite={nonfinite_count}, abs>1e4={large_count}"
    )

    if large_count or nonfinite_count:
        print(
            "Warning: decoded values are far outside a normal frequency-response range. "
            "This usually means the binary frame started from the middle of a UART packet."
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read STM32 freq_response binary float32 data from UART and plot it."
    )
    parser.add_argument("-p", "--port", help="Serial port, for example COM5.")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="UART baud rate.")
    parser.add_argument("--timeout", type=float, default=10.0, help="Serial read timeout in seconds.")
    parser.add_argument("--frame", choices=["freq", "xy", "adc", "all", "both"], default="freq",
                        help="freq reads FSRT; xy reads one XYRT record; all captures XYRT during sweep and then FSRT, or falls back to FSRT if XYRT was already missed; adc reads ADCT; both reads ADCT then FSRT.")
    parser.add_argument("--mode", choices=["mag", "complex"], default="mag",
                        help="mag reads one magnitude value per swept frequency point.")
    parser.add_argument("--start-hz", type=float, default=1000.0)
    parser.add_argument("--stop-hz", type=float, default=10000.0)
    parser.add_argument("--x-axis", choices=["log-sweep", "fft-bin"], default="log-sweep",
                        help="log-sweep maps received points onto a logarithmic sweep axis; fft-bin uses FFT bin spacing.")
    parser.add_argument("-o", "--output", type=Path, help="Save plot image instead of opening a window.")
    parser.add_argument("--csv", type=Path, help="Also save decoded data to CSV.")
    parser.add_argument("--no-reset-input", action="store_true",
                        help="Do not clear pending serial input before reading.")
    parser.add_argument("--sync-gap", type=float, default=0.0,
                        help="Raw mode only: drain input until the UART stream is idle for this many seconds, then read one frame.")
    parser.add_argument("--raw", action="store_true",
                        help="Read one fixed-size freq_response payload without the FSRT header.")
    parser.add_argument("--header-timeout", type=float, default=30.0,
                        help="Seconds to search for the framed protocol header before failing.")
    parser.add_argument("--list-ports", action="store_true", help="List serial ports and exit.")
    parser.add_argument("--dump-bytes", type=int, metavar="N",
                        help="Print the next N raw serial bytes as hex/ascii and exit.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_ports:
        list_serial_ports()
        return 0

    if not args.port:
        print("Missing --port. Use --list-ports to see available ports.", file=sys.stderr)
        return 2

    if args.dump_bytes:
        dump_serial_bytes(args.port, args.baud, args.timeout, args.dump_bytes)
        return 0

    if args.frame == "all":
        if args.raw:
            raise ValueError("--frame all requires framed UART data; do not use --raw.")
        freq_data, xy_data = load_freq_xy_frames(
            port=args.port,
            baud=args.baud,
            timeout=args.timeout,
            reset_input=not args.no_reset_input,
            header_timeout=args.header_timeout,
        )
        print(
            f"Received {freq_data.size} freq float32 values and "
            f"{xy_data.size} X/Y float32 values."
        )

        if args.csv:
            freq_csv = args.csv.with_name(f"{args.csv.stem}_freq{args.csv.suffix}")
            save_csv(freq_data, args.mode, "freq", freq_csv, args.start_hz, args.stop_hz, args.x_axis)
            if xy_data.size:
                xy_csv = args.csv.with_name(f"{args.csv.stem}_xy{args.csv.suffix}")
                save_csv(xy_data, args.mode, "xy", xy_csv, args.start_hz, args.stop_hz, args.x_axis)
            else:
                print("No XYRT records captured; skipped XY CSV.")

        if xy_data.size:
            plot_freq_xy_mode(freq_data, xy_data, args.start_hz, args.stop_hz, args.x_axis, args.output)
        else:
            print("No XYRT records captured; plotting frequency response only.")
            plot_mag_mode(freq_data, args.start_hz, args.stop_hz, args.x_axis, args.output)
        return 0

    if args.frame == "both":
        if args.raw:
            raise ValueError("--frame both requires framed UART data; do not use --raw.")
        adc_data, freq_data = load_both_frames(
            port=args.port,
            baud=args.baud,
            timeout=args.timeout,
            reset_input=not args.no_reset_input,
            header_timeout=args.header_timeout,
        )
        print(f"Received {adc_data.size} uint16 ADC samples and {freq_data.size} float32 values.")

        if args.csv:
            adc_csv = args.csv.with_name(f"{args.csv.stem}_adc{args.csv.suffix}")
            freq_csv = args.csv.with_name(f"{args.csv.stem}_freq{args.csv.suffix}")
            save_csv(adc_data, args.mode, "adc", adc_csv, args.start_hz, args.stop_hz, args.x_axis)
            save_csv(freq_data, args.mode, "freq", freq_csv, args.start_hz, args.stop_hz, args.x_axis)

        plot_both_mode(adc_data, freq_data, args.start_hz, args.stop_hz, args.x_axis, args.output)
        return 0

    data = load_uart_frame(
        port=args.port,
        baud=args.baud,
        timeout=args.timeout,
        reset_input=not args.no_reset_input,
        sync_gap=args.sync_gap,
        raw=args.raw,
        header_timeout=args.header_timeout,
        frame=args.frame,
    )

    unit = "uint16 ADC samples" if args.frame == "adc" else "float32 values"
    print(f"Received {data.size} {unit}.")

    if args.csv:
        save_csv(data, args.mode, args.frame, args.csv, args.start_hz, args.stop_hz, args.x_axis)

    if args.frame == "adc":
        plot_adc_mode(data, args.output)
    elif args.frame == "xy":
        plot_xy_mode(data, args.start_hz, args.stop_hz, args.x_axis, args.output)
    elif args.mode == "mag":
        plot_mag_mode(data, args.start_hz, args.stop_hz, args.x_axis, args.output)
    else:
        plot_complex_mode(data, args.start_hz, args.stop_hz, args.x_axis, args.output)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
