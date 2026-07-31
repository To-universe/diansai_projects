#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import numpy as np


def read_exact(serial_port, size):
    data = bytearray()
    while len(data) < size:
        chunk = serial_port.read(size - len(data))
        if not chunk:
            raise TimeoutError(f"timeout after reading {len(data)} of {size} bytes")
        data += chunk
        print(f"\rreceived {len(data)}/{size} bytes", end="", flush=True)
    print()
    return bytes(data)


def decode_int16_payload(payload, endian):
    if len(payload) % 2 != 0:
        raise ValueError("payload byte count must be even for int16 samples")

    dtype = np.dtype("<i2" if endian == "little" else ">i2")
    return np.frombuffer(payload, dtype=dtype).copy()


def plot_waveform(samples, fs, volt_per_code, remove_dc, save, no_show):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is not installed. Install it with: python -m pip install matplotlib"
        ) from exc

    y = samples.astype(np.float64)
    ylabel = "FPGA sample code"
    if volt_per_code is not None:
        y = y * volt_per_code
        ylabel = "Voltage (V)"

    mean_value = float(np.mean(y))
    if remove_dc:
        y_plot = y - mean_value
        ylabel += " (DC removed)"
    else:
        y_plot = y

    if fs is None:
        x = np.arange(len(samples))
        xlabel = "Sample index"
    else:
        x = np.arange(len(samples)) / fs * 1e6
        xlabel = "Time (us)"

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, y_plot, linewidth=1.0)
    ax.set_title("FPGA sampled waveform")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.35)

    info = (
        f"samples: {len(samples)}\n"
        f"raw min: {int(samples.min())}\n"
        f"raw max: {int(samples.max())}\n"
        f"raw mean: {float(samples.mean()):.2f}\n"
        f"raw pp: {int(samples.max()) - int(samples.min())}"
    )
    if volt_per_code is not None:
        info += (
            f"\nmin: {float(y.min()):.6f} V"
            f"\nmax: {float(y.max()):.6f} V"
            f"\nmean: {mean_value:.6f} V"
            f"\nVpp: {float(y.max() - y.min()):.6f} V"
        )

    ax.text(
        0.99,
        0.98,
        info,
        transform=ax.transAxes,
        va="top",
        ha="right",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=160)
        print(f"saved plot: {save}")
    if not no_show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Receive one raw 4096-point signed-int16 FPGA frame from STM32 UART."
    )
    parser.add_argument("--port", default="COM20", help="serial port, default: COM20")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate, default: 115200")
    parser.add_argument("--count", type=int, default=4096, help="number of int16 samples")
    parser.add_argument("--timeout", type=float, default=10.0, help="serial timeout in seconds")
    parser.add_argument(
        "--fs",
        type=float,
        default=2.047e6,
        help="sampling rate in Hz for the time axis, default: 2.047e6",
    )
    parser.add_argument(
        "--endian",
        choices=("little", "big"),
        default="little",
        help="UART int16 byte order, default: little",
    )
    parser.add_argument(
        "--volt-per-code",
        type=float,
        default=None,
        help="optional voltage scale. If omitted, plot raw signed codes.",
    )
    parser.add_argument("--remove-dc", action="store_true", help="remove mean value in the plot")
    parser.add_argument("--bin", default="fpga_data_uart_i16.bin", help="raw binary output path")
    parser.add_argument("--csv", default=None, help="optional CSV output path")
    parser.add_argument("--plot", default=None, help="optional PNG output path")
    parser.add_argument("--no-show", action="store_true", help="do not show the plot window")
    parser.add_argument(
        "--keep-buffer",
        action="store_true",
        help="do not clear the serial input buffer before reading",
    )
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is not installed. Install it with: python -m pip install pyserial") from exc

    payload_size = args.count * 2
    with serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1, timeout=args.timeout) as ser:
        if not args.keep_buffer:
            ser.reset_input_buffer()
        print(f"waiting for {payload_size} bytes on {args.port} @ {args.baud}...")
        started = time.perf_counter()
        payload = read_exact(ser, payload_size)
        elapsed = time.perf_counter() - started

    samples = decode_int16_payload(payload, args.endian)
    Path(args.bin).write_bytes(payload)
    print(f"saved raw data: {args.bin}")
    print(f"received {len(samples)} samples in {elapsed:.3f}s")
    print(
        f"min={int(samples.min())}, max={int(samples.max())}, "
        f"mean={float(samples.mean()):.2f}, pp={int(samples.max()) - int(samples.min())}"
    )

    if args.csv:
        with open(args.csv, "w", encoding="ascii") as f:
            if args.volt_per_code is None:
                f.write("index,code\n")
                for index, sample in enumerate(samples):
                    f.write(f"{index},{int(sample)}\n")
            else:
                f.write("index,code,voltage\n")
                for index, sample in enumerate(samples):
                    voltage = float(sample) * args.volt_per_code
                    f.write(f"{index},{int(sample)},{voltage:.9f}\n")
        print(f"saved csv: {args.csv}")

    plot_waveform(
        samples,
        fs=args.fs,
        volt_per_code=args.volt_per_code,
        remove_dc=args.remove_dc,
        save=args.plot,
        no_show=args.no_show,
    )


if __name__ == "__main__":
    main()
