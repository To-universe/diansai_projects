#!/usr/bin/env python3
import argparse
import struct
import time
from pathlib import Path
import numpy as np

HEADER = b"ADC4096_LE_U16\n"
FOOTER = b"\nEND\n"


def read_until(serial_port, marker):
    window = bytearray()
    while True:
        byte = serial_port.read(1)
        if not byte:
            raise TimeoutError(f"timeout while waiting for {marker!r}")

        window += byte
        if len(window) > len(marker):
            del window[0 : len(window) - len(marker)]

        if bytes(window) == marker:
            return


def read_exact(serial_port, size):
    data = bytearray()
    while len(data) < size:
        chunk = serial_port.read(size - len(data))
        if not chunk:
            raise TimeoutError(f"timeout after reading {len(data)} of {size} bytes")
        data += chunk
    return bytes(data)


def read_adc_payload(serial_port, payload_bytes=None, idle_timeout=0.3):
    read_until(serial_port, HEADER)

    old_timeout = serial_port.timeout
    serial_port.timeout = 0.02
    first = serial_port.read(1)

    chunks = []
    if first and first != b"\x00":
        chunks.append(first)

    if payload_bytes is not None:
        remain = payload_bytes - sum(len(chunk) for chunk in chunks)
        if remain > 0:
            chunks.append(read_exact(serial_port, remain))
    else:
        serial_port.timeout = idle_timeout
        while True:
            chunk = serial_port.read(4096)
            if not chunk:
                break
            chunks.append(chunk)

    serial_port.timeout = old_timeout

    payload = b"".join(chunks)
    footer_index = payload.find(FOOTER)
    if footer_index >= 0:
        payload = payload[:footer_index]

    if len(payload) % 2 != 0:
        print("warning: odd payload length; dropping the last byte")
        payload = payload[:-1]

    return payload


def compute_rfft_dbv(samples, fs=None, vref=3.3, adc_max=4095, remove_dc=True):
    try:
        import numpy as np
    except ImportError as exc:
        raise SystemExit("numpy is not installed. Install it with: python -m pip install numpy") from exc

    y = np.asarray(samples, dtype=float) * vref / adc_max
    if remove_dc:
        y = y - np.mean(y)

    amplitude = np.abs(np.fft.rfft(y)) / len(y)
    if len(amplitude) > 2:
        amplitude[1:-1] *= 2.0
    amplitude_dbv = 20.0 * np.log10(np.maximum(amplitude, 1e-12))

    if fs is None:
        x = np.arange(len(amplitude_dbv))
    else:
        x = np.fft.rfftfreq(len(y), d=1.0 / fs)

    return x, amplitude_dbv


def plot_samples(samples, vref=None, adc_max=4095, save=None, fs=None, remove_dc=True, waveform_only=False):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is not installed. Install it with: python -m pip install matplotlib"
        ) from exc

    if vref is None:
        vref = 3.3

    volts = np.asarray(samples, dtype=float) * vref / adc_max
    if fs is None:
        time_x = np.arange(len(samples))
        time_label = "Sample index"
    else:
        time_x = np.arange(len(samples)) / fs * 1e6
        time_label = "Time (us)"

    if waveform_only:
        fig, ax_time = plt.subplots(figsize=(11, 5))
        ax_fft = None
    else:
        fft_x, fft_dbv = compute_rfft_dbv(
            samples,
            fs=fs,
            vref=vref,
            adc_max=adc_max,
            remove_dc=remove_dc,
        )
        fft_dbv_mean = float(np.mean(fft_dbv))
        fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(11, 8))

    ax_time.plot(time_x, volts, linewidth=1.0)
    ax_time.set_title("ADC sampled waveform")
    ax_time.set_xlabel(time_label)
    ax_time.set_ylabel("Voltage (V)")
    ax_time.grid(True, linestyle="--", alpha=0.35)

    if ax_fft is not None:
        ax_fft.plot(fft_x, fft_dbv, linewidth=1.0)
        ax_fft.axhline(fft_dbv_mean, linestyle="--", linewidth=1.0, label=f"mean {fft_dbv_mean:.1f} dBV")
        ax_fft.set_title("RFFT amplitude spectrum")
        ax_fft.set_xlabel("Frequency (Hz)" if fs is not None else "FFT bin")
        ax_fft.set_ylabel("Amplitude (dBV)")
        ax_fft.grid(True, linestyle="--", alpha=0.35)
        ax_fft.legend(loc="best")

    text = (
        f"samples: {len(samples)}\n"
        f"min: {min(samples)}\n"
        f"max: {max(samples)}\n"
        f"avg: {sum(samples) / len(samples):.2f}"
    )
    if vref is not None:
        volts = [sample * vref / adc_max for sample in samples]
        text += (
            f"\nmin V: {min(volts):.4f}"
            f"\nmax V: {max(volts):.4f}"
            f"\navg V: {sum(volts) / len(volts):.4f}"
        )

    ax_time.text(
        0.99,
        0.98,
        text,
        transform=ax_time.transAxes,
        va="top",
        ha="right",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )
    fig.tight_layout()

    if save:
        fig.savefig(save, dpi=160)
        print(f"saved plot: {save}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Receive one 4096-point ADC frame from STM32 UART.")
    parser.add_argument("--port", default="COM20", help="serial port, default: COM20")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate, default: 115200")
    parser.add_argument("--count", type=int, default=4096, help="number of uint16 ADC samples")
    parser.add_argument("--timeout", type=float, default=10.0, help="serial timeout in seconds")
    parser.add_argument("--vref", type=float, default=3.3, help="ADC reference voltage")
    parser.add_argument("--adc-max", type=int, default=4095, help="ADC full-scale code")
    parser.add_argument("--fs", type=float, default=None, help="ADC sampling rate in Hz for the FFT frequency axis")
    parser.add_argument("--keep-dc", action="store_true", help="do not remove DC before calculating RFFT")
    parser.add_argument("--payload-bytes", type=int, default=None, help="raw payload bytes to read after the frame header")
    parser.add_argument("--idle-timeout", type=float, default=0.3, help="end frame after this many idle seconds")
    parser.add_argument("--waveform-only", action="store_true", help="only plot the sampled waveform")
    parser.add_argument("--bin", default="adc_data_uart.bin", help="where to save raw binary samples")
    parser.add_argument("--csv", default=None, help="optional CSV output path")
    parser.add_argument("--plot", default=None, help="optional PNG output path")
    parser.add_argument("--no-show", action="store_true", help="do not show plot window")
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is not installed. Install it with: python -m pip install pyserial") from exc

    with serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1, timeout=args.timeout) as ser:
        ser.reset_input_buffer()
        print(f"waiting for ADC frame on {args.port} @ {args.baud}...")
        started = time.perf_counter()
        payload = read_adc_payload(
            ser,
            payload_bytes=args.payload_bytes,
            idle_timeout=args.idle_timeout,
        )
        elapsed = time.perf_counter() - started

    if not payload:
        raise SystemExit("no ADC payload received")

    samples = list(struct.unpack(f"<{len(payload) // 2}H", payload))
    if len(samples) != args.count:
        print(f"warning: expected {args.count} samples, received {len(samples)} samples")

    Path(args.bin).write_bytes(payload)
    print(f"saved raw data: {args.bin}")
    print(f"received {len(samples)} samples in {elapsed:.3f}s")
    print(f"min={min(samples)}, max={max(samples)}, avg={sum(samples) / len(samples):.2f}")

    if args.csv:
        with open(args.csv, "w", encoding="ascii") as f:
            f.write("index,adc_code,voltage\n")
            for index, sample in enumerate(samples):
                voltage = sample * args.vref / args.adc_max
                f.write(f"{index},{sample},{voltage:.6f}\n")
        print(f"saved csv: {args.csv}")

    if not args.no_show or args.plot:
        plot_samples(
            samples,
            vref=args.vref,
            adc_max=args.adc_max,
            save=args.plot,
            fs=args.fs,
            remove_dc=not args.keep_dc,
            waveform_only=args.waveform_only,
        )


if __name__ == "__main__":
    main()
