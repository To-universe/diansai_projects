import math

FFT_SIZE =  4096
Q15_SCALE = 32768

def float_to_q15(value: float) -> int:
    q15_value = int(round(value * Q15_SCALE))
    if q15_value > 32767:
        return 32767
    if q15_value < -32768:
        return -32768
    return q15_value

def make_hann_q15(size: int) -> list[int]:
    return [
        float_to_q15(0.5 * (1.0 - math.cos(2.0 * math.pi * n / (size - 1))))
        for n in range(size)
    ]

def main()->None:
    values = make_hann_q15(FFT_SIZE)
    window_sum = sum(values)

    print(window_sum)

if __name__ == "__main__":
    main()