import numpy as np
import matplotlib.pyplot as plt

Fs = 10000000.0
F = 100_000
t = np.arange(0,0.001,1/Fs)

A = [0,0.1,0.05,0.15,0,0,0,0,0,0]


y = 0
for i in range(9):
    y += A[i]*np.sin(2*np.pi*i*F*t)

rms = np.sqrt(np.mean(y**2))
print(f"数值法 RMS: {rms:.6f}")

Vpp = np.max(y) - np.min(y)
print(f"Vpp:{Vpp:.6f}")

plt.plot(t,y)
plt.show()