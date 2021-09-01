import numpy as np
import matplotlib.pyplot as plt

from math import pi

PULSES_PER_REVOLUTION = 1600


def get_physical_velocity(velocity, frequency):
	ticks_per_pulse = round(2.0 * pi * frequency / (velocity * PULSES_PER_REVOLUTION))
	return 2.0 * pi * frequency / (ticks_per_pulse * PULSES_PER_REVOLUTION)

times = np.linspace(0, pi, 1000)
velocities = np.sin(times)
physical_velocities = list(map(lambda x: get_physical_velocity(x, 1600), velocities))

physical_velocities_2 = list(map(lambda x: get_physical_velocity(x, 50000), velocities))

plt.plot(times, velocities)
slow_label, = plt.plot(times, physical_velocities, label="1.6kHz")
fast_label, = plt.plot(times, physical_velocities_2, label="50kHz")
plt.legend([slow_label, fast_label], ['1.6kHz', '50kHz'])
plt.grid(True)
plt.show()
