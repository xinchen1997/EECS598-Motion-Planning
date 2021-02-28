import numpy as np
import matplotlib.pyplot as plt

x_axis_max = 310
x_axis_min = 50
y_axis_min = 10.5
y_axis_max = 10.8

A_time = [69.285074, 69.285074, x_axis_max]
A_cost = [y_axis_max, 10.6901705374, 10.6901705374]

ANA_time = [92.301352, 92.301352, 214.969208, 265.220737, 305.019734, x_axis_max]
ANA_cost = [y_axis_max, 10.666723023, 10.6315918937, 10.5730132499, 10.5730132499, 10.5730132499]

plt.figure()

plt.plot(A_time, A_cost, color='red', linewidth=2, label='A$^*$')
plt.plot(ANA_time, ANA_cost, color='blue', linewidth=2, label='ANA$^*$')

plt.xlabel('Time (s)')
plt.ylabel('Solution Cost')
plt.title('Solution Path Cost vs. Time')
plt.xlim([x_axis_min, x_axis_max])
plt.ylim([y_axis_min, y_axis_max])
plt.legend()

plt.show()