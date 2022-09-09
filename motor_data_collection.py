import matplotlib.pyplot as plt
import numpy as np
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
n_ignore = 12;

duty = []
rpm_right = []
rpm_left = []

fig, ax = plt.subplots(1, 1, figsize=(10, 8))
ax.set_xlim(0, 1100)
ax.set_ylim(0, 600)
fig.show()
fig.canvas.draw()

scatter_right = ax.plot(0, 0, 'o', color='orange')[0]
scatter_left = ax.plot(0, 0, 'o', color='blue')[0]
background = fig.canvas.copy_from_bbox(ax.bbox)

try:
	for i in range(n_ignore):
		line = ser.readline()
		print(line)
	while(True):
		line = ser.readline()
		if line == b'':
			continue

		line_items = list(map(str.strip, line.decode("utf-8").split(" ")))
		print(line_items, len(line_items))
		if len(line_items) == 6:
			i, d, r, l, _, _ = map(lambda s: float(s), line_items)
			r = np.abs(r)
			l = np.abs(l)
			duty.append(d)
			rpm_right.append(r)
			rpm_left.append(l)

			fig.canvas.restore_region(background)
			scatter_right.set_data(duty, rpm_right);
			scatter_left.set_data(duty, rpm_left);
			ax.draw_artist(scatter_right)
			ax.draw_artist(scatter_left)
			fig.canvas.blit(ax.bbox)
			plt.pause(.01)
except KeyboardInterrupt:
	pass

ser.close()

np.save('duty.npy', np.array(duty))
np.save('right.npy', np.array(rpm_right))
np.save('left.npy', np.array(rpm_left))