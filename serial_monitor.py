import matplotlib.pyplot as plt
# plt.switch_backend('Qt5Agg')
# import matplotlib
# matplotlib.use("Qt5agg")
import numpy as np
import serial
import time
import sys

# argv is number of items to expect
if __name__ == '__main__':
	assert(len(sys.argv) == 2)
	n_outputs = int(sys.argv[1])
	print(f'Monitoring {n_outputs} outputs')

	ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
	n_ignore = 11;

	fig, ax = plt.subplots(1, 1, figsize=(10, 8))
	# ax.set_xlim(0, 1100)
	# ax.set_ylim(0, 600)
	fig.show()
	fig.canvas.draw()
	plt.ion()

	scatters = []
	values = []
	for i in range(n_outputs):	
		scatters.append(ax.plot(0, 0, '-')[0])
		values.append([])

	background = fig.canvas.copy_from_bbox(ax.bbox)

	indices = []
	index = 0
	ymin = 0
	ymax = 0

	n_lines = 50
	normalize = False
	try:
		while(n_ignore > 0):
			line = ser.readline()
			print(line)
			if len(line) > 0:
				n_ignore -= 1
		while(True):
			for i in range(n_lines):
				line = ser.readline()
				if line == b'':
					continue

				line_items = list(map(str.strip, line.decode("utf-8").split(" ")))
				print(line_items, len(line_items))
				if len(line_items) == n_outputs:
					try:
						new_vals = []
						for i in range(n_outputs):
							val = float(line_items[i].split(':')[1])
							ymin = min(ymin, val)
							ymax = max(ymax, val)
							new_vals.append(val)

						indices.append(index)
						index += 1
						for i in range(n_outputs):
							values[i].append(new_vals[i])
					except:
						continue
				else:
					print("Incorrect number of items")
			for i in range(n_outputs):
				if normalize:
					minval = min(values[i])
					maxval = max(values[i])
					if minval == maxval:
						scatters[i].set_data(indices, np.zeros(len(values[i])))
					else:
						scatters[i].set_data(indices, (np.array(values[i]) - minval) / (maxval - minval))
					ax.set_xlim(0, index)
					ax.set_ylim(0, 1)
				else:
					scatters[i].set_data(indices, values[i])
					ax.draw_artist(scatters[i])
					ax.set_xlim(0, index)
					ax.set_ylim(ymin, ymax)
			fig.canvas.blit(ax.bbox)
			plt.gcf().canvas.draw_idle()
			plt.gcf().canvas.start_event_loop(0.001)
	except KeyboardInterrupt:
		pass

	ser.close()
	plt.ioff()