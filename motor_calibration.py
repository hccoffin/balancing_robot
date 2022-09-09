import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.interpolate import UnivariateSpline

duty = np.flip(np.load("duty0.npy"))
right = np.flip(np.load("right0.npy"))
left = np.flip(np.load("left0.npy"))
print(duty.shape)

plt.plot(duty, right, color='orange')
plt.plot(duty, left, color='blue')
plt.scatter(duty, right, s=10, color='orange')
plt.scatter(duty, left, s=10, color='blue')
plt.show()

valid = (duty > 190) & (duty < 1000)
duty = duty[valid] - 190
right = right[valid]
left = left[valid]

# def curve(x, a, b):
# 	return a*x / (b + x)#a * ((2 / (1 + np.exp(-(x+c)/b))) - 1)

# # popt, pcov = curve_fit(curve, duty, left, [500, 150, -200])
# popt, pcov = curve_fit(curve, duty, left, bounds=([0, 0], [np.inf, np.inf]))
# print(popt)
samples = np.arange(1, 800)
# left_curve = curve(samples, *[200, 150])
# left_curve = curve(samples, *popt)
# print(samples.shape, left_curve.shape)

# plt.plot(samples, left_curve, color='blue')
# plt.scatter(duty, right, s=10, color='orange')
# plt.scatter(0, 0, s=1)
# plt.scatter(duty, left, s=10, color='blue')
x = 40
y = 460
spl = UnivariateSpline(duty, left, s=10000)
# plt.plot(samples, spl(samples), color='orange')
plt.plot(samples, spl.derivative()(samples), color='orange')
plt.plot(samples, 25/(samples**.8 + 1))
plt.show()