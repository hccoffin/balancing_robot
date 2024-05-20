import allantools
import matplotlib.pyplot as plt
import numpy as np


gravity = 9.80665
# factor = (.001 / 16) * gravity
# freq = 1344
factor = 0.00875
freq = 800
xyz_data = np.loadtxt('data_gyr_vertical_motors_on_low_tens.txt')
xyz_data = xyz_data * factor
xyz_data = xyz_data[100:, :]
# xyz_data = np.loadtxt('data_gyr.txt')
# xyz_data = xyz_data * factor
# xyz_data = xyz_data[10000:, :]

xyz_data2 = np.loadtxt('data_gyr_vertical.txt')
xyz_data2 = xyz_data2 * factor
xyz_data2 = xyz_data2[100:, :]

def notch_filter_data(data, w0, ww, freq):
    x = np.zeros(3)
    y = np.zeros(3)
    w0_pw = (2*freq) * np.tan(w0 * .5 / freq)
    alpha = 4 + (w0_pw / freq)**2
    beta = 2 * ww / freq
    filtered = []
    for v in data:
        x[2] = x[1]
        x[1] = x[0]
        x[0] = v
        y[2] = y[1]
        y[1] = y[0]
        y0 = (alpha * x[0] + 2*x[1]*(alpha-8) + alpha*x[2] - 2*y[1]*(alpha-8) - y[2]*(alpha-beta)) / (alpha + beta)
        y[0] = y0
        filtered.append(y0)
    return np.array(filtered)

filtered = np.zeros(xyz_data.shape)
filtered[:, 0] = notch_filter_data(xyz_data[:, 0], 320*2*np.pi, 2*np.pi*5, 800)
filtered[:, 1] = notch_filter_data(xyz_data[:, 1], 320*2*np.pi, 2*np.pi*5, 800)
filtered[:, 2] = notch_filter_data(xyz_data[:, 2], 320*2*np.pi, 2*np.pi*5, 800)
filtered[:, 0] = notch_filter_data(filtered[:, 0], 223*2*np.pi, 2*np.pi*8, 800)
filtered[:, 1] = notch_filter_data(filtered[:, 1], 223*2*np.pi, 2*np.pi*8, 800)

print(np.mean(filtered, axis=0))

# yz_data = xyz_data[:, 1:]
# x_zeroed = (xyz_data[:, 0] - np.mean(xyz_data[:, 0])).reshape(-1, 1)

# xyz_cov = np.cov(xyz_data.T)
# yz_est = yz_data - (xyz_cov[0, 1:] * (1 / xyz_cov[0, 0])).reshape(1, 2) * x_zeroed

# print(xyz_cov)
# print(xyz_cov[0, 1:])
# print(np.sqrt(np.diag(np.cov(yz_data.T))))
# print(np.sqrt(np.diag(np.cov(yz_est.T))))

# plt.plot(xyz_data[:, 0], label='x', color='red', alpha=.2)
# plt.plot(xyz_data[:, 1], label='y', color='green', alpha=.2)
# plt.plot(xyz_data[:, 2], label='z', color='blue', alpha=.2)
# plt.plot(filtered[:, 0], label='x', color='red')
# plt.plot(filtered[:, 1], label='y', color='green')
# plt.plot(filtered[:, 2], label='z', color='blue')
# # plt.plot(yz_est[:, 0], label='y', color='orange', ls='--')
# # plt.plot(yz_est[:, 1], label='z', color='teal', ls='--')
# plt.legend()
# plt.show()

samples = 2**10
plt.psd(xyz_data[:, 0] - np.mean(xyz_data[:, 0]), samples, freq, color='red', label='x (motor running)', alpha=.25)
plt.psd(xyz_data[:, 1] - np.mean(xyz_data[:, 1]), samples, freq, color='green', label='y (motor running)', alpha=.25)
plt.psd(xyz_data[:, 2] - np.mean(xyz_data[:, 2]), samples, freq, color='blue', label='z (motor running)', alpha=.25)
plt.psd(filtered[:, 0] - np.mean(filtered[:, 0]), samples, freq, color='red', label='x filtered (motor running)')
plt.psd(filtered[:, 1] - np.mean(filtered[:, 1]), samples, freq, color='green', label='y filtered (motor running)')
plt.psd(filtered[:, 2] - np.mean(filtered[:, 2]), samples, freq, color='blue', label='z filtered (motor running)')
# plt.psd(xyz_data2[:, 0] - np.mean(xyz_data2[:, 0]), samples, freq, color='red', alpha=.25, label='x')
# plt.psd(xyz_data2[:, 1] - np.mean(xyz_data2[:, 1]), samples, freq, color='green', alpha=.25, label='y')
# plt.psd(xyz_data2[:, 2] - np.mean(xyz_data2[:, 2]), samples, freq, color='blue', alpha=.25, label='z')
# plt.xscale('log')
plt.legend()
plt.title('Power Spectral Density L3G4200D Gyro')
plt.xlabel('Frequency (Hz)')
plt.show()

print(np.cov(xyz_data.T))
print(np.cov(filtered.T))

(t2, ad, _, _) = allantools.oadev(np.cumsum(filtered[:, 0]) / freq, freq)
plt.loglog(t2, ad, label='x', color='red')
(t2, ad, _, _) = allantools.oadev(np.cumsum(filtered[:, 1]) / freq, freq)
plt.loglog(t2, ad, label='y', color='green')
(t2, ad, _, _) = allantools.oadev(np.cumsum(filtered[:, 2]) / freq, freq)
plt.loglog(t2, ad, label='z', color='blue')

(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data[:, 0]) / freq, freq)
plt.loglog(t2, ad, label='x unfiltered', alpha=.25, color='red')
(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data[:, 1]) / freq, freq)
plt.loglog(t2, ad, label='y unfiltered', alpha=.25, color='green')
(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data[:, 2]) / freq, freq)
plt.loglog(t2, ad, label='z unfiltered', alpha=.25, color='blue')

# plt.title('Allan Deviation LSM303DLHC Acclerometer')
# plt.ylabel('Allan Deviation (m / s^2)')
plt.title('Allan Deviation L3G4200D Gyro')
plt.ylabel('Allan Deviation (degrees / s)')

plt.xlabel('Averaging Time (seconds)')
plt.legend()

plt.grid()
plt.show()