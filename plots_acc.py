import allantools
import matplotlib.pyplot as plt
import numpy as np


gravity = 9.80665
factor = (.001 / 16) * gravity
freq = 1344
xyz_data = np.loadtxt('data_acc_vertical.txt')
xyz_data = xyz_data * factor
xyz_data = xyz_data[100:, :]

print(np.mean(np.sqrt(np.sum(np.square(xyz_data[:, 1:]), axis=1))))

xyz_data2 = np.loadtxt('data_acc_vertical_motors_on_low_tens.txt')
xyz_data2 = xyz_data2 * factor
xyz_data2 = xyz_data2[100:, :]

print(np.mean(np.sqrt(np.sum(np.square(xyz_data2[:, 1:]), axis=1))))

xyz_data3 = np.loadtxt('data_acc_vertical_motors_on.txt')
xyz_data3 = xyz_data3 * factor
xyz_data3 = xyz_data3[100:, :]

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

filtered = np.zeros(xyz_data2.shape)
filtered[:, 0] = notch_filter_data(xyz_data2[:, 0], 244.9*2*np.pi, 2*np.pi*30, 1344)
filtered[:, 1] = notch_filter_data(xyz_data2[:, 1], 244.9*2*np.pi, 2*np.pi*30, 1344)
filtered[:, 2] = notch_filter_data(xyz_data2[:, 2], 244.9*2*np.pi, 2*np.pi*30, 1344)

# xyz_data2 = np.loadtxt('data_acc_vertical_motors_on.txt')
# xyz_data2 = xyz_data2 * factor
# xyz_data2 = xyz_data2[100:, :]

yz_data = filtered[:, 1:]
n = 200
x_cumsum = np.cumsum(filtered[:, 0])
x_rolling_average = np.zeros(filtered.shape[0])
x_rolling_average[0:n] = x_cumsum[0:n] / (np.arange(n) + 1)
x_rolling_average[n:] = (x_cumsum[n:] - x_cumsum[0:-n]) / n
x_zeroed = (filtered[:, 0] - x_rolling_average).reshape(-1, 1)
plt.plot(filtered[:, 0])
plt.plot(x_zeroed)
plt.show()

x_var = np.var(x_zeroed)
print(x_var)
xyz_cov = np.cov(x_zeroed.T, yz_data.T)
print(xyz_cov)
yz_est = yz_data - (xyz_cov[0, 1:] * (1 / xyz_cov[0, 0])).reshape(1, 2) * x_zeroed

print(xyz_cov)
print(xyz_cov[0, 1:])
print(np.cov(yz_data.T))
print(np.cov(yz_est.T))

plt.plot(xyz_data[:, 0], label='x', color='red', alpha=.2)
plt.plot(xyz_data[:, 1], label='y', color='green', alpha=.2)
plt.plot(xyz_data[:, 2], label='z', color='blue', alpha=.2)
plt.plot(xyz_data2[:, 0], label='x', color='red', alpha=.5)
plt.plot(xyz_data2[:, 1], label='y', color='green', alpha=.5)
plt.plot(xyz_data2[:, 2], label='z', color='blue', alpha=.5)
# # plt.plot(yz_est[:, 0], label='y', color='orange', ls='--')
# # plt.plot(yz_est[:, 1], label='z', color='teal', ls='--')
plt.legend()
plt.show()

samples = 2**10
plt.psd(xyz_data[:, 0] - np.mean(xyz_data[:, 0]), samples, freq, color='red', alpha=.25, label='x')
plt.psd(xyz_data[:, 1] - np.mean(xyz_data[:, 1]), samples, freq, color='green', alpha=.25, label='y')
plt.psd(xyz_data[:, 2] - np.mean(xyz_data[:, 2]), samples, freq, color='blue', alpha=.25, label='z')
plt.psd(xyz_data2[:, 0] - np.mean(xyz_data2[:, 0]), samples, freq, color='red', label='x (motor running)')
plt.psd(xyz_data2[:, 1] - np.mean(xyz_data2[:, 1]), samples, freq, color='green', label='y (motor running)')
plt.psd(xyz_data2[:, 2] - np.mean(xyz_data2[:, 2]), samples, freq, color='blue', label='z (motor running)')
# plt.psd(xyz_data3[:, 0] - np.mean(xyz_data3[:, 0]), samples, freq, color='red', label='x high belt tension (motor running)', ls=':')
# plt.psd(xyz_data3[:, 1] - np.mean(xyz_data3[:, 1]), samples, freq, color='green', label='y high belt tension (motor running)', ls=':')
# plt.psd(xyz_data3[:, 2] - np.mean(xyz_data3[:, 2]), samples, freq, color='blue', label='z high belt tension (motor running)', ls=':')
plt.psd(filtered[:, 0] - np.mean(filtered[:, 0]), samples, freq, color='red', label='x filtered (motor running)', ls=':')
plt.psd(filtered[:, 1] - np.mean(filtered[:, 1]), samples, freq, color='green', label='y filtered (motor running)', ls=':')
plt.psd(filtered[:, 2] - np.mean(filtered[:, 2]), samples, freq, color='blue', label='z filtered (motor running)', ls=':')
# plt.xscale('log')
plt.legend()
plt.title('Power Spectral Density LSM303DLHC Acclerometer')
plt.xlabel('Frequency (Hz)')
plt.show()

print(np.cov(xyz_data2.T))
print(np.cov(filtered.T))

(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data[:, 0]) / freq, freq)
plt.loglog(t2, ad, label='x', alpha=.25, color='red')
(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data[:, 1]) / freq, freq)
plt.loglog(t2, ad, label='y', alpha=.25, color='green')
(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data[:, 2]) / freq, freq)
plt.loglog(t2, ad, label='z', alpha=.25, color='blue')

(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data2[:, 0]) / freq, freq)
plt.loglog(t2, ad, label='x (motors on)', color='red')
(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data2[:, 1]) / freq, freq)
plt.loglog(t2, ad, label='y (motors on)', color='green')
(t2, ad, _, _) = allantools.oadev(np.cumsum(xyz_data2[:, 2]) / freq, freq)
plt.loglog(t2, ad, label='z (motors on)', color='blue')

plt.title('Allan Deviation LSM303DLHC Acclerometer')
plt.ylabel('Allan Deviation (m / s^2)')
# plt.title('Allan Deviation L3G4200D Gyro')
# plt.ylabel('Allan Deviation (degrees / s)')

plt.xlabel('Averaging Time (seconds)')
plt.legend()

plt.grid()
plt.show()