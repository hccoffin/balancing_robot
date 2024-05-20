import matplotlib.pyplot as plt
import numpy as np
import scipy

data = np.loadtxt('data_u.txt')
data = data[13000:, :]

print(np.cov(data[:, 0:2].T))

plt.figure(0)
plt.plot(data[:, 0], color='green')
plt.plot(data[:, 1], color='blue')
plt.plot(data[:, 2], color='orange')
plt.show()

print(data.shape)

cur_err = data[:, 1]
print(np.cov(cur_err))
for i in range(data.shape[1] - 2):
    slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(data[:, 2+i], cur_err)
    # plt.scatter(data[:, 2+i], cur_err)
    # plt.scatter(data[:, 2+i], (intercept + slope*data[:, 2+i]))
    # plt.show()
    cur_err = cur_err - (intercept + slope*data[:, 2+i])
    print(i, r_value**2, np.cov(cur_err))

plt.figure(1)
plt.scatter(data[:, 2], data[:, 1], alpha=.03)
plt.show()

# plt.figure(2)
# plt.scatter(data[:, 2], data[:, 0])
# plt.show()