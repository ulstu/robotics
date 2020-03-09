import numpy as np
import math
import matplotlib.pyplot as plt

N = 100     # number of samples
a = 0.1     # velocity
sigmaPsi = 1 # model error variance
sigmaEta = 50 # sensor error variance
x, z = np.zeros(N), np.zeros(N)
z[0] = x[0] + np.random.normal(0, sigmaEta)
for t in range(N - 1):
    x[t + 1] = x[t] + a * t + np.random.normal(0, sigmaPsi)
    z[t + 1] = x[t + 1] + np.random.normal(0, sigmaEta)

#kalman filter
xOpt, eOpt = np.zeros(N), np.zeros(N)
xOpt[0] = z[0]
eOpt[0] = sigmaEta # eOpt(t) is a square root of the error variance. It's not a random variable.
k = np.zeros(N)
for t in range(N - 1):
    eOpt[t + 1] = math.sqrt((sigmaEta ** 2) * (eOpt[t] ** 2 + sigmaPsi ** 2) / (sigmaEta ** 2 + eOpt[t] ** 2 + sigmaPsi ** 2))
    k[t + 1] = (eOpt[t + 1]) ** 2 / sigmaEta ** 2
    print(k[t + 1])
    xOpt[t + 1] = (xOpt[t] + a * t) * (1 - k[t + 1]) + k[t + 1] * z[t + 1]
k[0] = k[1]
i = [j for j in range(N)]
ax1 = plt.subplot(211)
ax1.plot(i, xOpt, 'r', label='xOpt')
ax1.plot(i, z, 'b', label = 'z')
ax1.plot(i, x, 'g', label = 'x')
ax1.legend(loc='upper right')
ax2 = plt.subplot(212)
ax2.plot(i, k, 'b')
plt.show()