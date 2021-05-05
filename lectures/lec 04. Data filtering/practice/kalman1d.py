import numpy as np
import math
import matplotlib.pyplot as plt

def generate_data():
    x, y = np.zeros(N), np.zeros(N)
    y[0] = x[0] + np.random.normal(0, sigma_sensor)
    for t in range(N - 1):
        x[t + 1] = x[t] + acceleration * t + np.random.normal(0, sigma_model)
        y[t + 1] = x[t + 1] + np.random.normal(0, sigma_sensor)
    return x, y

def filter_kalman(x, y, acceleration):
    xOpt, eOpt = np.zeros(N), np.zeros(N)
    xOpt[0] = y[0]
    eOpt[0] = sigma_sensor  # eOpt(t) is a square root of the error variance. It's not a random variable.
    k = np.zeros(N)
    for t in range(N - 1):
        eOpt[t + 1] = math.sqrt(
            (sigma_sensor ** 2) * (eOpt[t] ** 2 + sigma_model ** 2) / (sigma_sensor ** 2 + eOpt[t] ** 2 + sigma_model ** 2))
        k[t + 1] = (eOpt[t + 1]) ** 2 / sigma_sensor ** 2
        #print(k[t + 1])
        xOpt[t + 1] = (xOpt[t] + acceleration * t) * (1 - k[t + 1]) + k[t + 1] * y[t + 1]
    k[0] = k[1]
    return k, xOpt

def plot_kalman_data(N, x, y, x_filtered, k):
    i = [j for j in range(N)]
    ax1 = plt.subplot(211)
    ax1.plot(i, x_filtered, 'r', label='x filtered')
    ax1.plot(i, y, 'b', label='y')
    ax1.plot(i, x, 'g', label='x')
    ax1.legend(loc='upper right')
    ax2 = plt.subplot(212)
    ax2.plot(i, k, 'b')
    plt.show()

def filter_lowpass(points, alpha = 0.8, eps = 10, stride=1):
    new_points = [points[0]]
    for i in range(1, len(points)):
        p = points[i] + alpha * (new_points[-1] - points[i])
        if (len(new_points) > 2) and (np.mean(new_points[min(i, 0 - stride): -1]) - new_points[-2] < eps):
            new_points.append(p)
        else:
            new_points.append(points[i])
        print(i, points[i])
    return new_points

def plot_lowpass_data(N, x, y, x_filtered):
    i = [j for j in range(N)]
    ax1 = plt.subplot(111)
    ax1.plot(i, x_filtered, 'r', label='x filtered')
    ax1.plot(i, y, 'b', label='y')
    ax1.plot(i, x, 'g', label='x')
    ax1.legend(loc='upper right')
    plt.show()


N = 100  # number of samples
sigma_model = 2    # model error variance
sigma_sensor = 50   # sensor error variance
acceleration = 0.1

x, y = generate_data()
k, xOpt = filter_kalman(x, y, acceleration)
plot_kalman_data(N, x, y, xOpt, k)

#x_lowpass = filter_lowpass(y, alpha=0.8, eps=20, stride=4)
#print('here')
#plot_lowpass_data(N, x, y, x_lowpass)


