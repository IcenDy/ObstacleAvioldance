import numpy as np
import matplotlib.pyplot as plt
# a = np.array([1, 2, 3])
# b = np.ones(3)
# c = 0.5 * np.ones(2)
# d = np.ones((3, 2, 2))
# d[0] += 0.5
# d[:, 1] -= 0.5 
# print(d)
# print(d[:, :, 0])
a = np.ones(3)
b = np.arange(3)
c =  np.linspace(0, 2 * np.pi, 21, True)
theta = np.linspace(0, 2 * np.pi, 21, True)
radius = 0.5
xx = radius * np.cos(theta) + 1
yy = radius * np.sin(theta) + 1
plt.figure()
plt.plot(xx, yy)
plt.show()